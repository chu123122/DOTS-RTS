using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct SoftAvoidancePairContribution
{
    public float3 VelocityA;
    public float3 VelocityB;
    public byte ActiveA;
    public byte ActiveB;
}

public struct ParallelBodyStageStatistics
{
    public float Total;
    public float Maximum;
    public float SecondaryTotal;
    public float TertiaryTotal;
    public int Count;
    public int ActivatedCount;
    public int EscapeCount;
}

public struct ActiveIncidentIndexState
{
    public ulong Fingerprint;
    public int PairCount;
    public int BodyCount;
    public byte IsValid;
}

/// <summary>
/// P1-P6 staged parallel path. Expensive independent body loops and the
/// soft-avoidance pair scatter are moved out of the serial coordinator jobs.
/// Topology mutation, repair and deterministic compaction remain serialized.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private const int ParallelBodyBatchSize = 64;
    private const int SoftPairBatchSize = 64;

    public NativeArray<byte> EnvelopeEscapeFlags;
    public NativeArray<ParallelBodyStageStatistics> ParallelBodyStatistics;
    public NativeArray<int> SoftIncidentOffsets;
    public NativeArray<int> SoftIncidentWriteCursors;
    public NativeList<int> SoftIncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> SoftPairContributions;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;

    public JobHandle ScheduleParallelJacobiP1P6(
        NativeReference<ParallelJacobiRuntimeState> runtimeState,
        NativeReference<ParallelJacobiIterationState> iterationState,
        NativeList<JacobiBlockStatistics> blockStatistics,
        JobHandle dependency)
    {
        JobHandle handle = new InitializeP1P6PipelineJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(dependency);

        int substepCount = math.max(1, Configuration.SubstepCount);
        int iterationCount = math.max(1, Configuration.IterationCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;
        if (substepDeltaTime <= 0f)
        {
            return new FinalizeParallelJacobiPipelineJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }

        if (Configuration.EnableTimestepContactSetCache)
        {
            handle = new PrepareTimestepPredictionBodiesJob
            {
                States = States,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                FromSolvedPosition = 0,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);
        }

        handle = new BuildInitialP1P6ContactSetJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(handle);

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            handle = new PrepareBaseVelocityBodiesJob
            {
                States = States,
                SubstepDeltaTime = substepDeltaTime,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            if (!Configuration.EnableTimestepContactSetCache)
            {
                handle = new PrepareTimestepPredictionBodiesJob
                {
                    States = States,
                    Duration = substepDeltaTime,
                    Skin = Configuration.PredictiveSkin,
                    Margin = Configuration.TimestepContactMargin,
                    GridOrigin = GridOrigin,
                    CellRadius = CellRadius,
                    FromSolvedPosition = 1,
                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon
                }.Schedule(States.Length, ParallelBodyBatchSize, handle);
            }

            handle = new ValidateBaseMotionBodiesJob
            {
                States = States,
                EscapeFlags = EnvelopeEscapeFlags,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            handle = new BeginP1P6SubstepJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                BlockStatistics = blockStatistics,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new InitializeSoftAvoidanceBodiesJob
            {
                States = States,
                Grid = Grid,
                GridOrigin = GridOrigin,
                GridDimensions = GridDimensions,
                CellRadius = CellRadius,
                SoftShell = Configuration.SoftAvoidanceShell
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            var evaluateSoftPairsJob = new EvaluateSoftAvoidancePairsJob
            {
                States = States,
                Pairs = SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                SolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftShell = Configuration.SoftAvoidanceShell,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SubstepDeltaTime = substepDeltaTime
            };
            handle = evaluateSoftPairsJob.Schedule(
                SoftAvoidancePairs,
                SoftPairBatchSize,
                handle);

            handle = new ReduceSoftAvoidanceBlocksJob
            {
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                Blocks = blockStatistics.AsDeferredJobArray()
            }.Schedule(blockStatistics, 1, handle);

            handle = new GatherSoftAvoidanceBodiesJob
            {
                States = States,
                Pairs = SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                IncidentOffsets = SoftIncidentOffsets,
                IncidentPairIndices = SoftIncidentPairIndices.AsDeferredJobArray(),
                EscapeFlags = EnvelopeEscapeFlags,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftShell = Configuration.SoftAvoidanceShell,
                ClampToEnvelope = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            handle = new FinalizeP1P6SoftAvoidanceJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                BlockStatistics = blockStatistics
            }.Schedule(handle);

            handle = new PredictUnconstrainedBodiesJob
            {
                States = States,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            handle = new ValidatePredictedContactEnvelopeBodiesJob
            {
                States = States,
                EscapeFlags = EnvelopeEscapeFlags,
                PredictiveSkin = Configuration.PredictiveSkin
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            handle = new FinalizeP1P6PreparedSubstepJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = TimestepContactPairs.AsDeferredJobArray()
            }.Schedule(TimestepContactPairs, SoftPairBatchSize, handle);

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                handle = new BeginP1P6IterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
                    IterationState = iterationState,
                    SubstepIndex = substepIndex
                }.Schedule(handle);

                handle = new SolveWallConstraintBodiesJob
                {
                    States = States,
                    Grid = Grid,
                    GridOrigin = GridOrigin,
                    GridDimensions = GridDimensions,
                    CellRadius = CellRadius,
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics
                }.Schedule(States.Length, ParallelBodyBatchSize, handle);

                handle = new FinalizeP1P6WallIterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
                    IterationState = iterationState,
                    BlockStatistics = blockStatistics,
                    SubstepIndex = substepIndex
                }.Schedule(handle);

                handle = new EvaluateParallelJacobiPairsJob
                {
                    Alpha = Configuration.Compliance /
                            math.max(0.0000001f, substepDeltaTime * substepDeltaTime),
                    SubstepIndex = substepIndex,
                    States = States,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray()
                }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);

                handle = new ReduceParallelJacobiBlocksJob
                {
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);

                handle = new GatherAndApplyParallelJacobiBodiesJob
                {
                    States = States,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    IncidentOffsets = ActiveIncidentOffsets,
                    IncidentPairIndices = ActiveIncidentPairIndices.AsDeferredJobArray(),
                    CorrectedBodyFlags = CorrectedBodyFlags
                }.Schedule(States.Length, ParallelBodyBatchSize, handle);

                handle = new FinalizeParallelJacobiIterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
                    IterationState = iterationState,
                    BlockStatistics = blockStatistics,
                    SubstepIndex = substepIndex,
                    IterationIndex = iterationIndex
                }.Schedule(handle);
            }

            handle = new BeginP1P6FinalizeSubstepJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);

            handle = new ReconstructVelocityBodiesJob
            {
                States = States,
                BodyStatistics = ParallelBodyStatistics,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            handle = new FinalizeP1P6VelocityStatisticsJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }

        return new FinalizeParallelJacobiPipelineJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(handle);
    }

    [BurstCompile]
    private struct InitializeP1P6PipelineJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public void Execute() => Solver.InitializeP1P6Pipeline(RuntimeState);
    }

    [BurstCompile]
    private struct BuildInitialP1P6ContactSetJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public void Execute() => Solver.BuildInitialP1P6ContactSet(RuntimeState);
    }

    [BurstCompile]
    private struct PrepareTimestepPredictionBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        public float Duration;
        public float Skin;
        public float Margin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                return;

            float3 start = FromSolvedPosition != 0
                ? state.PredictedPosition
                : state.CurrentPosition;
            float3 velocity = FromSolvedPosition != 0
                ? state.BasePredictedVelocity
                : CalculateBaseVelocity(state, Duration, GridOrigin, CellRadius);
            if (state.IsSettled)
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = state.CurrentPosition.y;
            float extent = math.max(0f, state.Radius) + math.max(0f, Skin) + math.max(0f, Margin);
            state.TimestepStartPosition = start;
            state.TimestepPredictedPosition = end;
            state.BasePredictedVelocity = velocity;
            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;
            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;
            CalculateInteractionBounds(
                state,
                Skin,
                Margin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out state.TimestepInteractionEnvelopeMin,
                out state.TimestepInteractionEnvelopeMax);
            if (FromSolvedPosition == 0)
                state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct PrepareBaseVelocityBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        public float SubstepDeltaTime;
        public float3 GridOrigin;
        public float CellRadius;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                return;
            state.BasePredictedVelocity = CalculateBaseVelocity(
                state,
                SubstepDeltaTime,
                GridOrigin,
                CellRadius);
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct ValidateBaseMotionBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<FlowMovementFrameState> States;
        public NativeArray<byte> EscapeFlags;
        public byte Enabled;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            if (Enabled == 0)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            CalculateValidationBounds(
                state,
                PredictiveSkin,
                TimestepContactMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out float2 min,
                out float2 max);
            EscapeFlags[bodyIndex] = (byte)(Contains(
                state.TimestepInteractionEnvelopeMin,
                state.TimestepInteractionEnvelopeMax,
                min,
                max) ? 0 : 1);
        }
    }

    [BurstCompile]
    private struct BeginP1P6SubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public NativeList<JacobiBlockStatistics> BlockStatistics;
        public int SubstepIndex;
        public void Execute() => Solver.BeginP1P6Substep(SubstepIndex, RuntimeState, BlockStatistics);
    }

    [BurstCompile]
    private struct InitializeSoftAvoidanceBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<FlowFieldCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public float SoftShell;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            state.SoftAvoidanceVelocity = float3.zero;
            state.WallAvoidanceVelocity = float3.zero;
            state.SoftAvoidanceNeighborCount = 0;
            if (!state.IsInsideGrid)
            {
                States[bodyIndex] = state;
                return;
            }

            int2 currentCell = FlowFieldUtils.WorldToCell(
                state.PredictedPosition,
                GridOrigin,
                CellRadius);
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                        checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                        continue;
                    int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                    if (Grid[checkIndex].Cost != 0)
                        continue;
                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2f + CellRadius,
                        state.PredictedPosition.y,
                        checkCell.y * CellRadius * 2f + CellRadius);
                    float wallRadius = CellRadius + math.max(0f, state.Radius) + math.max(0f, SoftShell);
                    state.WallAvoidanceVelocity += SoftAvoidanceMath.CalculateWallVelocity(
                        state.PredictedPosition,
                        wallPosition,
                        state.MoveSpeed,
                        wallRadius);
                }
            }
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct EvaluateSoftAvoidancePairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<UnitCollisionPair> Pairs;
        public NativeArray<SoftAvoidancePairContribution> Contributions;
        public SoftAvoidanceVelocitySolverMode SolverMode;
        public float SoftShell;
        public float RvoTimeHorizon;
        public float SubstepDeltaTime;

        public void Execute(int pairIndex)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            SoftAvoidancePairContribution result = default;

            float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            delta.y = 0f;
            float maxDistance = bodyA.Radius + bodyB.Radius + math.max(0f, SoftShell);
            if (math.lengthsq(delta) <= maxDistance * maxDistance &&
                SoftAvoidanceMath.TryCalculatePairVelocities(
                    SolverMode,
                    bodyA.PredictedPosition,
                    bodyB.PredictedPosition,
                    bodyA.BasePredictedVelocity,
                    bodyB.BasePredictedVelocity,
                    bodyA.Radius,
                    bodyB.Radius,
                    bodyA.InverseMass,
                    bodyB.InverseMass,
                    bodyA.MoveSpeed,
                    bodyB.MoveSpeed,
                    SoftShell,
                    RvoTimeHorizon,
                    SubstepDeltaTime,
                    DeterministicPairNormal(pair.BodyA, pair.BodyB),
                    out result.VelocityA,
                    out result.VelocityB))
            {
                result.ActiveA = 1;
                result.ActiveB = 1;
            }
            Contributions[pairIndex] = result;
        }
    }

    [BurstCompile]
    private struct ReduceSoftAvoidanceBlocksJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        public NativeArray<JacobiBlockStatistics> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * SoftPairBatchSize;
            int end = math.min(begin + SoftPairBatchSize, Contributions.Length);
            int active = 0;
            for (int i = begin; i < end; i++)
                active += Contributions[i].ActiveA != 0 ? 1 : 0;
            Blocks[blockIndex] = new JacobiBlockStatistics
            {
                NewlyActivatedPairCount = active
            };
        }
    }

    [BurstCompile]
    private struct GatherSoftAvoidanceBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<UnitCollisionPair> Pairs;
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        [ReadOnly] public NativeArray<int> IncidentOffsets;
        [ReadOnly] public NativeArray<int> IncidentPairIndices;
        public NativeArray<byte> EscapeFlags;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftShell;
        public byte ClampToEnvelope;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }

            float3 sum = float3.zero;
            int count = 0;
            int begin = IncidentOffsets[bodyIndex];
            int end = IncidentOffsets[bodyIndex + 1];
            for (int incident = begin; incident < end; incident++)
            {
                int pairIndex = IncidentPairIndices[incident];
                UnitCollisionPair pair = Pairs[pairIndex];
                SoftAvoidancePairContribution contribution = Contributions[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    sum += contribution.VelocityA;
                    count++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    sum += contribution.VelocityB;
                    count++;
                }
            }

            if (count > 0 && SoftSolverMode == SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer)
                sum /= count;
            state.SoftAvoidanceVelocity = sum + state.WallAvoidanceVelocity;
            state.SoftAvoidanceNeighborCount = count;
            float maxSpeed = math.max(0f, state.MoveSpeed);
            if (math.lengthsq(state.SoftAvoidanceVelocity) > maxSpeed * maxSpeed)
                state.SoftAvoidanceVelocity = math.normalizesafe(state.SoftAvoidanceVelocity) * maxSpeed;

            byte escaped = 0;
            if (ClampToEnvelope != 0)
            {
                float3 requested = state.SoftAvoidanceVelocity;
                if (!SoftOutputInsideEnvelope(
                        state,
                        requested,
                        SoftAvoidanceResponseRate,
                        SettledMultiplier,
                        SubstepDeltaTime,
                        PredictiveSkin,
                        TimestepContactMargin,
                        SoftShell))
                {
                    float lower = 0f;
                    float upper = 1f;
                    if (SoftOutputInsideEnvelope(
                            state,
                            float3.zero,
                            SoftAvoidanceResponseRate,
                            SettledMultiplier,
                            SubstepDeltaTime,
                            PredictiveSkin,
                            TimestepContactMargin,
                            SoftShell))
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            float middle = (lower + upper) * 0.5f;
                            if (SoftOutputInsideEnvelope(
                                    state,
                                    requested * middle,
                                    SoftAvoidanceResponseRate,
                                    SettledMultiplier,
                                    SubstepDeltaTime,
                                    PredictiveSkin,
                                    TimestepContactMargin,
                                    SoftShell))
                                lower = middle;
                            else
                                upper = middle;
                        }
                    }
                    state.SoftAvoidanceVelocity = requested * lower;
                    escaped = 1;
                }
            }
            EscapeFlags[bodyIndex] = escaped;
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6SoftAvoidanceJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        [ReadOnly] public NativeList<JacobiBlockStatistics> BlockStatistics;
        public void Execute() => Solver.FinalizeP1P6SoftAvoidance(RuntimeState, BlockStatistics);
    }

    [BurstCompile]
    private struct PredictUnconstrainedBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                return;
            state.StartPosition = state.PredictedPosition;
            state.PreviousSubstepPosition = state.StartPosition;
            state.ContactPositionCorrection = float3.zero;
            state.WallPositionCorrection = float3.zero;
            float response = math.max(0f, SoftAvoidanceResponseRate);
            if (state.IsSettled)
                response *= math.max(0f, SettledMultiplier);
            float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                state.BasePredictedVelocity,
                state.SoftAvoidanceVelocity,
                response,
                SubstepDeltaTime,
                state.MoveSpeed);
            if (state.IsSettled)
                velocity *= math.pow(0.8f, SubstepDeltaTime * 60f);
            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;
            state.PredictedPosition = state.StartPosition + velocity * SubstepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.UnconstrainedPredictedPosition = state.PredictedPosition;
            state.VelocityBeforeContact = velocity;
            state.IntegratedVelocity = velocity;
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct ValidatePredictedContactEnvelopeBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<FlowMovementFrameState> States;
        public NativeArray<byte> EscapeFlags;
        public float PredictiveSkin;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            float extent = math.max(0f, state.Radius) + math.max(0f, PredictiveSkin);
            EscapeFlags[bodyIndex] = (byte)(Contains(
                state.TimestepEnvelopeMin,
                state.TimestepEnvelopeMax,
                state.PredictedPosition.xz - extent,
                state.PredictedPosition.xz + extent) ? 0 : 1);
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6PreparedSubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Solver.FinalizeP1P6PreparedSubstep(SubstepIndex, RuntimeState);
    }

    [BurstCompile]
    private struct ResetContactPairStateJob : IJobParallelForDefer
    {
        public NativeArray<UnitCollisionPair> Pairs;
        public void Execute(int pairIndex)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            Pairs[pairIndex] = pair;
        }
    }

    [BurstCompile]
    private struct BeginP1P6IterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public NativeReference<ParallelJacobiIterationState> IterationState;
        public int SubstepIndex;
        public void Execute() => Solver.BeginP1P6Iteration(SubstepIndex, RuntimeState, IterationState);
    }

    [BurstCompile]
    private struct SolveWallConstraintBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<FlowFieldCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public NativeArray<byte> CorrectedBodyFlags;
        public NativeArray<ParallelBodyStageStatistics> BodyStatistics;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            float total = 0f;
            float maximum = 0f;
            int corrected = 0;
            if (Grid.IsCreated && state.IsInsideGrid && state.InverseMass > 0f)
            {
                int2 currentCell = FlowFieldUtils.WorldToCell(
                    state.PredictedPosition,
                    GridOrigin,
                    CellRadius);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        int2 checkCell = currentCell + new int2(x, y);
                        if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                            checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                            continue;
                        int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                        if (Grid[checkIndex].Cost != 0)
                            continue;
                        float3 wallPosition = GridOrigin + new float3(
                            checkCell.x * CellRadius * 2f + CellRadius,
                            state.PredictedPosition.y,
                            checkCell.y * CellRadius * 2f + CellRadius);
                        float3 delta = state.PredictedPosition - wallPosition;
                        delta.y = 0f;
                        float distance = math.length(delta);
                        float hardDistance = CellRadius + math.max(0f, state.Radius);
                        if (distance >= hardDistance)
                            continue;
                        float3 normal = distance > 0.00001f
                            ? delta / distance
                            : DeterministicPairNormal(bodyIndex, checkIndex);
                        float3 correction = normal * ((hardDistance - distance) * 0.5f);
                        state.PredictedPosition += correction;
                        state.PredictedPosition.y = state.CurrentPosition.y;
                        state.WallPositionCorrection += correction;
                        state.TimestepWallCorrection += correction;
                        float length = math.length(correction);
                        total += length;
                        maximum = math.max(maximum, length);
                        corrected = 1;
                    }
                }
            }
            States[bodyIndex] = state;
            CorrectedBodyFlags[bodyIndex] = (byte)corrected;
            BodyStatistics[bodyIndex] = new ParallelBodyStageStatistics
            {
                Total = total,
                Maximum = maximum,
                Count = corrected
            };
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6WallIterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public NativeReference<ParallelJacobiIterationState> IterationState;
        public NativeList<JacobiBlockStatistics> BlockStatistics;
        public int SubstepIndex;
        public void Execute() => Solver.FinalizeP1P6WallIteration(
            SubstepIndex,
            RuntimeState,
            IterationState,
            BlockStatistics);
    }

    [BurstCompile]
    private struct BeginP1P6FinalizeSubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public void Execute() => Solver.BeginP1P6FinalizeSubstep(RuntimeState);
    }

    [BurstCompile]
    private struct ReconstructVelocityBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        public NativeArray<ParallelBodyStageStatistics> BodyStatistics;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
            {
                BodyStatistics[bodyIndex] = default;
                return;
            }
            state.IntegratedVelocity =
                (state.PredictedPosition - state.PreviousSubstepPosition) / SubstepDeltaTime;
            state.IntegratedVelocity.y = 0f;
            float change = math.distance(state.IntegratedVelocity, state.VelocityBeforeContact);
            BodyStatistics[bodyIndex] = new ParallelBodyStageStatistics
            {
                Total = change,
                Maximum = change,
                SecondaryTotal = math.length(state.VelocityBeforeContact),
                TertiaryTotal = math.length(state.IntegratedVelocity),
                Count = 1
            };
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6VelocityStatisticsJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiRuntimeState> RuntimeState;
        public void Execute() => Solver.FinalizeP1P6VelocityStatistics(RuntimeState);
    }

    private void InitializeP1P6Pipeline(
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        var runtime = new ParallelJacobiRuntimeState
        {
            SolverStartTimestamp = ProfilerUnsafeUtility.Timestamp,
            IsValid = 1
        };
        var statistics = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        IterationDiagnostics.Clear();
        PairDiagnostics.Clear();
        SelectedBodyDiagnostic.Value = default;
        ResetSimulationDebuggerCapture();
        IncrementalStatistics.Value = default;
        Statistics.Value = statistics;
        ActiveIncidentIndexState.Value = default;

        if (DeltaTime / math.max(1, SubstepCount) <= 0f)
            runtime.IsValid = 0;
        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            if (PersistentIncidentPairLookup.IsCreated)
                PersistentIncidentPairLookup.Clear();
            if (PersistentIncidentLookupEpoch.IsCreated)
                PersistentIncidentLookupEpoch.Value = 0;
            IncrementalCacheState.Value = default;
        }
        runtimeState.Value = runtime;
    }

    private void BuildInitialP1P6ContactSet(
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0 || !EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        long start = ProfilerUnsafeUtility.Timestamp;
        BuildTimestepContactSet(ref statistics, ref incremental, false, false);
        statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - start);
        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
    }

    private void BeginP1P6Substep(
        int substepIndex,
        NativeReference<ParallelJacobiRuntimeState> runtimeState,
        NativeList<JacobiBlockStatistics> blockStatistics)
    {
        ParallelJacobiRuntimeState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;

        if (!EnableTimestepContactSetCache)
        {
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepInteractionSet(ref statistics, ref incremental);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }
        else
        {
            ClearIncrementalDirtyBodySet();
            for (int bodyIndex = 0; bodyIndex < EnvelopeEscapeFlags.Length; bodyIndex++)
            {
                if (EnvelopeEscapeFlags[bodyIndex] == 0)
                    continue;
                MarkContactEnvelopeEscape(
                    bodyIndex,
                    substepIndex,
                    IncrementalBodyDirtyFlags.Motion,
                    ref statistics);
            }
            incremental.InteractionEnvelopeEscapeCount += IncrementalDirtyBodies.Length;
            if (IncrementalDirtyBodies.Length > 0)
            {
                RepairOrRebuildContactViewForRemainingTime(
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    true,
                    ref statistics,
                    ref incremental,
                    false);
                RebuildPersistentIncidentPairLookupIfNeededP1P6();
            }
        }

        BuildSoftIncidentIndexP1P6();
        SoftPairContributions.ResizeUninitialized(SoftAvoidancePairs.Length);
        blockStatistics.ResizeUninitialized(
            (SoftAvoidancePairs.Length + SoftPairBatchSize - 1) / SoftPairBatchSize);
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
        runtimeState.Value = runtime;
    }

    private void FinalizeP1P6SoftAvoidance(
        NativeReference<ParallelJacobiRuntimeState> runtimeState,
        NativeList<JacobiBlockStatistics> blocks)
    {
        ParallelJacobiRuntimeState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        int activated = 0;
        for (int i = 0; i < blocks.Length; i++)
            activated += blocks[i].NewlyActivatedPairCount;
        int escaped = 0;
        for (int i = 0; i < EnvelopeEscapeFlags.Length; i++)
            escaped += EnvelopeEscapeFlags[i] != 0 ? 1 : 0;
        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
        statistics.SoftAvoidanceCandidatePairCount += SoftAvoidancePairs.Length;
        statistics.SoftAvoidanceActivatedPairCount += activated;
        statistics.SoftAvoidanceEvaluationCount++;
        statistics.SoftAvoidanceNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        incremental.SoftAvoidancePairEvaluationCount += SoftAvoidancePairs.Length;
        incremental.InteractionEnvelopeEscapeCount += escaped;
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
    }

    private void FinalizeP1P6PreparedSubstep(
        int substepIndex,
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        ParallelJacobiRuntimeState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;

        ClearIncrementalDirtyBodySet();
        for (int bodyIndex = 0; bodyIndex < EnvelopeEscapeFlags.Length; bodyIndex++)
        {
            if (EnvelopeEscapeFlags[bodyIndex] == 0)
                continue;
            MarkContactEnvelopeEscape(
                bodyIndex,
                substepIndex,
                IncrementalBodyDirtyFlags.Motion,
                ref statistics);
        }
        incremental.CorrectedEscapeBodyCount += IncrementalDirtyBodies.Length;
        bool rebuilt = false;
        if (IncrementalDirtyBodies.Length > 0)
        {
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incremental,
                false);
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            rebuilt = true;
        }
        if (!EnableTimestepContactSetCache && !rebuilt)
        {
            // Preserve the reference ordering: first validate the pre-soft swept
            // envelope, then publish the actual solved substep trajectory used by
            // Narrow Phase. Preparing this before validation would make every B0
            // validation trivially pass.
            PrepareSubstepContactPrediction();
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incremental);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }

        ActivateScheduledPredictiveContactsForSubstep(
            EnableTimestepContactSetCache ? substepIndex : 0,
            EnableTimestepContactSetCache ? substepCount : 1,
            ref incremental);
        EnsureActiveConstraintIncidentIndexP1P6();
        statistics.TimestepContactSetSubstepUseCount++;
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
        runtimeState.Value = runtime;
    }

    private void BeginP1P6Iteration(
        int substepIndex,
        NativeReference<ParallelJacobiRuntimeState> runtimeState,
        NativeReference<ParallelJacobiIterationState> iterationState)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        ParallelJacobiIterationState iteration = default;
        if (EnableDiagnostics)
        {
            MeasureContactResidual(
                out iteration.MaxViolationBeforeSolve,
                out iteration.AverageViolationBeforeSolve);
        }
        ResetCorrectedBodyTracking();
        iterationState.Value = iteration;
    }

    private void FinalizeP1P6WallIteration(
        int substepIndex,
        NativeReference<ParallelJacobiRuntimeState> runtimeState,
        NativeReference<ParallelJacobiIterationState> iterationState,
        NativeList<JacobiBlockStatistics> blockStatistics)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        ParallelJacobiIterationState iteration = iterationState.Value;
        CorrectedBodyIndices.Clear();
        for (int bodyIndex = 0; bodyIndex < CorrectedBodyFlags.Length; bodyIndex++)
        {
            ParallelBodyStageStatistics body = ParallelBodyStatistics[bodyIndex];
            iteration.TotalWallPositionCorrection += body.Total;
            iteration.MaxWallPositionCorrection = math.max(
                iteration.MaxWallPositionCorrection,
                body.Maximum);
            if (CorrectedBodyFlags[bodyIndex] != 0)
                CorrectedBodyIndices.Add(bodyIndex);
        }

        if (!ValidateSolverCorrectionContactEnvelope(
                substepIndex,
                ref statistics,
                ref incremental))
        {
            int substepCount = math.max(1, SubstepCount);
            float substepDeltaTime = DeltaTime / substepCount;
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incremental);
            ResetTimestepContactSetForSubstep();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            ActiveIncidentIndexState.Value = default;
            EnsureActiveConstraintIncidentIndexP1P6();
        }

        ResetCorrectedBodyTracking();
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);
        blockStatistics.ResizeUninitialized(
            (TimestepContactPairs.Length + JacobiPairBatchSize - 1) / JacobiPairBatchSize);
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
        iterationState.Value = iteration;
    }

    private void BeginP1P6FinalizeSubstep(
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        ParallelJacobiRuntimeState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        statistics.IterationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        AccumulateConstraintStatistics(ref statistics, ref runtime.PenetrationSum);
        Statistics.Value = statistics;
        runtimeState.Value = runtime;
    }

    private void FinalizeP1P6VelocityStatistics(
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        float speedBefore = 0f;
        float speedAfter = 0f;
        int count = 0;
        for (int bodyIndex = 0; bodyIndex < ParallelBodyStatistics.Length; bodyIndex++)
        {
            ParallelBodyStageStatistics body = ParallelBodyStatistics[bodyIndex];
            statistics.TotalVelocityChange += body.Total;
            statistics.MaxVelocityChange = math.max(statistics.MaxVelocityChange, body.Maximum);
            speedBefore += body.SecondaryTotal;
            speedAfter += body.TertiaryTotal;
            count += body.Count;
        }
        if (count > 0)
        {
            statistics.AverageSpeedBeforeContact += speedBefore / count;
            statistics.AverageSpeedAfterContact += speedAfter / count;
        }
        Statistics.Value = statistics;
    }

    private void BuildSoftIncidentIndexP1P6()
    {
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
            SoftIncidentWriteCursors[bodyIndex] = 0;
        for (int pairIndex = 0; pairIndex < SoftAvoidancePairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = SoftAvoidancePairs[pairIndex];
            SoftIncidentWriteCursors[pair.BodyA]++;
            SoftIncidentWriteCursors[pair.BodyB]++;
        }
        int entries = 0;
        SoftIncidentOffsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            entries += SoftIncidentWriteCursors[bodyIndex];
            SoftIncidentOffsets[bodyIndex + 1] = entries;
            SoftIncidentWriteCursors[bodyIndex] = SoftIncidentOffsets[bodyIndex];
        }
        SoftIncidentPairIndices.ResizeUninitialized(entries);
        for (int pairIndex = 0; pairIndex < SoftAvoidancePairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = SoftAvoidancePairs[pairIndex];
            SoftIncidentPairIndices[SoftIncidentWriteCursors[pair.BodyA]++] = pairIndex;
            SoftIncidentPairIndices[SoftIncidentWriteCursors[pair.BodyB]++] = pairIndex;
        }
    }

    private void EnsureActiveConstraintIncidentIndexP1P6()
    {
        if (ContactPositionSolver != ContactPositionSolverMode.Jacobi)
            return;
        ulong fingerprint = 1469598103934665603UL;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            fingerprint = (fingerprint ^ (uint)pair.BodyA) * 1099511628211UL;
            fingerprint = (fingerprint ^ (uint)pair.BodyB) * 1099511628211UL;
        }
        ActiveIncidentIndexState state = ActiveIncidentIndexState.Value;
        if (state.IsValid != 0 &&
            state.Fingerprint == fingerprint &&
            state.PairCount == TimestepContactPairs.Length &&
            state.BodyCount == States.Length)
            return;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
            ActiveIncidentWriteCursors[bodyIndex] = 0;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            ActiveIncidentWriteCursors[pair.BodyA]++;
            ActiveIncidentWriteCursors[pair.BodyB]++;
        }
        int entries = 0;
        ActiveIncidentOffsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            entries += ActiveIncidentWriteCursors[bodyIndex];
            ActiveIncidentOffsets[bodyIndex + 1] = entries;
            ActiveIncidentWriteCursors[bodyIndex] = ActiveIncidentOffsets[bodyIndex];
        }
        ActiveIncidentPairIndices.ResizeUninitialized(entries);
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            ActiveIncidentPairIndices[ActiveIncidentWriteCursors[pair.BodyA]++] = pairIndex;
            ActiveIncidentPairIndices[ActiveIncidentWriteCursors[pair.BodyB]++] = pairIndex;
        }
        ActiveIncidentIndexState.Value = new ActiveIncidentIndexState
        {
            Fingerprint = fingerprint,
            PairCount = TimestepContactPairs.Length,
            BodyCount = States.Length,
            IsValid = 1
        };
    }

    private void RebuildPersistentIncidentPairLookupIfNeededP1P6()
    {
        if (!EnablePersistentContactCache ||
            !PersistentIncidentPairLookup.IsCreated ||
            !PersistentIncidentLookupEpoch.IsCreated)
            return;
        uint epoch = IncrementalCacheState.Value.TopologyEpoch;
        int requiredEntryCount = PersistentNeighborPairs.Length * 2;
        if (requiredEntryCount > PersistentIncidentPairLookup.Capacity)
        {
            // Never publish a partial incident index. The repair caller detects
            // the invalid epoch and takes the authoritative full-rebuild path.
            PersistentIncidentPairLookup.Clear();
            PersistentIncidentLookupEpoch.Value = uint.MaxValue;
            return;
        }
        if (PersistentIncidentLookupEpoch.Value == epoch &&
            PersistentIncidentPairLookup.Count() == requiredEntryCount)
            return;
        PersistentIncidentPairLookup.Clear();
        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            StableEntityPairKey key = PersistentNeighborPairs[pairIndex].Key;
            PersistentIncidentPairLookup.Add(key.EntityA, pairIndex);
            PersistentIncidentPairLookup.Add(key.EntityB, pairIndex);
        }
        PersistentIncidentLookupEpoch.Value = epoch;
    }

    private static float3 CalculateBaseVelocity(
        FlowMovementFrameState state,
        float deltaTime,
        float3 gridOrigin,
        float cellRadius)
    {
        float3 totalForce = state.IndependentForce;
        if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 center = gridOrigin + new float3(
                state.CellPosition.x * cellRadius * 2f + cellRadius,
                state.CurrentPosition.y,
                state.CellPosition.y * cellRadius * 2f + cellRadius);
            float3 escape = state.PredictedPosition - center;
            escape.y = 0f;
            totalForce += math.normalizesafe(escape, new float3(1f, 0f, 0f)) *
                          state.MoveSpeed * 5f;
        }
        if (math.lengthsq(totalForce) > state.MaxForce * state.MaxForce)
            totalForce = math.normalizesafe(totalForce) * state.MaxForce;
        return state.IntegratedVelocity + totalForce * deltaTime;
    }

    private static bool Contains(float2 outerMin, float2 outerMax, float2 innerMin, float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    private static float3 DeterministicPairNormal(int a, int b)
    {
        return DeterministicFallbackNormal(a, b);
    }

    private static void CalculateInteractionBounds(
        FlowMovementFrameState state,
        float predictiveSkin,
        float margin,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        CalculatePathBounds(
            state,
            softShell,
            softResponseRate,
            softSolverMode,
            rvoTimeHorizon,
            out float2 pathMin,
            out float2 pathMax);
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, margin) * 2f;
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
        min = pathMin - extent;
        max = pathMax + extent;
    }

    private static void CalculateValidationBounds(
        FlowMovementFrameState state,
        float predictiveSkin,
        float margin,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        CalculatePathBounds(
            state,
            softShell,
            softResponseRate,
            softSolverMode,
            rvoTimeHorizon,
            out float2 pathMin,
            out float2 pathMax);
        float contactPadding = math.max(0f, predictiveSkin) + math.max(0f, margin);
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
        min = pathMin - extent;
        max = pathMax + extent;
    }

    private static void CalculatePathBounds(
        FlowMovementFrameState state,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        min = math.min(
            state.TimestepStartPosition.xz,
            math.min(
                state.TimestepPredictedPosition.xz,
                math.min(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        max = math.max(
            state.TimestepStartPosition.xz,
            math.max(
                state.TimestepPredictedPosition.xz,
                math.max(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        if (softSolverMode != SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            softShell <= 0f || softResponseRate <= 0f)
            return;
        float2 horizonEnd = state.PredictedPosition.xz +
                            state.BasePredictedVelocity.xz * math.max(0f, rvoTimeHorizon);
        min = math.min(min, horizonEnd);
        max = math.max(max, horizonEnd);
    }

    private static bool SoftOutputInsideEnvelope(
        FlowMovementFrameState state,
        float3 avoidance,
        float responseRate,
        float settledMultiplier,
        float deltaTime,
        float predictiveSkin,
        float margin,
        float softShell)
    {
        float response = math.max(0f, responseRate);
        if (state.IsSettled)
            response *= math.max(0f, settledMultiplier);
        float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
            state.BasePredictedVelocity,
            avoidance,
            response,
            deltaTime,
            state.MoveSpeed);
        if (state.IsSettled)
            velocity *= math.pow(0.8f, deltaTime * 60f);
        if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
            velocity = math.normalizesafe(velocity) * state.MoveSpeed;
        float3 end = state.PredictedPosition + velocity * deltaTime;
        float contactPadding = math.max(0f, predictiveSkin) + math.max(0f, margin);
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, state.Radius) + math.max(contactPadding, avoidancePadding);
        return Contains(
            state.TimestepInteractionEnvelopeMin,
            state.TimestepInteractionEnvelopeMax,
            end.xz - extent,
            end.xz + extent);
    }
}
}