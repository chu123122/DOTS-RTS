using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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

public struct ParallelBodyStageResult
{
    // EscapeCount is authoritative: it drives dirty-body repair and must exist
    // independently of observation.
    public int EscapeCount;
#if RTS_CONTACT_DIAGNOSTICS
    public float Total;
    public float Maximum;
    public float SecondaryTotal;
    public float TertiaryTotal;
    public int Count;
    public int ActivatedCount;
#else
    public float Total { get => default; set { } }
    public float Maximum { get => default; set { } }
    public float SecondaryTotal { get => default; set { } }
    public float TertiaryTotal { get => default; set { } }
    public int Count { get => default; set { } }
    public int ActivatedCount { get => default; set { } }
#endif
}

public struct ActiveIncidentIndexState
{
    public ulong Fingerprint;
    public int PairCount;
    public int BodyCount;
    public int SoftPairCount;
    public int SoftBodyCount;
    public byte IsValid;
    public byte SoftIsValid;
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
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
    public NativeArray<int> SoftIncidentOffsets;
    public NativeArray<int> SoftIncidentWriteCursors;
    public NativeList<int> SoftIncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> SoftPairContributions;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;

    public JobHandle ScheduleParallelJacobiP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
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
        bool captureSelectedPairs =
            Configuration.EnableDiagnostics && DiagnosticSelectedEntity != Entity.Null &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedPairs) != 0;
        int escapeBlockCount =
            (States.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
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
                PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody = PersistentProxyIndexByBody.AsDeferredJobArray(),
                PersistentCacheState = IncrementalCacheState,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                FromSolvedPosition = 0,
                DetectPersistentDirty = (byte)(Configuration.EnablePersistentContactCache ? 1 : 0),
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialP1P6DirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialP1P6DirtyBodiesJob
                {
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    DirtyBodies = IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialP1P6DirtyBodiesJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsets = SoftIncidentWriteCursors,
                    DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }
        }

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = ScheduleInitialPersistentContactSetP1P6(
                runtimeState,
                handle);
        }
        else
        {
            handle = new BuildInitialP1P6ContactSetJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }

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
                    RvoTimeHorizon = Configuration.RvoTimeHorizon,
                    DetectPersistentDirty = 0
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

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                BodyCount = States.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = SoftIncidentWriteCursors,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                States = States,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = States.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new FinalizeP1P6EnvelopeEscapesJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new PrepareP1P6RepairPredictionBodiesJob
            {
                States = States,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                Duration = math.max(
                    substepDeltaTime,
                    (substepCount - substepIndex) * substepDeltaTime),
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(IncrementalDirtyBodies, ParallelBodyBatchSize, handle);

            handle = new PrepareP1P6SubstepRepairClassificationJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new EvaluatePersistentPairClassificationsP1P6Job
            {
                States = States,
                RawPairs = Pairs.AsDeferredJobArray(),
                PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
                PreviousContacts = PersistentPredictiveContacts.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                PhaseState = PersistentClassificationState,
                Results = PersistentClassificationResults.AsDeferredJobArray(),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                EnablePredictivePairGeneration =
                    (byte)(Configuration.EnablePredictivePairGeneration ? 1 : 0),
                EnablePredictiveContacts =
                    (byte)(Configuration.EnablePredictiveContacts ? 1 : 0),
                SubstepCount = math.max(1, Configuration.SubstepCount),
                ScheduleStartSubstep = substepIndex
            }.Schedule(PersistentClassificationResults, SoftPairBatchSize, handle);

            handle = new CommitP1P6SubstepRepairClassificationJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new PrepareP1P6SoftWorksetJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                BlockStatistics = blockStatistics
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

            handle = new ReduceP1P6SoftEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                EscapeCountsByBlock = SoftIncidentWriteCursors,
                BodyCount = States.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new FinalizeP1P6SoftAvoidanceJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                BlockStatistics = blockStatistics,
                EscapeCountsByBlock = SoftIncidentWriteCursors,
                EscapeBlockCount = escapeBlockCount
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

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                BodyCount = States.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = SoftIncidentWriteCursors,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                States = States,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = States.Length
            }.Schedule(escapeBlockCount, 1, handle);

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

                handle = new CountAndReduceP1P6WallBlocksJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics,
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixP1P6CorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    CorrectedBodyIndices = CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterP1P6CorrectedBodiesJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BlockOffsets = SoftIncidentWriteCursors,
                    CorrectedBodyIndices = CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new FinalizeP1P6WallIterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
                    IterationState = iterationState,
                    BlockStatistics = blockStatistics,
                    SubstepIndex = substepIndex,
                    BodyBlockCount = escapeBlockCount
                }.Schedule(handle);

                if (captureSelectedPairs)
                {
                    handle = new EvaluateParallelJacobiPairsWithDiagnosticsJob
                    {
                        Alpha = Configuration.Compliance /
                                math.max(
                                    0.0000001f,
                                    substepDeltaTime * substepDeltaTime),
                        SubstepIndex = substepIndex,
                        States = States,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                        DiagnosticPairCandidates =
                            ParallelSimulationDebuggerPairCandidates
                                .AsDeferredJobArray(),
                        DiagnosticSelectedEntity = DiagnosticSelectedEntity
                    }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                }
                else
                {
                    handle = new EvaluateParallelJacobiPairsJob
                    {
                        Alpha = Configuration.Compliance /
                                math.max(
                                    0.0000001f,
                                    substepDeltaTime * substepDeltaTime),
                        SubstepIndex = substepIndex,
                        States = States,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray()
                    }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                }

                handle = new ReduceParallelJacobiBlocksJob
                {
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);

                if (captureSelectedPairs)
                {
                    handle = new CountParallelSimulationDebuggerPairBlocksJob
                    {
                        Candidates =
                            ParallelSimulationDebuggerPairCandidates
                                .AsDeferredJobArray(),
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);

                    handle = new PrefixParallelSimulationDebuggerPairsJob
                    {
                        Blocks = blockStatistics.AsDeferredJobArray(),
                        Scratch = ParallelSimulationDebuggerPairScratch
                    }.Schedule(handle);

                    handle = new ScatterParallelSimulationDebuggerPairsJob
                    {
                        Candidates =
                            ParallelSimulationDebuggerPairCandidates.AsDeferredJobArray(),
                        Blocks = blockStatistics.AsDeferredJobArray(),
                        Scratch =
                            ParallelSimulationDebuggerPairScratch.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);

                    handle = new MergeParallelSimulationDebuggerPairsJob
                    {
                        Solver = this
                    }.Schedule(handle);
                }

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

            handle = new ReduceP1P6VelocityBodyBlocksJob
            {
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = States.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new FinalizeP1P6VelocityStatisticsJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                BlockCount = escapeBlockCount
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
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public void Execute() => Solver.InitializeP1P6Pipeline(RuntimeState);
    }

    [BurstCompile]
    private struct BuildInitialP1P6ContactSetJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public void Execute() => Solver.BuildInitialP1P6ContactSet(RuntimeState);
    }

    [BurstCompile]
    private struct PrepareTimestepPredictionBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [NativeDisableParallelForRestriction]
        public NativeArray<PersistentSweptProxy> PersistentProxies;
        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
        [ReadOnly] public NativeReference<IncrementalContactCacheState> PersistentCacheState;
        public NativeArray<byte> DirtyFlagsByBody;
        public float Duration;
        public float Skin;
        public float Margin;
        public float GuardMargin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public byte DetectPersistentDirty;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
            {
                if (DetectPersistentDirty != 0)
                    DirtyFlagsByBody[bodyIndex] = (byte)ClassifyAndUpdatePersistentProxyForBodyP1P6(
                        bodyIndex, state, PersistentProxies, PersistentProxyIndexByBody,
                        PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                        SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
                return;
            }

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
            if (DetectPersistentDirty != 0)
                DirtyFlagsByBody[bodyIndex] = (byte)ClassifyAndUpdatePersistentProxyForBodyP1P6(
                    bodyIndex, state, PersistentProxies, PersistentProxyIndexByBody,
                    PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                    SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
        }
    }

    [BurstCompile]
    private struct CountInitialP1P6DirtyBodyBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                count += DirtyFlagsByBody[bodyIndex] != 0 ? 1 : 0;
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    private struct PrefixInitialP1P6DirtyBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public int BlockCount;
        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterInitialP1P6DirtyBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                IncrementalBodyDirtyFlags flags = (IncrementalBodyDirtyFlags)DirtyFlagsByBody[bodyIndex];
                if (flags == IncrementalBodyDirtyFlags.None)
                    continue;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody { BodyIndex = bodyIndex, Flags = flags };
            }
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
    private struct CountP1P6EnvelopeEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public byte Enabled;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (Enabled != 0 && EscapeFlags[bodyIndex] != 0)
                    count++;
            }
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    private struct PrefixP1P6EnvelopeEscapesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public NativeArray<byte> DirtyFlagsByBody;
        public int BlockCount;

        public void Execute()
        {
            for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)
            {
                int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
                if ((uint)bodyIndex < (uint)DirtyFlagsByBody.Length)
                    DirtyFlagsByBody[bodyIndex] = 0;
            }

            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterP1P6EnvelopeEscapesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<IncrementalDirtyBody> DirtyBodies;
        [NativeDisableParallelForRestriction]
        public NativeArray<byte> DirtyFlagsByBody;
        [NativeDisableParallelForRestriction]
        public NativeArray<FlowMovementFrameState> States;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (EscapeFlags[bodyIndex] == 0)
                    continue;

                const IncrementalBodyDirtyFlags flags =
                    IncrementalBodyDirtyFlags.Motion |
                    IncrementalBodyDirtyFlags.CorrectedEscape;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody
                {
                    BodyIndex = bodyIndex,
                    Flags = flags
                };
                DirtyFlagsByBody[bodyIndex] = (byte)flags;

                FlowMovementFrameState state = States[bodyIndex];
                int newlyEscaped = state.TimestepEscaped == 0 ? 1 : 0;
                state.TimestepEscaped = 1;
                States[bodyIndex] = state;

                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                body.EscapeCount = newlyEscaped;
                BodyStatistics[bodyIndex] = body;
            }
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6EnvelopeEscapesJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Solver.FinalizeP1P6EnvelopeEscapes(SubstepIndex, RuntimeState);
    }

    [BurstCompile]
    private struct PrepareP1P6RepairPredictionBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public float Duration;
        public float Skin;
        public float Margin;
        public float3 GridOrigin;
        public float CellRadius;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;
        public byte Enabled;

        public void Execute(int dirtyIndex)
        {
            if (Enabled == 0)
                return;

            int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex >= (uint)States.Length)
                return;
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                return;

            float3 start = state.PredictedPosition;
            float3 velocity = state.BasePredictedVelocity;
            if (state.IsSettled)
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = state.CurrentPosition.y;
            float extent = math.max(0f, state.Radius) +
                           math.max(0f, Skin) + math.max(0f, Margin);
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
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct PrepareP1P6SubstepRepairClassificationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Solver.PrepareP1P6SubstepRepairClassification(
            SubstepIndex,
            RuntimeState);
    }

    [BurstCompile]
    private struct CommitP1P6SubstepRepairClassificationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Solver.CommitP1P6SubstepRepairClassification(
            SubstepIndex,
            RuntimeState);
    }

    [BurstCompile]
    private struct PrepareP1P6SoftWorksetJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public NativeList<JacobiBlockTelemetry> BlockStatistics;
        public void Execute() => Solver.PrepareP1P6SoftWorkset(RuntimeState, BlockStatistics);
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
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * SoftPairBatchSize;
            int end = math.min(begin + SoftPairBatchSize, Contributions.Length);
            int active = 0;
            for (int i = begin; i < end; i++)
                active += Contributions[i].ActiveA != 0 ? 1 : 0;
            Blocks[blockIndex] = new JacobiBlockTelemetry
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
    private struct ReduceP1P6SoftEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> EscapeCountsByBlock;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int escaped = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                escaped += EscapeFlags[bodyIndex] != 0 ? 1 : 0;
            EscapeCountsByBlock[blockIndex] = escaped;
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6SoftAvoidanceJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        [ReadOnly] public NativeList<JacobiBlockTelemetry> BlockStatistics;
        [ReadOnly] public NativeArray<int> EscapeCountsByBlock;
        public int EscapeBlockCount;
        public void Execute() => Solver.FinalizeP1P6SoftAvoidance(
            RuntimeState,
            BlockStatistics,
            EscapeCountsByBlock,
            EscapeBlockCount);
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
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
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
    private struct CountParallelSimulationDebuggerPairBlocksJob :
        IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<ParallelSimulationDebuggerPairCapture>
            Candidates;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int selectedPairCount = 0;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                selectedPairCount += Candidates[pairIndex].IsValid != 0 ? 1 : 0;
            }

            JacobiBlockTelemetry block = Blocks[blockIndex];
            block.SelectedPairCount = selectedPairCount;
            Blocks[blockIndex] = block;
        }
    }

    [BurstCompile]
    private struct PrefixParallelSimulationDebuggerPairsJob : IJob
    {
        public NativeArray<JacobiBlockTelemetry> Blocks;
        public NativeList<SimulationDebuggerPairSample> Scratch;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
            {
                JacobiBlockTelemetry block = Blocks[blockIndex];
                block.SelectedPairOffset = offset;
                offset += block.SelectedPairCount;
                Blocks[blockIndex] = block;
            }
            Scratch.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterParallelSimulationDebuggerPairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<ParallelSimulationDebuggerPairCapture>
            Candidates;
        [ReadOnly] public NativeArray<JacobiBlockTelemetry> Blocks;
        [NativeDisableParallelForRestriction]
        public NativeArray<SimulationDebuggerPairSample> Scratch;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int writeIndex = Blocks[blockIndex].SelectedPairOffset;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                ParallelSimulationDebuggerPairCapture candidate =
                    Candidates[pairIndex];
                if (candidate.IsValid == 0)
                    continue;
                Scratch[writeIndex++] = candidate.Sample;
            }
        }
    }

    [BurstCompile]
    private struct MergeParallelSimulationDebuggerPairsJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public void Execute() => Solver.MergeParallelSimulationDebuggerPairScratch();
    }

    [BurstCompile]
    private struct BeginP1P6IterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
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
        public NativeArray<ParallelBodyStageResult> BodyStatistics;

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
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
            {
                Total = total,
                Maximum = maximum,
                Count = corrected
            };
        }
    }

    [BurstCompile]
    private struct CountAndReduceP1P6WallBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int correctedCount = 0;
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                correctedCount += CorrectedBodyFlags[bodyIndex] != 0 ? 1 : 0;
            }

            BlockOffsetsAndCounts[blockIndex] = correctedCount;
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }

    [BurstCompile]
    private struct PrefixP1P6CorrectedBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<int> CorrectedBodyIndices;
        public int BlockCount;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            CorrectedBodyIndices.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterP1P6CorrectedBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<int> CorrectedBodyIndices;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (CorrectedBodyFlags[bodyIndex] != 0)
                    CorrectedBodyIndices[writeIndex++] = bodyIndex;
            }
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6WallIterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
        public NativeList<JacobiBlockTelemetry> BlockStatistics;
        public int SubstepIndex;
        public int BodyBlockCount;
        public void Execute() => Solver.FinalizeP1P6WallIteration(
            SubstepIndex,
            RuntimeState,
            IterationState,
            BlockStatistics,
            BodyBlockCount);
    }

    [BurstCompile]
    private struct BeginP1P6FinalizeSubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public void Execute() => Solver.BeginP1P6FinalizeSubstep(RuntimeState);
    }

    [BurstCompile]
    private struct ReconstructVelocityBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
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
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
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
    private struct ReduceP1P6VelocityBodyBlocksJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                aggregate.SecondaryTotal += body.SecondaryTotal;
                aggregate.TertiaryTotal += body.TertiaryTotal;
                aggregate.Count += body.Count;
            }
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }

    [BurstCompile]
    private struct FinalizeP1P6VelocityStatisticsJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public int BlockCount;
        public void Execute() => Solver.FinalizeP1P6VelocityStatistics(
            RuntimeState,
            BlockCount);
    }

    private void InitializeP1P6Pipeline(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        var runtime = new ParallelJacobiExecutionState
        {
            SolverStartTimestamp = ProfilerUnsafeUtility.Timestamp,
            IsValid = 1
        };
        var statistics = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        ResetContactDiagnosticsCapture();
        StoreIncrementalStatistics(default);
        StoreContactStatistics(statistics);
        ActiveIncidentIndexState.Value = default;

        if (DeltaTime / math.max(1, SubstepCount) <= 0f)
            runtime.IsValid = 0;
        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentProxyIndexByBody.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            if (PersistentSpatialMembership.IsCreated)
                PersistentSpatialMembership.Clear();
            if (PersistentSpatialMembershipEpoch.IsCreated)
                PersistentSpatialMembershipEpoch.Value = 0;
            if (PersistentIncidentPairLookup.IsCreated)
                PersistentIncidentPairLookup.Clear();
            if (PersistentIncidentLookupEpoch.IsCreated)
                PersistentIncidentLookupEpoch.Value = 0;
            IncrementalCacheState.Value = default;
        }
        runtimeState.Value = runtime;
    }

    private void BuildInitialP1P6ContactSet(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0 || !EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        long start = ProfilerUnsafeUtility.Timestamp;
        BuildOrRefreshTimestepContactViews(ref statistics, ref incremental, false, false);
        statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - start);
        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void FinalizeP1P6EnvelopeEscapes(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0 || !EnableTimestepContactSetCache)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int newlyEscaped = CountNewlyEscapedP1P6();
        if (newlyEscaped > 0)
        {
            statistics.TimestepContactSetEscapeBodyCount += newlyEscaped;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
        }
        incremental.InteractionEnvelopeEscapeCount += IncrementalDirtyBodies.Length;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private int CountNewlyEscapedP1P6()
    {
        int newlyEscaped = 0;
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            newlyEscaped += ParallelBodyStatistics[bodyIndex].EscapeCount;
        }
        return newlyEscaped;
    }

    private void PrepareP1P6SubstepRepairClassification(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelPersistentClassificationState phase = default;
        PersistentClassificationResults.Clear();
        PersistentClassificationState.Value = phase;

        if (runtimeState.Value.IsValid == 0)
            return;
        if (!EnableTimestepContactSetCache ||
            !EnablePersistentContactCache ||
            IncrementalDirtyBodies.Length == 0)
        {
            RepairP1P6SubstepContactView(substepIndex, runtimeState);
            return;
        }

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        if (!RefreshPreparedIncrementalDirtyBodiesP1P6(
                ref incremental,
                out int topologyDirtyCount))
        {
            incremental.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }
        incremental.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = States.Length > 0
            ? (float)IncrementalDirtyBodies.Length / States.Length
            : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
        {
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }

        long pairDiffStart = ProfilerUnsafeUtility.Timestamp;
        long localBroadPhaseBefore = incremental.LocalBroadPhaseNanoseconds;
        if (topologyDirtyCount > 0)
            IncrementallyRepairPersistentNeighborTopology(ref incremental, false);
        long pairDiffElapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localBroadPhaseElapsed =
            incremental.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
        long pairDiffExclusive = pairDiffElapsed - localBroadPhaseElapsed;
        incremental.PairDiffNanoseconds += pairDiffExclusive > 0L
            ? pairDiffExclusive
            : 0L;

        PreviousTimestepContactPairs.Clear();
        PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies())
        {
            incremental.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }
        incremental.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);

        RemoveDirtyPredictiveContactSchedules();
        PredictiveContactScratch.Clear();
        for (int contactIndex = 0;
             contactIndex < PersistentPredictiveContacts.Length;
             contactIndex++)
        {
            PersistentPredictiveContact contact =
                PersistentPredictiveContacts[contactIndex];
            if (IsDirtyEntity(contact.Key.EntityA) ||
                IsDirtyEntity(contact.Key.EntityB))
                continue;
            PredictiveContactScratch.Add(contact);
        }

        phase.BuildStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        phase.ClassificationStartTimestamp = phase.BuildStartTimestamp;
        phase.Timestep = IncrementalCacheState.Value.Timestep;
        phase.ClassificationEpoch = CalculateClassificationEpoch();
        phase.NeedsCommit = 2;
        PersistentClassificationResults.ResizeUninitialized(Pairs.Length);
        PersistentClassificationState.Value = phase;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void CommitP1P6SubstepRepairClassification(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelPersistentClassificationState phase =
            PersistentClassificationState.Value;
        if (runtimeState.Value.IsValid == 0 || phase.NeedsCommit != 2)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int retainedCount = 0;
        int activeWriteIndex = 0;
        statistics.CandidatePairCount += PersistentClassificationResults.Length;

        for (int pairIndex = 0;
             pairIndex < PersistentClassificationResults.Length;
             pairIndex++)
        {
            PersistentPairClassificationResult result =
                PersistentClassificationResults[pairIndex];
            UnitCollisionPair rawPair = result.RawPair;
            PersistentPredictiveContact contact = result.Contact;
            PredictiveContactScratch.Add(contact);
            if (result.WasReclassified != 0)
            {
                incremental.ReclassifiedPairEvaluationCount++;
                incremental.SweptClassificationEvaluationCount++;
            }
            else
            {
                incremental.ClassificationReuseCount++;
                incremental.ClassificationSkippedCount++;
            }
            AccumulatePersistentClassificationStatistics(contact, ref statistics);

            if (contact.Lifecycle == PersistentContactLifecycle.Expired)
                continue;
            retainedCount++;
            if (contact.Lifecycle == PersistentContactLifecycle.Dormant)
            {
                PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = contact.Key,
                    Substep = contact.NextCheckSubstep
                });
                continue;
            }
            Pairs[activeWriteIndex++] = BuildUnitCollisionPairFromPersistentContact(
                rawPair.BodyA,
                rawPair.BodyB,
                contact);
        }

        Pairs.ResizeUninitialized(activeWriteIndex);
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(
                new PersistentPredictiveContactComparer());
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(
                new PredictiveContactScheduleEntryComparer());
        PredictiveContactScheduleCursor.Value = 0;

        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
        RebuildPersistentContactViews();
        RebuildSoftAvoidancePairSetFromPersistentContacts();
        statistics.ContactPairCount += retainedCount;
        incremental.CurrentInteractionPairCount = PersistentNeighborPairs.Length;
        incremental.CurrentSoftAvoidancePairCount = SoftAvoidancePairs.Length;
        incremental.PersistentViewRebuildCount++;

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.ClassificationEpoch = phase.ClassificationEpoch;
        cacheState.LastUpdateWasFullRebuild = 0;
        cacheState.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = cacheState;

        RebuildEscapedTimestepContactView(ref statistics, ref incremental);
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            FlowMovementFrameState state = States[bodyIndex];
            state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }

        incremental.IncrementalRepairCount++;
        incremental.UsedIncrementalTopology = 1;
        incremental.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        incremental.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - phase.ClassificationStartTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - phase.BuildStartTimestamp);

        InvalidateSoftIncidentIndexP1P6();
        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        phase.NeedsCommit = 0;
        PersistentClassificationState.Value = phase;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private bool TryFindCurrentIncrementalProxyP1P6(
        Entity entity,
        out PersistentSweptProxy proxy,
        out int proxyIndex)
    {
        int low = 0;
        int high = CurrentIncrementalProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = CurrentIncrementalProxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                proxyIndex = middle;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        proxy = default;
        proxyIndex = -1;
        return false;
    }

    private void RepairP1P6SubstepContactView(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;

        if (!EnableTimestepContactSetCache)
        {
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepInteractionAndSoftViews(ref statistics, ref incremental);
            InvalidateSoftIncidentIndexP1P6();
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }
        else if (IncrementalDirtyBodies.Length > 0)
        {
            RepairOrRebuildPreparedContactViewForRemainingTimeP1P6(
                substepIndex,
                ref statistics,
                ref incremental);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void RepairOrRebuildPreparedContactViewForRemainingTimeP1P6(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        int scheduleStartSubstep = substepIndex;
        if (EnablePersistentContactCache &&
            TryIncrementallyRepairEscapedContactSet(
                substepIndex,
                scheduleStartSubstep,
                ref statistics,
                ref incrementalStatistics))
            return;

        BuildOrRefreshTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            true,
            true,
            scheduleStartSubstep);
    }

    private void PrepareP1P6SoftWorkset(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        NativeList<JacobiBlockTelemetry> blockStatistics)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        EnsureSoftIncidentIndexP1P6();
        SoftPairContributions.ResizeUninitialized(SoftAvoidancePairs.Length);
        blockStatistics.ResizeUninitialized(
            (SoftAvoidancePairs.Length + SoftPairBatchSize - 1) / SoftPairBatchSize);
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        runtimeState.Value = runtime;
    }

    private void FinalizeP1P6SoftAvoidance(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        NativeList<JacobiBlockTelemetry> blocks,
        NativeArray<int> escapeCountsByBlock,
        int escapeBlockCount)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int activated = 0;
        for (int i = 0; i < blocks.Length; i++)
            activated += blocks[i].NewlyActivatedPairCount;
        int escaped = 0;
        for (int blockIndex = 0; blockIndex < escapeBlockCount; blockIndex++)
            escaped += escapeCountsByBlock[blockIndex];
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
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void FinalizeP1P6PreparedSubstep(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;

        int newlyEscaped = CountNewlyEscapedP1P6();
        if (newlyEscaped > 0)
        {
            statistics.TimestepContactSetEscapeBodyCount += newlyEscaped;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
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
            InvalidateSoftIncidentIndexP1P6();
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
            InvalidateSoftIncidentIndexP1P6();
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
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
        runtimeState.Value = runtime;
    }

    private void BeginP1P6Iteration(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        NativeReference<ParallelJacobiIterationTelemetry> iterationState)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        ParallelJacobiIterationTelemetry iteration = default;
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
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
        int bodyBlockCount)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        ParallelJacobiIterationTelemetry iteration = iterationState.Value;
        for (int blockIndex = 0; blockIndex < bodyBlockCount; blockIndex++)
        {
            int bodyIndex = blockIndex * ParallelBodyBatchSize;
            ParallelBodyStageResult body = ParallelBodyStatistics[bodyIndex];
            iteration.TotalWallPositionCorrection += body.Total;
            iteration.MaxWallPositionCorrection = math.max(
                iteration.MaxWallPositionCorrection,
                body.Maximum);
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
            InvalidateSoftIncidentIndexP1P6();
            ResetTimestepContactSetForSubstep();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            ActiveIncidentIndexState.Value = default;
            EnsureActiveConstraintIncidentIndexP1P6();
        }

        ResetCorrectedBodyTracking();
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);
        if (ParallelSimulationDebuggerPairCandidates.IsCreated)
        {
            ParallelSimulationDebuggerPairCandidates.ResizeUninitialized(
                TimestepContactPairs.Length);
        }
        blockStatistics.ResizeUninitialized(
            (TimestepContactPairs.Length + JacobiPairBatchSize - 1) / JacobiPairBatchSize);
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
        iterationState.Value = iteration;
    }

    private void BeginP1P6FinalizeSubstep(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        statistics.IterationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        AccumulateConstraintStatistics(ref statistics, ref runtime.PenetrationSum);
        StoreContactStatistics(statistics);
        runtimeState.Value = runtime;
    }

    private void FinalizeP1P6VelocityStatistics(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        int blockCount)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        float speedBefore = 0f;
        float speedAfter = 0f;
        int count = 0;
        for (int blockIndex = 0; blockIndex < blockCount; blockIndex++)
        {
            int bodyIndex = blockIndex * ParallelBodyBatchSize;
            ParallelBodyStageResult body = ParallelBodyStatistics[bodyIndex];
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
        StoreContactStatistics(statistics);
    }

    private void EnsureSoftIncidentIndexP1P6()
    {
        ActiveIncidentIndexState state = ActiveIncidentIndexState.Value;
        if (state.SoftIsValid != 0 &&
            state.SoftPairCount == SoftAvoidancePairs.Length &&
            state.SoftBodyCount == States.Length)
            return;

        BuildSoftIncidentIndexP1P6();
        state = ActiveIncidentIndexState.Value;
        state.SoftPairCount = SoftAvoidancePairs.Length;
        state.SoftBodyCount = States.Length;
        state.SoftIsValid = 1;
        ActiveIncidentIndexState.Value = state;
    }

    private void InvalidateSoftIncidentIndexP1P6()
    {
        ActiveIncidentIndexState state = ActiveIncidentIndexState.Value;
        state.SoftIsValid = 0;
        ActiveIncidentIndexState.Value = state;
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
        state = ActiveIncidentIndexState.Value;
        state.Fingerprint = fingerprint;
        state.PairCount = TimestepContactPairs.Length;
        state.BodyCount = States.Length;
        state.IsValid = 1;
        ActiveIncidentIndexState.Value = state;
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
