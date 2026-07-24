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
public struct ParallelJacobiExecutionState
{
    public byte IsValid;
#if RTS_CONTACT_DIAGNOSTICS
    public float PenetrationSum;
    public long SolverStartTimestamp;
    public long IterationStartTimestamp;
#else
    public float PenetrationSum { get => default; set { } }
    public long SolverStartTimestamp { get => default; set { } }
    public long IterationStartTimestamp { get => default; set { } }
#endif
}

#if RTS_CONTACT_DIAGNOSTICS
public struct ParallelJacobiIterationTelemetry
{
    public float MaxViolationBeforeSolve;
    public float AverageViolationBeforeSolve;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
}

public struct JacobiBlockTelemetry
{
    public float TotalPositionCorrection;
    public float MaxPositionCorrection;
    public int NewlyActivatedPairCount;
    public int NewlyCorrectedPairCount;
    public int SelectedPairCount;
    public int SelectedPairOffset;
}
#endif

/// <summary>
/// Multi-job Jacobi path. The topology, lifecycle, envelope validation and fallback
/// remain serial coordination stages; pair evaluation and body gather/apply are
/// conflict-free parallel stages. Selected-pair debugger capture uses pair-exclusive
/// scratch slots and deterministic compaction without changing the solver backend.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private const int JacobiPairBatchSize = 64;

    public JobHandle ScheduleParallelJacobi(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
#endif
        JobHandle dependency)
    {
        JobHandle handle = new InitializeParallelJacobiPipelineJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(dependency);

        int substepCount = math.max(1, Configuration.SubstepCount);
        int iterationCount = math.max(1, Configuration.IterationCount);
        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            handle = new PrepareParallelJacobiSubstepJob
            {
                Solver = this,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                handle = new PrepareParallelJacobiIterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
                    IterationState = iterationState,
                    BlockStatistics = blockStatistics,
#endif
                    SubstepIndex = substepIndex
                }.Schedule(handle);

                var evaluatePairsJob = new EvaluateParallelJacobiPairsJob
                {
                    Alpha = Configuration.Compliance /
                            math.max(0.0000001f,
                                math.pow(Configuration.DeltaTime / substepCount, 2f)),
                    SubstepIndex = substepIndex,
                    States = States,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray()
                };
                handle = evaluatePairsJob.Schedule(
                    TimestepContactPairs,
                    JacobiPairBatchSize,
                    handle);

#if RTS_CONTACT_DIAGNOSTICS
                var reduceBlocksJob = new ReduceParallelJacobiBlocksJob
                {
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                };
                handle = reduceBlocksJob.Schedule(blockStatistics, 1, handle);
#endif

                var gatherBodiesJob = new GatherAndApplyParallelJacobiBodiesJob
                {
                    States = States,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    IncidentOffsets = ActiveIncidentOffsets,
                    IncidentPairIndices = ActiveIncidentPairIndices.AsDeferredJobArray(),
                    CorrectedBodyFlags = CorrectedBodyFlags
                };
                handle = gatherBodiesJob.Schedule(States.Length, 64, handle);

                handle = new FinalizeParallelJacobiIterationJob
                {
                    Solver = this,
                    RuntimeState = runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
                    IterationState = iterationState,
                    BlockStatistics = blockStatistics,
#endif
                    SubstepIndex = substepIndex,
                    IterationIndex = iterationIndex
                }.Schedule(handle);
            }

            handle = new FinalizeParallelJacobiSubstepJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }

#if RTS_CONTACT_DIAGNOSTICS
        return new FinalizeParallelJacobiPipelineJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(handle);
#else
        return handle;
#endif
    }

    [BurstCompile]
    private struct InitializeParallelJacobiPipelineJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;

        public void Execute()
        {
            Solver.InitializeParallelJacobiPipeline(RuntimeState);
        }
    }

    [BurstCompile]
    private struct PrepareParallelJacobiSubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
        public int SubstepIndex;

        public void Execute()
        {
            Solver.PrepareParallelJacobiSubstep(SubstepIndex, RuntimeState);
        }
    }

    [BurstCompile]
    private struct PrepareParallelJacobiIterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
        public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
        public NativeList<JacobiBlockTelemetry> BlockStatistics;
#endif
        public int SubstepIndex;

        public void Execute()
        {
            Solver.PrepareParallelJacobiIteration(
                SubstepIndex,
                RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                , IterationState,
                BlockStatistics
#endif
            );
        }
    }

    [BurstCompile]
    private struct EvaluateParallelJacobiPairsJob : IJobParallelForDefer
    {
        public float Alpha;
        public int SubstepIndex;
        [ReadOnly] public NativeArray<FlowMovementFrameState> States;
        public NativeArray<UnitCollisionPair> Pairs;
        public NativeArray<JacobiPairCorrection> Corrections;

        public void Execute(int pairIndex)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            ContactConstraintEvaluation evaluation = XpbdContactConstraintMath.Evaluate(
                ref pair,
                bodyA,
                bodyB,
                Alpha,
                SubstepIndex);

            JacobiPairCorrection correction = default;
            correction.NewlyActivated = evaluation.NewlyActivated;
            correction.PairCorrection = evaluation.PairCorrection;
            if (math.abs(evaluation.AppliedLambda) > 0.0000001f)
            {
                if (pair.WasCorrectedThisTimestep == 0)
                {
                    pair.WasCorrectedThisTimestep = 1;
                    correction.NewlyCorrected = 1;
                }
                if (bodyA.InverseMass > 0f)
                {
                    correction.DeltaA = evaluation.Normal *
                                        (bodyA.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveA = 1;
                }
                if (bodyB.InverseMass > 0f)
                {
                    correction.DeltaB = -evaluation.Normal *
                                        (bodyB.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveB = 1;
                }
            }

            Pairs[pairIndex] = pair;
            Corrections[pairIndex] = correction;
        }
    }

    [BurstCompile]
    private struct EvaluateParallelJacobiPairsWithDiagnosticsJob :
        IJobParallelForDefer
    {
        public float Alpha;
        public int SubstepIndex;
        [ReadOnly] public NativeArray<FlowMovementFrameState> States;
        public NativeArray<UnitCollisionPair> Pairs;
        public NativeArray<JacobiPairCorrection> Corrections;
        public NativeArray<ParallelSimulationDebuggerPairCapture>
            DiagnosticPairCandidates;
        public Entity DiagnosticSelectedEntity;

        public void Execute(int pairIndex)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            ContactConstraintEvaluation evaluation = XpbdContactConstraintMath.Evaluate(
                ref pair,
                bodyA,
                bodyB,
                Alpha,
                SubstepIndex);

            JacobiPairCorrection correction = default;
            correction.NewlyActivated = evaluation.NewlyActivated;
            correction.PairCorrection = evaluation.PairCorrection;
            if (math.abs(evaluation.AppliedLambda) > 0.0000001f)
            {
                if (pair.WasCorrectedThisTimestep == 0)
                {
                    pair.WasCorrectedThisTimestep = 1;
                    correction.NewlyCorrected = 1;
                }
                if (bodyA.InverseMass > 0f)
                {
                    correction.DeltaA = evaluation.Normal *
                                        (bodyA.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveA = 1;
                }
                if (bodyB.InverseMass > 0f)
                {
                    correction.DeltaB = -evaluation.Normal *
                                        (bodyB.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveB = 1;
                }
            }

            Pairs[pairIndex] = pair;
            Corrections[pairIndex] = correction;

            ParallelSimulationDebuggerPairCapture capture = default;
            if (bodyA.Entity == DiagnosticSelectedEntity ||
                bodyB.Entity == DiagnosticSelectedEntity)
            {
                capture.IsValid = 1;
                capture.Sample =
                    SolveXpbdUnitContactsJob.BuildSimulationDebuggerPairSample(
                        SubstepIndex,
                        pair,
                        bodyA,
                        bodyB,
                        evaluation.Normal,
                        evaluation.ConstraintValue,
                        evaluation.PairCorrection);
            }
            DiagnosticPairCandidates[pairIndex] = capture;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    private struct ReduceParallelJacobiBlocksJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<JacobiPairCorrection> Corrections;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Corrections.Length);
            JacobiBlockTelemetry block = default;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                JacobiPairCorrection correction = Corrections[pairIndex];
                if (correction.ActiveA != 0 || correction.ActiveB != 0)
                {
                    block.TotalPositionCorrection += correction.PairCorrection;
                    block.MaxPositionCorrection = math.max(
                        block.MaxPositionCorrection,
                        correction.PairCorrection);
                }
                block.NewlyActivatedPairCount += correction.NewlyActivated;
                block.NewlyCorrectedPairCount += correction.NewlyCorrected;
            }
            Blocks[blockIndex] = block;
        }
    }

#endif

    [BurstCompile]
    private struct GatherAndApplyParallelJacobiBodiesJob : IJobParallelFor
    {
        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<UnitCollisionPair> Pairs;
        [ReadOnly] public NativeArray<JacobiPairCorrection> Corrections;
        [ReadOnly] public NativeArray<int> IncidentOffsets;
        [ReadOnly] public NativeArray<int> IncidentPairIndices;
        public NativeArray<byte> CorrectedBodyFlags;

        public void Execute(int bodyIndex)
        {
            float3 correctionSum = float3.zero;
            int correctionCount = 0;
            int begin = IncidentOffsets[bodyIndex];
            int end = IncidentOffsets[bodyIndex + 1];
            for (int incidentIndex = begin; incidentIndex < end; incidentIndex++)
            {
                int pairIndex = IncidentPairIndices[incidentIndex];
                UnitCollisionPair pair = Pairs[pairIndex];
                JacobiPairCorrection contribution = Corrections[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    correctionSum += contribution.DeltaA;
                    correctionCount++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    correctionSum += contribution.DeltaB;
                    correctionCount++;
                }
            }

            if (correctionCount <= 0)
                return;

            FlowMovementFrameState body = States[bodyIndex];
            float3 correction = correctionSum / correctionCount;
            body.PredictedPosition += correction;
            body.ContactPositionCorrection += correction;
            body.TimestepContactCorrection += correction;
            body.PredictedPosition.y = body.CurrentPosition.y;
            States[bodyIndex] = body;
            CorrectedBodyFlags[bodyIndex] = 1;
        }
    }

    [BurstCompile]
    private struct FinalizeParallelJacobiIterationJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
        public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
        [ReadOnly] public NativeList<JacobiBlockTelemetry> BlockStatistics;
#endif
        public int SubstepIndex;
        public int IterationIndex;

        public void Execute()
        {
            Solver.FinalizeParallelJacobiIteration(
                SubstepIndex,
                IterationIndex,
                RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                , IterationState,
                BlockStatistics
#endif
            );
        }
    }

    [BurstCompile]
    private struct FinalizeParallelJacobiSubstepJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;

        public void Execute()
        {
            Solver.FinalizeParallelJacobiSubstep(RuntimeState);
        }
    }

    [BurstCompile]
    private struct FinalizeParallelJacobiPipelineJob : IJob
    {
        public SolveXpbdUnitContactsJob Solver;
        public NativeReference<ParallelJacobiExecutionState> RuntimeState;

        public void Execute()
        {
            Solver.FinalizeParallelJacobiPipeline(RuntimeState);
        }
    }

    private void InitializeParallelJacobiPipeline(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        var state = new ParallelJacobiExecutionState
        {
            IsValid = 1
        };
#if RTS_CONTACT_DIAGNOSTICS
        state.SolverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
#endif
        var statistics = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        var incrementalStatistics = new IncrementalContactPipelineStatistics();
        ResetContactDiagnosticsCapture();

        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;
        if (substepDeltaTime <= 0f)
        {
            state.IsValid = 0;
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incrementalStatistics);
            runtimeState.Value = state;
            return;
        }

        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            IncrementalCacheState.Value = default;
        }

        if (EnableTimestepContactSetCache)
        {
            PrepareTimestepContactPrediction(DeltaTime, false);
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incrementalStatistics,
                false,
                false);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
        runtimeState.Value = state;
    }

    private void PrepareParallelJacobiSubstep(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;

        PrepareBaseVelocitiesForSubstep(substepDeltaTime);
        if (!EnableTimestepContactSetCache)
        {
            PrepareTimestepContactPrediction(substepDeltaTime, true);
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepInteractionAndSoftViews(ref statistics, ref incrementalStatistics);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }
        else if (!ValidateBaseMotionInteractionEnvelope(
                     substepIndex,
                     ref statistics,
                     ref incrementalStatistics))
        {
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                true,
                ref statistics,
                ref incrementalStatistics,
                false);
        }

        long softStart = ProfilerUnsafeUtility.Timestamp;
        CalculateSoftAvoidanceForSubstep(
            substepDeltaTime,
            ref statistics,
            ref incrementalStatistics);
        ClampSoftOutputToInteractionEnvelope(
            substepDeltaTime,
            ref incrementalStatistics);
        statistics.SoftAvoidanceEvaluationCount++;
        statistics.SoftAvoidanceNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - softStart);

        PredictUnconstrainedPositions(substepDeltaTime);
        bool rebuiltPredictedContactView = false;
        if (!ValidatePredictedContactEnvelope(
                substepIndex,
                ref statistics,
                ref incrementalStatistics))
        {
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incrementalStatistics,
                false);
            rebuiltPredictedContactView = true;
        }
        if (!EnableTimestepContactSetCache && !rebuiltPredictedContactView)
        {
            PrepareSubstepContactPrediction();
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incrementalStatistics);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }

        ActivateScheduledPredictiveContactsForSubstep(
            EnableTimestepContactSetCache ? substepIndex : 0,
            EnableTimestepContactSetCache ? substepCount : 1,
            ref incrementalStatistics);
        ResetTimestepContactSetForSubstep();
        RebuildActiveConstraintIncidentIndexIfNeeded();
        statistics.TimestepContactSetSubstepUseCount++;
#if RTS_CONTACT_DIAGNOSTICS
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
#endif

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
        runtimeState.Value = runtime;
    }

    private void PrepareParallelJacobiIteration(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        )
    {
        if (runtimeState.Value.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float substepDeltaTime = DeltaTime / substepCount;
#if RTS_CONTACT_DIAGNOSTICS
        ParallelJacobiIterationTelemetry iteration = default;
        if (EnableDiagnostics)
        {
            MeasureContactResidual(
                out iteration.MaxViolationBeforeSolve,
                out iteration.AverageViolationBeforeSolve);
        }
        SolveWallConstraintIteration(
            true,
            out iteration.TotalWallPositionCorrection,
            out iteration.MaxWallPositionCorrection);
#else
        SolveWallConstraintIteration(true, out _, out _);
#endif
        if (!ValidateSolverCorrectionContactEnvelope(
                substepIndex,
                ref statistics,
                ref incrementalStatistics))
        {
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incrementalStatistics);
            ResetTimestepContactSetForSubstep();
            RebuildActiveConstraintIncidentIndexIfNeeded();
        }

        // Contact corrections replace the wall-correction dirty set exactly as
        // in the serial solver: the wall set has already been envelope-validated.
        ResetCorrectedBodyTracking();
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);
#if RTS_CONTACT_DIAGNOSTICS
        int blockCount = (TimestepContactPairs.Length + JacobiPairBatchSize - 1) /
                         JacobiPairBatchSize;
        blockStatistics.ResizeUninitialized(blockCount);
#endif

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
#if RTS_CONTACT_DIAGNOSTICS
        iterationState.Value = iteration;
#endif
    }

    private void FinalizeParallelJacobiIteration(
        int substepIndex,
        int iterationIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        )
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        ParallelJacobiIterationTelemetry iteration = iterationState.Value;
#endif
        // Parallel bodies only set disjoint flags. Rebuild the corrected-body
        // list in body-index order so envelope repair stays deterministic.
        CorrectedBodyIndices.Clear();
        for (int bodyIndex = 0; bodyIndex < CorrectedBodyFlags.Length; bodyIndex++)
        {
            if (CorrectedBodyFlags[bodyIndex] != 0)
                CorrectedBodyIndices.Add(bodyIndex);
        }
#if RTS_CONTACT_DIAGNOSTICS
        float totalPositionCorrection = 0f;
        float maxPositionCorrection = 0f;
        int newlyActivated = 0;
        int newlyCorrected = 0;
        for (int i = 0; i < blockStatistics.Length; i++)
        {
            JacobiBlockTelemetry block = blockStatistics[i];
            totalPositionCorrection += block.TotalPositionCorrection;
            maxPositionCorrection = math.max(
                maxPositionCorrection,
                block.MaxPositionCorrection);
            newlyActivated += block.NewlyActivatedPairCount;
            newlyCorrected += block.NewlyCorrectedPairCount;
        }

        statistics.TimestepContactSetUniqueActivatedPairCount += newlyActivated;
        incrementalStatistics.UniqueCorrectedPairCount += newlyCorrected;
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;
        statistics.TotalContactPositionCorrection += totalPositionCorrection;
        statistics.MaxContactPositionCorrection = math.max(
            statistics.MaxContactPositionCorrection,
            maxPositionCorrection);
        statistics.TotalWallPositionCorrection +=
            iteration.TotalWallPositionCorrection;
        statistics.MaxWallPositionCorrection = math.max(
            statistics.MaxWallPositionCorrection,
            iteration.MaxWallPositionCorrection);

        if (EnableDiagnostics)
        {
            RecordIterationDiagnostic(
                substepIndex,
                iterationIndex,
                iteration.MaxViolationBeforeSolve,
                iteration.AverageViolationBeforeSolve,
                totalPositionCorrection,
                maxPositionCorrection,
                iteration.TotalWallPositionCorrection,
                iteration.MaxWallPositionCorrection);
        }
#endif

        if (!ValidateSolverCorrectionContactEnvelope(
                substepIndex,
                ref statistics,
                ref incrementalStatistics))
        {
            int substepCount = math.max(1, SubstepCount);
            float substepDeltaTime = DeltaTime / substepCount;
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incrementalStatistics);
            RebuildActiveConstraintIncidentIndexIfNeeded();

            // A repair on the final iteration is rare and correctness-sensitive.
            // Use the serial Jacobi reference for the one recovery projection;
            // normal iterations remain fully parallel.
            if (iterationIndex == math.max(1, IterationCount) - 1)
            {
                ResetTimestepContactSetForSubstep();
                SolveJacobiContactIteration(
                    substepDeltaTime,
                    substepIndex,
                    true,
                    ref statistics,
                    ref incrementalStatistics,
                    out float recoveryCorrection,
                    out float recoveryMaxCorrection);
                statistics.TotalContactPositionCorrection += recoveryCorrection;
                statistics.MaxContactPositionCorrection = math.max(
                    statistics.MaxContactPositionCorrection,
                    recoveryMaxCorrection);
            }
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
        runtimeState.Value = runtime;
    }

    private void FinalizeParallelJacobiSubstep(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        statistics.IterationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        AccumulateConstraintStatistics(ref statistics, ref runtime.PenetrationSum);
#endif
        ReconstructVelocities(
            DeltaTime / math.max(1, SubstepCount),
            ref statistics);
        StoreContactStatistics(statistics);
        runtimeState.Value = runtime;
    }

    private void FinalizeParallelJacobiPipeline(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();

        if (EnableDiagnostics)
            CaptureSelectedBodyAndPairs(substepCount - 1);
        BuildContactHeatSamples();
        statistics.AveragePenetration = statistics.PenetratingPairCount > 0
            ? runtime.PenetrationSum / statistics.PenetratingPairCount
            : 0f;
        statistics.UnactivatedPairCount =
            statistics.ContactPairCount - statistics.ActiveConstraintCount;
        statistics.PredictiveUnactivatedCount =
            statistics.PredictivePairCount - statistics.PredictiveActivatedCount;
        statistics.UnactivatedRatio = statistics.ContactPairCount > 0
            ? (float)statistics.UnactivatedPairCount / statistics.ContactPairCount
            : 0f;
        statistics.PredictiveUnactivatedRatio = statistics.PredictivePairCount > 0
            ? (float)statistics.PredictiveUnactivatedCount / statistics.PredictivePairCount
            : 0f;
        statistics.AverageIterationNanoseconds =
            statistics.IterationNanoseconds / math.max(1, substepCount * iterationCount);
        statistics.AverageSoftAvoidanceNanoseconds =
            statistics.SoftAvoidanceNanoseconds / substepCount;
        statistics.AverageSpeedBeforeContact /= substepCount;
        statistics.AverageSpeedAfterContact /= substepCount;
        statistics.SolverNanoseconds = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.SolverStartTimestamp);

        incrementalStatistics.UniqueActivatedPairCount =
            statistics.TimestepContactSetUniqueActivatedPairCount;
        incrementalStatistics.CurrentSweptContactCount =
            incrementalStatistics.CurrentDormantPairCount +
            incrementalStatistics.CurrentApproachingPairCount +
            incrementalStatistics.CurrentPredictivePairCount +
            incrementalStatistics.CurrentActualPairCount;
        incrementalStatistics.CurrentActiveConstraintCount =
            TimestepContactPairs.Length;
        incrementalStatistics.PeakActiveConstraintCount = math.max(
            incrementalStatistics.PeakActiveConstraintCount,
            incrementalStatistics.CurrentActiveConstraintCount);
        incrementalStatistics.CleanProxyRatio = incrementalStatistics.ProxyCount > 0
            ? 1f - math.saturate(
                (float)incrementalStatistics.TopologyDirtyBodyCount /
                incrementalStatistics.ProxyCount)
            : 0f;
        incrementalStatistics.RetainedNeighborPairRatio =
            incrementalStatistics.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.NeighborPairRetainedCount /
                    incrementalStatistics.PersistentNeighborPairCount)
                : 0f;
        incrementalStatistics.NeighborToSweptRatio =
            incrementalStatistics.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.CurrentSweptContactCount /
                    incrementalStatistics.PersistentNeighborPairCount)
                : 0f;
        incrementalStatistics.SweptToCurrentActiveRatio =
            incrementalStatistics.CurrentSweptContactCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.CurrentActiveConstraintCount /
                    incrementalStatistics.CurrentSweptContactCount)
                : 0f;
        incrementalStatistics.ActivatedToCorrectedRatio =
            incrementalStatistics.UniqueActivatedPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.UniqueCorrectedPairCount /
                    incrementalStatistics.UniqueActivatedPairCount)
                : 0f;

        CaptureSimulationDebuggerSelectedUnit();
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
#endif
    }
}
}
