using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

using static RTS.Unit.FlowField.Jobs.ParallelContactStageJobs;
using static RTS.Unit.FlowField.Jobs.ParallelJacobiJobs;

namespace RTS.Unit.FlowField.Jobs
{

public partial struct CrowdContactPipelineScheduler
{
    public JobHandle ScheduleParallelStages(
        NativeReference<ContactPipelineExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
#endif
        JobHandle dependency)
    {
        JobHandle handle = Lifecycle.Schedule(dependency);
#if RTS_CONTACT_DIAGNOSTICS
        handle = BeginStageTiming(runtimeState, handle);
#endif

        int substepCount = math.max(1, Configuration.SubstepCount);
        int iterationCount = math.max(1, Configuration.IterationCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;
        bool useJacobiSolver =
            Configuration.ContactPositionSolver ==
            ContactPositionSolverMode.Jacobi;
#if RTS_CONTACT_DIAGNOSTICS
        bool captureSelectedPairs = false;
        if (EnableDiagnostics)
        {
            captureSelectedPairs = DiagnosticSelectedEntity != Entity.Null &&
                (SimulationDebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
                (SimulationDebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedPairs) != 0;
        }
#endif
        int escapeBlockCount =
            (Bodies.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
        if (substepDeltaTime <= 0f)
        {
#if RTS_CONTACT_DIAGNOSTICS
            // Degenerate-dt early-out still publishes an (empty) pipeline
            // snapshot via FinalizePipeline (no runtime gate) so the
            // publish chain stays consistent with the oracle disabled.
            ConstraintSolverJob finalizeEmptyPipeline = ConstraintSolver;
            finalizeEmptyPipeline.Operation = ConstraintSolverOperation.FinalizePipeline;
            finalizeEmptyPipeline.RuntimeState = runtimeState;
            return finalizeEmptyPipeline.Schedule(handle);
#else
            return handle;
#endif
        }

        if (Configuration.EnableTimestepContactSetCache)
        {
            handle = new PrepareTimestepPredictionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
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
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialDirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialDirtyBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    DirtyBodies = IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialDirtyBodiesJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsets = DirtyBodyBlockOffsets,
                    DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }
        }

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = Certification.ScheduleInitialPersistentContactSet(
                runtimeState,
                handle);
        }
        else
        {
            handle = Certification.CreateBuildInitialContactSetJob(runtimeState).Schedule(handle);
        }
#if RTS_CONTACT_DIAGNOSTICS
        handle = EndStageTiming(
            runtimeState,
            ContactPipelineTimingOperation.EndValidationRepair,
            handle);
#endif

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
#if RTS_CONTACT_DIAGNOSTICS
            handle = BeginStageTiming(runtimeState, handle);
#endif
            handle = new PrepareBaseVelocityBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                SubstepDeltaTime = substepDeltaTime,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndMotion,
                handle);
            handle = BeginStageTiming(runtimeState, handle);
#endif

            if (!Configuration.EnableTimestepContactSetCache)
            {
                handle = new PrepareTimestepPredictionBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
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
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);
            }

            handle = new ValidateBaseMotionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                EscapeFlags = EnvelopeEscapeFlags,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = Certification.CreateFinalizeEnvelopeEscapesJob(substepIndex, runtimeState).Schedule(handle);

            handle = new PrepareRepairPredictionBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
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

            handle = Certification.CreatePrepareSubstepRepairJob(substepIndex, runtimeState).Schedule(handle);

            handle = new InteractionCertificationAlgorithms.EvaluatePersistentPairClassificationsJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                RawPairs = ClassificationBodyPairs.AsDeferredJobArray(),
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

            handle = Certification.CreateCommitSubstepRepairJob(substepIndex, runtimeState).Schedule(handle);
            handle = Certification.CreateValidateConsumerViewsJob(substepIndex, runtimeState).Schedule(handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndValidationRepair,
                handle);
#endif

            SoftAvoidanceJob prepareSoft = SoftAvoidance;
            prepareSoft.Operation = SoftAvoidanceOperation.PrepareParallelWorkset;
            prepareSoft.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
            prepareSoft.BlockStatistics = blockStatistics;
#endif
            handle = prepareSoft.Schedule(handle);

            handle = new InitializeSoftAvoidanceBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                Grid = Grid,
                GridOrigin = GridOrigin,
                GridDimensions = GridDimensions,
                CellRadius = CellRadius,
                SoftShell = Configuration.SoftAvoidanceShell
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            var evaluateSoftPairsJob = new EvaluateSoftAvoidancePairsJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
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

#if RTS_CONTACT_DIAGNOSTICS
            // ReduceSoftAvoidanceBlocksJob carries block telemetry counters
            // (no EnableDiagnostics runtime gate) so benchmarks with the oracle
            // off still capture valid soft-avoidance stats.
            {
                handle = new ReduceSoftAvoidanceBlocksJob
                {
                    Contributions = SoftPairContributions.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);
            }
#endif

            handle = new GatherSoftAvoidanceBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
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
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Soft-avoidance finalize carries timing/escape counters (no
            // EnableDiagnostics runtime gate) so benchmarks with the oracle
            // disabled still capture valid soft-avoidance telemetry.
            {
                handle = new ReduceSoftEscapeBlocksJob
                {
                    EscapeFlags = EnvelopeEscapeFlags,
                    EscapeCountsByBlock = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                SoftAvoidanceJob finalizeSoft = SoftAvoidance;
                finalizeSoft.Operation = SoftAvoidanceOperation.FinalizeParallel;
                finalizeSoft.RuntimeState = runtimeState;
                finalizeSoft.BlockStatistics = blockStatistics;
                finalizeSoft.EscapeCountsByBlock = DirtyBodyBlockOffsets;
                finalizeSoft.EscapeBlockCount = escapeBlockCount;
                handle = finalizeSoft.Schedule(handle);
            }
#endif

#if RTS_CONTACT_DIAGNOSTICS
            handle = BeginStageTiming(runtimeState, handle);
#endif
            handle = new PredictUnconstrainedBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndMotion,
                handle);
            handle = BeginStageTiming(runtimeState, handle);
#endif

            handle = new ValidatePredictedContactEnvelopeBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                EscapeFlags = EnvelopeEscapeFlags,
                PredictiveSkin = Configuration.PredictiveSkin
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsets = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = Certification.CreateFinalizePreparedSubstepJob(substepIndex, runtimeState).Schedule(handle);
            handle = Certification.CreateValidateConsumerViewsJob(substepIndex, runtimeState).Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = TimestepContactPairs.AsDeferredJobArray()
            }.Schedule(TimestepContactPairs, SoftPairBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndValidationRepair,
                handle);
#endif

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                ConstraintSolverJob beginIteration = ConstraintSolver;
                beginIteration.Operation =
                    ConstraintSolverOperation.InitializeContactIteration;
                beginIteration.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                beginIteration.IterationState = iterationState;
#endif
                beginIteration.SubstepIndex = substepIndex;
                handle = beginIteration.Schedule(handle);

                handle = new SolveWallConstraintBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                    Grid = Grid,
                    GridOrigin = GridOrigin,
                    GridDimensions = GridDimensions,
                    CellRadius = CellRadius,
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

                handle = new CountAndReduceWallBlocksJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixCorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterCorrectedBodiesJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BlockOffsets = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = Certification.CreateFinalizeWallIterationJob(substepIndex, escapeBlockCount, runtimeState).Schedule(handle);

                if (iterationIndex != iterationCount - 1)
                    continue;

                if (!useJacobiSolver)
                {
                    ConstraintSolverJob solveGaussSeidel = ConstraintSolver;
                    solveGaussSeidel.Operation =
                        ConstraintSolverOperation.SolveGaussSeidelContact;
                    solveGaussSeidel.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                    solveGaussSeidel.IterationState = iterationState;
#endif
                    solveGaussSeidel.SubstepIndex = substepIndex;
                    solveGaussSeidel.IterationIndex = iterationIndex;
                    handle = solveGaussSeidel.Schedule(handle);
                }
                else
                {
#if RTS_CONTACT_DIAGNOSTICS
                    if (captureSelectedPairs)
                    {
                        handle = new EvaluateParallelJacobiPairsWithDiagnosticsJob
                        {
                            Alpha = Configuration.Compliance /
                                    math.max(
                                        0.0000001f,
                                        substepDeltaTime * substepDeltaTime),
                            SubstepIndex = substepIndex,
                            RecoveryOnly = 0,
                            RuntimeState = runtimeState,
                            Bodies = Bodies,
                            NavigationStates = NavigationStates,
                            MotionIntents = MotionIntents,
                            MotionEvidence = MotionEvidence,
                            StepStates = StepStates,
                            Pairs = TimestepContactPairs.AsDeferredJobArray(),
                            Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                            DiagnosticPairCandidates =
                                ParallelSimulationDebuggerPairCandidates.AsDeferredJobArray(),
                            DiagnosticSelectedEntity = DiagnosticSelectedEntity
                        }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                    }
                    else
#endif
                    {
                        handle = new EvaluateParallelJacobiPairsJob
                        {
                            Alpha = Configuration.Compliance /
                                    math.max(
                                        0.0000001f,
                                        substepDeltaTime * substepDeltaTime),
                            SubstepIndex = substepIndex,
                            RecoveryOnly = 0,
                            RuntimeState = runtimeState,
                            Bodies = Bodies,
                            NavigationStates = NavigationStates,
                            MotionIntents = MotionIntents,
                            MotionEvidence = MotionEvidence,
                            StepStates = StepStates,
                            Pairs = TimestepContactPairs.AsDeferredJobArray(),
                            Corrections = JacobiPairCorrections.AsDeferredJobArray()
                        }.Schedule(TimestepContactPairs, JacobiPairBatchSize, handle);
                    }

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

#if RTS_CONTACT_DIAGNOSTICS
                    if (captureSelectedPairs)
                    {
                        handle = new CountParallelSimulationDebuggerPairBlocksJob
                        {
                            Candidates =
                                ParallelSimulationDebuggerPairCandidates,
                            Blocks = blockStatistics
                        }.Schedule(blockStatistics, 1, handle);

                        handle = new PrefixParallelSimulationDebuggerPairsJob
                        {
                            Blocks = blockStatistics,
                            Scratch = ParallelSimulationDebuggerPairScratch
                        }.Schedule(handle);

                        handle = new ScatterParallelSimulationDebuggerPairsJob
                        {
                            Candidates =
                                ParallelSimulationDebuggerPairCandidates,
                            Blocks = blockStatistics,
                            Scratch = ParallelSimulationDebuggerPairScratch
                        }.Schedule(blockStatistics, 1, handle);

                        ConstraintSolverJob mergeDebuggerPairs = ConstraintSolver;
                        mergeDebuggerPairs.Operation =
                            ConstraintSolverOperation.MergeParallelDebuggerPairs;
                        handle = mergeDebuggerPairs.Schedule(handle);
                    }
#endif

                    handle = new GatherAndApplyParallelJacobiBodiesJob
                    {
                        RecoveryOnly = 0,
                        RuntimeState = runtimeState,
                        Bodies = Bodies,
                        NavigationStates = NavigationStates,
                        MotionIntents = MotionIntents,
                        MotionEvidence = MotionEvidence,
                        StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = ActiveIncidentOffsets,
                        IncidentPairIndices =
                            ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = CorrectedBodyFlags
                    }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);
                }

                handle = Certification.CreateFinalizeContactIterationJob(substepIndex, iterationIndex, runtimeState).Schedule(handle);

                if (!useJacobiSolver)
                {
                    ConstraintSolverJob recovery = ConstraintSolver;
                    recovery.Operation =
                        ConstraintSolverOperation.SolveGaussSeidelRecovery;
                    recovery.RuntimeState = runtimeState;
                    recovery.SubstepIndex = substepIndex;
                    handle = recovery.Schedule(handle);
                }
                else
                {
                    ConstraintSolverJob prepareRecovery = ConstraintSolver;
                    prepareRecovery.Operation =
                        ConstraintSolverOperation.PrepareJacobiRecovery;
                    prepareRecovery.RuntimeState = runtimeState;
                    handle = prepareRecovery.Schedule(handle);

                    handle = new EvaluateParallelJacobiPairsJob
                    {
                        Alpha = Configuration.Compliance /
                                math.max(
                                    0.0000001f,
                                    substepDeltaTime * substepDeltaTime),
                        SubstepIndex = substepIndex,
                        RecoveryOnly = 1,
                        RuntimeState = runtimeState,
                        Bodies = Bodies,
                        NavigationStates = NavigationStates,
                        MotionIntents = MotionIntents,
                        MotionEvidence = MotionEvidence,
                        StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections =
                            JacobiPairCorrections.AsDeferredJobArray()
                    }.Schedule(
                        TimestepContactPairs,
                        JacobiPairBatchSize,
                        handle);

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

                    handle = new GatherAndApplyParallelJacobiBodiesJob
                    {
                        RecoveryOnly = 1,
                        RuntimeState = runtimeState,
                        Bodies = Bodies,
                        NavigationStates = NavigationStates,
                        MotionIntents = MotionIntents,
                        MotionEvidence = MotionEvidence,
                        StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections =
                            JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = ActiveIncidentOffsets,
                        IncidentPairIndices =
                            ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = CorrectedBodyFlags
                    }.Schedule(
                        Bodies.Length,
                        ParallelBodyBatchSize,
                        handle);

                    ConstraintSolverJob finalizeRecovery = ConstraintSolver;
                    finalizeRecovery.Operation =
                        ConstraintSolverOperation.FinalizeJacobiRecovery;
                    finalizeRecovery.RuntimeState = runtimeState;
                    handle = finalizeRecovery.Schedule(handle);
                }
            }

#if RTS_CONTACT_DIAGNOSTICS
            // FinalizeSubstepTelemetry accumulates IterationNanoseconds and
            // constraint counters (no EnableDiagnostics runtime gate) so
            // benchmarks with the oracle disabled still capture valid iteration
            // timing.
            {
                ConstraintSolverJob beginFinalizeSubstep = ConstraintSolver;
                beginFinalizeSubstep.Operation =
                    ConstraintSolverOperation.FinalizeSubstepTelemetry;
                beginFinalizeSubstep.RuntimeState = runtimeState;
                handle = beginFinalizeSubstep.Schedule(handle);
            }
#endif

#if RTS_CONTACT_DIAGNOSTICS
            handle = BeginStageTiming(runtimeState, handle);
#endif
            handle = new ReconstructVelocityBodiesJob
            {
                Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                BodyStatistics = ParallelBodyStatistics,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Velocity-body block reduce + finalize carry timing/counters
            // (no EnableDiagnostics runtime gate) for benchmarks with oracle off.
            {
                handle = new ReduceVelocityBodyBlocksJob
                {
                    BodyStatistics = ParallelBodyStatistics,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                ConstraintSolverJob finalizeVelocity = ConstraintSolver;
                finalizeVelocity.Operation = ConstraintSolverOperation.FinalizeVelocity;
                finalizeVelocity.RuntimeState = runtimeState;
                finalizeVelocity.BlockCount = escapeBlockCount;
                handle = finalizeVelocity.Schedule(handle);
            }
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndMotion,
                handle);
#endif
        }

#if RTS_CONTACT_DIAGNOSTICS
        // FinalizePipeline publishes SolverNanoseconds, UniqueActivatedPairCount
        // and the cross-stage ratios. No EnableDiagnostics runtime gate so benchmarks
        // with the oracle disabled still get valid pipeline-total telemetry.
        {
            ConstraintSolverJob finalizePipeline = ConstraintSolver;
            finalizePipeline.Operation = ConstraintSolverOperation.FinalizePipeline;
            finalizePipeline.RuntimeState = runtimeState;
            return finalizePipeline.Schedule(handle);
        }
#else
        return handle;
#endif
    }

#if RTS_CONTACT_DIAGNOSTICS
    private JobHandle BeginStageTiming(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        JobHandle dependency) =>
        new ContactPipelineTimingJob
        {
            Operation = ContactPipelineTimingOperation.Begin,
            RuntimeState = runtimeState,
            Statistics = Statistics,
            IncrementalStatistics = IncrementalStatistics
        }.Schedule(dependency);

    private JobHandle EndStageTiming(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        ContactPipelineTimingOperation operation,
        JobHandle dependency) =>
        new ContactPipelineTimingJob
        {
            Operation = operation,
            RuntimeState = runtimeState,
            Statistics = Statistics,
            IncrementalStatistics = IncrementalStatistics
        }.Schedule(dependency);
#endif
}
}
