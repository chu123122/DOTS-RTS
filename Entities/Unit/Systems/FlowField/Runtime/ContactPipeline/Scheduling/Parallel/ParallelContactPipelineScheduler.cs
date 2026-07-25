using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

using static RTS.Unit.FlowField.Jobs.ParallelContactPipelineJobs;
using static RTS.Unit.FlowField.Jobs.ParallelJacobiJobs;

namespace RTS.Unit.FlowField.Jobs
{

public partial struct CrowdContactPipelineScheduler
{
    public JobHandle ScheduleParallelJacobiP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
#endif
        JobHandle dependency)
    {
        ContactPipelineLifecycleJob initialize = Lifecycle;
        initialize.Operation = ContactPipelineLifecycleOperation.InitializeParallel;
        initialize.RuntimeState = runtimeState;
        JobHandle handle = initialize.Schedule(dependency);

        int substepCount = math.max(1, Configuration.SubstepCount);
        int iterationCount = math.max(1, Configuration.IterationCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;
#if RTS_CONTACT_DIAGNOSTICS
        bool captureSelectedPairs =
            Configuration.EnableDiagnostics && DiagnosticSelectedEntity != Entity.Null &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
            (SimulationDebuggerCaptureMask &
             SimulationDebuggerCaptureMask.SelectedPairs) != 0;
#endif
        int escapeBlockCount =
            (Bodies.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
        if (substepDeltaTime <= 0f)
        {
#if RTS_CONTACT_DIAGNOSTICS
            ConstraintSolverJob finalizeEmptyPipeline = ConstraintSolver;
            finalizeEmptyPipeline.Operation = ConstraintSolverOperation.FinalizeParallelPipeline;
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
                handle = new CountInitialP1P6DirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialP1P6DirtyBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    DirtyBodies = IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialP1P6DirtyBodiesJob
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
            handle = Certification.ScheduleInitialPersistentContactSetP1P6(
                runtimeState,
                handle);
        }
        else
        {
            InteractionCertificationJob buildInitial = Certification;
            buildInitial.Operation = InteractionCertificationOperation.BuildInitialP1P6;
            buildInitial.RuntimeState = runtimeState;
            handle = buildInitial.Schedule(handle);
        }

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
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

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
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

            InteractionCertificationJob finalizeEscapes = Certification;
            finalizeEscapes.Operation = InteractionCertificationOperation.FinalizeEnvelopeEscapesP1P6;
            finalizeEscapes.RuntimeState = runtimeState;
            finalizeEscapes.SubstepIndex = substepIndex;
            handle = finalizeEscapes.Schedule(handle);

            handle = new PrepareP1P6RepairPredictionBodiesJob
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

            InteractionCertificationJob prepareRepair = Certification;
            prepareRepair.Operation = InteractionCertificationOperation.PrepareSubstepRepairP1P6;
            prepareRepair.RuntimeState = runtimeState;
            prepareRepair.SubstepIndex = substepIndex;
            handle = prepareRepair.Schedule(handle);

            handle = new InteractionCertificationJob.EvaluatePersistentPairClassificationsP1P6Job
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

            InteractionCertificationJob commitRepair = Certification;
            commitRepair.Operation = InteractionCertificationOperation.CommitSubstepRepairP1P6;
            commitRepair.RuntimeState = runtimeState;
            commitRepair.SubstepIndex = substepIndex;
            handle = commitRepair.Schedule(handle);

            InteractionCertificationJob gateSoftViews = Certification;
            gateSoftViews.Operation =
                InteractionCertificationOperation.ValidateConsumerViewsP1P6;
            gateSoftViews.RuntimeState = runtimeState;
            gateSoftViews.SubstepIndex = substepIndex;
            handle = gateSoftViews.Schedule(handle);

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
            handle = new ReduceSoftAvoidanceBlocksJob
            {
                Contributions = SoftPairContributions.AsDeferredJobArray(),
                Blocks = blockStatistics.AsDeferredJobArray()
            }.Schedule(blockStatistics, 1, handle);
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
            handle = new ReduceP1P6SoftEscapeBlocksJob
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

            handle = new CountP1P6EnvelopeEscapeBlocksJob
            {
                EscapeFlags = EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                BodyCount = Bodies.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixP1P6EnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                DirtyBodies = IncrementalDirtyBodies,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterP1P6EnvelopeEscapesJob
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

            InteractionCertificationJob finalizePrepared = Certification;
            finalizePrepared.Operation = InteractionCertificationOperation.FinalizePreparedSubstepP1P6;
            finalizePrepared.RuntimeState = runtimeState;
            finalizePrepared.SubstepIndex = substepIndex;
            handle = finalizePrepared.Schedule(handle);

            InteractionCertificationJob gateSolverViews = Certification;
            gateSolverViews.Operation =
                InteractionCertificationOperation.ValidateConsumerViewsP1P6;
            gateSolverViews.RuntimeState = runtimeState;
            gateSolverViews.SubstepIndex = substepIndex;
            handle = gateSolverViews.Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = TimestepContactPairs.AsDeferredJobArray()
            }.Schedule(TimestepContactPairs, SoftPairBatchSize, handle);

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                ConstraintSolverJob beginIteration = ConstraintSolver;
                beginIteration.Operation = ConstraintSolverOperation.BeginParallelIteration;
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

                handle = new CountAndReduceP1P6WallBlocksJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BodyStatistics = ParallelBodyStatistics,
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixP1P6CorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterP1P6CorrectedBodiesJob
                {
                    CorrectedBodyFlags = CorrectedBodyFlags,
                    BlockOffsets = DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                InteractionCertificationJob finalizeWall = Certification;
                finalizeWall.Operation = InteractionCertificationOperation.FinalizeWallIterationP1P6;
                finalizeWall.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                finalizeWall.IterationState = iterationState;
                finalizeWall.BlockStatistics = blockStatistics;
#endif
                finalizeWall.SubstepIndex = substepIndex;
                finalizeWall.BodyBlockCount = escapeBlockCount;
                handle = finalizeWall.Schedule(handle);

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
                        Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                        Pairs = TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                        DiagnosticPairCandidates =
                            ParallelSimulationDebuggerPairCandidates
                                .AsDeferredJobArray(),
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
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);
#endif

#if RTS_CONTACT_DIAGNOSTICS
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

                    ConstraintSolverJob mergeDebuggerPairs = ConstraintSolver;
                    mergeDebuggerPairs.Operation = ConstraintSolverOperation.MergeParallelDebuggerPairs;
                    handle = mergeDebuggerPairs.Schedule(handle);
                }
#endif

                handle = new GatherAndApplyParallelJacobiBodiesJob
                {
                    Bodies = Bodies,
                NavigationStates = NavigationStates,
                MotionIntents = MotionIntents,
                MotionEvidence = MotionEvidence,
                StepStates = StepStates,
                    Pairs = TimestepContactPairs.AsDeferredJobArray(),
                    Corrections = JacobiPairCorrections.AsDeferredJobArray(),
                    IncidentOffsets = ActiveIncidentOffsets,
                    IncidentPairIndices = ActiveIncidentPairIndices.AsDeferredJobArray(),
                    CorrectedBodyFlags = CorrectedBodyFlags
                }.Schedule(Bodies.Length, ParallelBodyBatchSize, handle);

                InteractionCertificationJob finalizeContact = Certification;
                finalizeContact.Operation = InteractionCertificationOperation.FinalizeContactIterationP1P6;
                finalizeContact.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
                finalizeContact.IterationState = iterationState;
                finalizeContact.BlockStatistics = blockStatistics;
#endif
                finalizeContact.SubstepIndex = substepIndex;
                finalizeContact.IterationIndex = iterationIndex;
                handle = finalizeContact.Schedule(handle);

                ConstraintSolverJob recovery = ConstraintSolver;
                recovery.Operation = ConstraintSolverOperation.SolveParallelRecovery;
                recovery.RuntimeState = runtimeState;
                recovery.SubstepIndex = substepIndex;
                handle = recovery.Schedule(handle);
            }

#if RTS_CONTACT_DIAGNOSTICS
            ConstraintSolverJob beginFinalizeSubstep = ConstraintSolver;
            beginFinalizeSubstep.Operation = ConstraintSolverOperation.BeginParallelFinalizeSubstep;
            beginFinalizeSubstep.RuntimeState = runtimeState;
            handle = beginFinalizeSubstep.Schedule(handle);
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
            handle = new ReduceP1P6VelocityBodyBlocksJob
            {
                BodyStatistics = ParallelBodyStatistics,
                BodyCount = Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            ConstraintSolverJob finalizeVelocity = ConstraintSolver;
            finalizeVelocity.Operation = ConstraintSolverOperation.FinalizeParallelVelocity;
            finalizeVelocity.RuntimeState = runtimeState;
            finalizeVelocity.BlockCount = escapeBlockCount;
            handle = finalizeVelocity.Schedule(handle);
#endif
        }

#if RTS_CONTACT_DIAGNOSTICS
        ConstraintSolverJob finalizePipeline = ConstraintSolver;
        finalizePipeline.Operation = ConstraintSolverOperation.FinalizeParallelPipeline;
        finalizePipeline.RuntimeState = runtimeState;
        return finalizePipeline.Schedule(handle);
#else
        return handle;
#endif
    }
}
}
