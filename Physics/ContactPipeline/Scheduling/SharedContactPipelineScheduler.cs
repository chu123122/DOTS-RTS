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

internal partial struct CrowdContactPipelineScheduler
{
    public JobHandle ScheduleParallelStages(
        NativeReference<ContactPipelineExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
#endif
        JobHandle dependency)
    {
        JobHandle handle = dependency;
        try
        {
        handle = Lifecycle.Schedule(dependency);
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
        if (Configuration.EnableDiagnostics)
        {
            captureSelectedPairs = DiagnosticSelectedEntity != Entity.Null &&
                (DebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
                (DebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedPairs) != 0;
        }
#endif
        int escapeBlockCount =
            (Body.Bodies.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
        if (substepDeltaTime <= 0f)
        {
#if RTS_CONTACT_DIAGNOSTICS
            // Degenerate-dt early-out still publishes an (empty) pipeline
            // snapshot via FinalizePipeline (no runtime gate) so the
            // publish chain stays consistent with the oracle disabled.
            ConstraintSolverJob finalizeEmptyPipeline =
                CreateConstraintSolverJob();
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
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody = Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                PersistentCacheState = Persistent.IncrementalCacheState,
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                GridOrigin = Obstacles.Geometry.Origin,
                CellRadius = Obstacles.Geometry.CellRadius,
                FromSolvedPosition = 0,
                DetectPersistentDirty = (byte)(Configuration.EnablePersistentContactCache ? 1 : 0),
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialDirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                    BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialDirtyBodiesJob
                {
                    BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                    DirtyBodies = Repair.DirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialDirtyBodiesJob
                {
                    DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                    BlockOffsets = Solver.DirtyBodyBlockOffsets,
                    DirtyBodies = Repair.DirtyBodies.AsDeferredJobArray(),
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }
        }

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = new PrepareCurrentBodyIndexJob
            {
                CurrentBodyIndexByEntity =
                    Body.CurrentBodyIndexByEntity,
                BodyCount = Body.Bodies.Length
            }.Schedule(handle);
            handle = new BuildCurrentBodyIndexJob
            {
                Bodies = Body.Bodies,
                CurrentBodyIndexByEntity = Body.CurrentBodyIndexByEntity.AsParallelWriter()
            }.Schedule(
                Body.Bodies.Length,
                ParallelBodyBatchSize,
                handle);
            ScheduleDirtyContactScheduleCompaction(ref handle);
        }

        if (Configuration.EnableTimestepContactSetCache)
        {
            bool persistentCache =
                Configuration.EnablePersistentContactCache;
            ScheduleFullSweepBroadPhase(
                ref handle,
                persistentCache,
                runtimeState,
                false,
                persistentCache);
        }

        if (Configuration.EnableTimestepContactSetCache)
        {
            SchedulePersistentTopologyPublication(ref handle);
            if (Configuration.EnablePersistentContactCache)
                SchedulePersistentReusePublication(ref handle, runtimeState);
            handle = new PreparePersistentClassificationJob
            {
                Configuration = Configuration,
                FullSweepPrepared =
                    BroadPhase.FullSweepPrepared,
                PreviousTimestepContactPairs =
                    PreviousTimestepContactPairs,
                TimestepInteractionPairs =
                    BroadPhaseCandidates.Pairs,
                ClassificationBodyPairs =
                    Classification.BodyPairs,
                IncrementalCacheState =
                    Persistent.IncrementalCacheState,
                ClassificationResults =
                    Classification.Results,
                ClassificationState =
                    Classification.State,
#if RTS_CONTACT_DIAGNOSTICS
                Telemetry = Classification.Telemetry,
#endif
                RuntimeState = runtimeState
            }.Schedule(handle);
            handle = new EvaluatePersistentPairClassificationsJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                RawPairs = Classification.BodyPairs.AsDeferredJobArray(),
                PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody =
                    Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                PreviousContacts = Persistent.PersistentPredictiveContacts.AsDeferredJobArray(),
                PreviousContactIndex =
                    Persistent.PersistentContactIndex,
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                PhaseState = Classification.State,
                Results = Classification.Results.AsDeferredJobArray(),
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
                SubstepCount = substepCount,
                ScheduleStartSubstep = 0
            }.Schedule(
                Classification.Results,
                SoftPairBatchSize,
                handle);
            ScheduleClassificationPublication(
                ref handle,
                runtimeState,
                1);
            SchedulePersistentClassificationFinalization(
                ref handle,
                runtimeState);
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
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                StepStates = Body.StepStates,
                SubstepDeltaTime = substepDeltaTime,
                GridOrigin = Obstacles.Geometry.Origin,
                CellRadius = Obstacles.Geometry.CellRadius
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);
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
                    Bodies = Body.Bodies,
                    NavigationStates = Body.NavigationStates,
                    MotionIntents = Body.MotionIntents,
                    MotionEvidence = Body.MotionEvidence,
                    StepStates = Body.StepStates,
                    PersistentProxies =
                        Persistent.PersistentSweptProxies.AsDeferredJobArray(),
                    PersistentProxyIndexByBody =
                        Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                    PersistentCacheState = Persistent.IncrementalCacheState,
                    DirtyFlagsByBody =
                        Repair.DirtyFlagsByBody,
                    Duration = substepDeltaTime,
                    Skin = Configuration.PredictiveSkin,
                    Margin = Configuration.TimestepContactMargin,
                    GuardMargin = Configuration.GuardEnvelopeMargin,
                    GridOrigin = Obstacles.Geometry.Origin,
                    CellRadius = Obstacles.Geometry.CellRadius,
                    FromSolvedPosition = 1,
                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon,
                    DetectPersistentDirty = 0
                }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);
                ScheduleFullSweepBroadPhase(
                    ref handle,
                    false,
                    runtimeState);
            }

            handle = new ValidateBaseMotionBodiesJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                BodyCount = Body.Bodies.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                DirtyBodies = Repair.DirtyBodies,
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                BlockOffsets = Solver.DirtyBodyBlockOffsets,
                DirtyBodies = Repair.DirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                BodyStatistics = Solver.ParallelBodyResults,
                BodyCount = Body.Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new FinalizeEnvelopeEscapesJob
            {
                DirtyBodies =
                    Repair.DirtyBodies,
                ParallelBodyStatistics =
                    Solver.ParallelBodyResults,
                RuntimeState = runtimeState,
                EnableTimestepContactSetCache = (byte)(
                    Configuration.EnableTimestepContactSetCache ? 1 : 0),
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = Diagnostics.ContactStatistics,
                IncrementalStatistics =
                    Diagnostics.IncrementalStatistics,
#endif
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new PrepareRepairPredictionBodiesJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                DirtyBodies = Repair.DirtyBodies.AsDeferredJobArray(),
                Duration = math.max(
                    substepDeltaTime,
                    (substepCount - substepIndex) * substepDeltaTime),
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = Obstacles.Geometry.Origin,
                CellRadius = Obstacles.Geometry.CellRadius,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(Repair.DirtyBodies, ParallelBodyBatchSize, handle);

            handle = new RefreshDirtyBodiesJob
            {
                Bodies = Body.Bodies,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                DirtyBodies =
                    Repair.DirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody =
                    Repair.DirtyFlagsByBody,
                PersistentProxies =
                    Persistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody =
                    Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                CacheState = Persistent.IncrementalCacheState,
                Results = Repair.BodyRefreshResults,
                ObstacleVersion = Configuration.ObstacleVersion,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate =
                    Configuration.SoftAvoidanceResponseRate,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SubstepCount = substepCount,
                SoftAvoidanceVelocitySolver =
                    Configuration.SoftAvoidanceVelocitySolver,
                PredictivePairGenerationEnabled =
                    (byte)(Configuration.EnablePredictivePairGeneration ? 1 : 0),
                PredictiveContactsEnabled =
                    (byte)(Configuration.EnablePredictiveContacts ? 1 : 0),
                Enabled = (byte)(
                    Configuration.EnableTimestepContactSetCache &&
                    Configuration.EnablePersistentContactCache ? 1 : 0)
            }.Schedule(
                Repair.DirtyBodies,
                ParallelBodyBatchSize,
                handle);
            handle = new ReduceDirtyBodyRefreshJob
            {
                DirtyBodies =
                    Repair.DirtyBodies.AsDeferredJobArray(),
                Results = Repair.BodyRefreshResults,
                Summary = Repair.BodyRefreshSummary,
                Enabled = (byte)(
                    Configuration.EnableTimestepContactSetCache &&
                    Configuration.EnablePersistentContactCache ? 1 : 0)
            }.Schedule(handle);
            if (Configuration.EnableTimestepContactSetCache &&
                Configuration.EnablePersistentContactCache)
            {
                ScheduleDirtyContactScheduleCompaction(ref handle);
                ScheduleFullSweepBroadPhase(
                    ref handle,
                    true,
                    runtimeState);
                SchedulePersistentTopologyPublication(ref handle);
            }
            else if (Configuration.EnableTimestepContactSetCache)
            {
                ScheduleFullSweepBroadPhase(
                    ref handle,
                    true,
                    runtimeState);
                SchedulePersistentTopologyPublication(ref handle);
            }
            else
            {
                SchedulePersistentTopologyPublication(ref handle);
            }
            ScheduleSubstepRepairPreparation(
                ref handle,
                runtimeState);
            handle = new EvaluatePersistentPairClassificationsJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                RawPairs = Classification.BodyPairs.AsDeferredJobArray(),
                PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody =
                    Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                PreviousContacts = Persistent.PersistentPredictiveContacts.AsDeferredJobArray(),
                PreviousContactIndex =
                    Persistent.PersistentContactIndex,
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                PhaseState = Classification.State,
                Results = Classification.Results.AsDeferredJobArray(),
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
            }.Schedule(Classification.Results, SoftPairBatchSize, handle);

            ScheduleClassificationPublication(
                ref handle,
                runtimeState,
                2);
            ScheduleSubstepRepairPublication(
                ref handle,
                runtimeState,
                substepIndex);
            handle = new ValidateConsumerViewsJob
            {
                Configuration = Configuration,
                Bodies = Body.Bodies,
                SoftAvoidancePairs =
                    NarrowPhaseConstraints.SoftInteractions,
                TimestepContactPairs =
                    NarrowPhaseConstraints.HardContacts,
                PredictiveContactSchedule =
                    Certificate.Schedule,
                DirtyBodies =
                    Repair.DirtyBodies,
                InteractionCertificate =
                    Certificate.Certificate,
                InteractionCertificateViolations =
                    Certificate.Violations,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = Diagnostics.ContactStatistics,
#endif
                RequireDirtyBodies = (byte)(
                    Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndValidationRepair,
                handle);
#endif

            SoftAvoidanceJob prepareSoft = CreateSoftAvoidanceJob();
            prepareSoft.Operation = SoftAvoidanceOperation.PrepareParallelWorkset;
            prepareSoft.RuntimeState = runtimeState;
#if RTS_CONTACT_DIAGNOSTICS
            prepareSoft.BlockStatistics = blockStatistics;
#endif
            handle = prepareSoft.Schedule(handle);

            handle = new InitializeSoftAvoidanceBodiesJob
            {
                Bodies = Body.Bodies,
                StepStates = Body.StepStates,
                AvoidanceStates = Body.AvoidanceStates,
                Grid = Obstacles.Cells,
                GridOrigin = Obstacles.Geometry.Origin,
                GridDimensions = Obstacles.Geometry.Dimensions,
                CellRadius = Obstacles.Geometry.CellRadius,
                SoftShell = Configuration.SoftAvoidanceShell
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

            var evaluateSoftPairsJob = new EvaluateSoftAvoidancePairsJob
            {
                Bodies = Body.Bodies,
                StepStates = Body.StepStates,
                Pairs = NarrowPhaseConstraints.SoftInteractions.AsDeferredJobArray(),
                Contributions = SoftAvoidanceResources.PairContributions.AsDeferredJobArray(),
                SolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftShell = Configuration.SoftAvoidanceShell,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SubstepDeltaTime = substepDeltaTime
            };
            handle = evaluateSoftPairsJob.Schedule(
                NarrowPhaseConstraints.SoftInteractions,
                SoftPairBatchSize,
                handle);

#if RTS_CONTACT_DIAGNOSTICS
            // ReduceSoftAvoidanceBlocksJob carries block telemetry counters
            // (no Configuration.EnableDiagnostics runtime gate) so benchmarks with the oracle
            // off still capture valid soft-avoidance stats.
            {
                handle = new ReduceSoftAvoidanceBlocksJob
                {
                    Contributions = SoftAvoidanceResources.PairContributions.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);
            }
#endif

            handle = new GatherSoftAvoidanceBodiesJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                AvoidanceStates = Body.AvoidanceStates,
                Pairs = NarrowPhaseConstraints.SoftInteractions.AsDeferredJobArray(),
                Contributions = SoftAvoidanceResources.PairContributions.AsDeferredJobArray(),
                IncidentOffsets = SoftAvoidanceResources.IncidentOffsets,
                IncidentPairIndices = SoftAvoidanceResources.IncidentPairIndices.AsDeferredJobArray(),
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftShell = Configuration.SoftAvoidanceShell,
                ClampToEnvelope = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Soft-avoidance finalize carries timing/escape counters (no
            // Configuration.EnableDiagnostics runtime gate) so benchmarks with the oracle
            // disabled still capture valid soft-avoidance telemetry.
            {
                handle = new ReduceSoftEscapeBlocksJob
                {
                    EscapeFlags = Solver.EnvelopeEscapeFlags,
                    EscapeCountsByBlock = Solver.DirtyBodyBlockOffsets,
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                SoftAvoidanceJob finalizeSoft = CreateSoftAvoidanceJob();
                finalizeSoft.Operation = SoftAvoidanceOperation.FinalizeParallel;
                finalizeSoft.RuntimeState = runtimeState;
                finalizeSoft.BlockStatistics = blockStatistics;
                finalizeSoft.EscapeCountsByBlock = Solver.DirtyBodyBlockOffsets;
                finalizeSoft.EscapeBlockCount = escapeBlockCount;
                handle = finalizeSoft.Schedule(handle);
            }
#endif

#if RTS_CONTACT_DIAGNOSTICS
            handle = BeginStageTiming(runtimeState, handle);
#endif
            handle = new PredictUnconstrainedBodiesJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                AvoidanceStates = Body.AvoidanceStates,
                StepStates = Body.StepStates,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndMotion,
                handle);
            handle = BeginStageTiming(runtimeState, handle);
#endif

            handle = new ValidatePredictedContactEnvelopeBodiesJob
            {
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                PredictiveSkin = Configuration.PredictiveSkin
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                BodyCount = Body.Bodies.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                DirtyBodies = Repair.DirtyBodies,
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = Solver.EnvelopeEscapeFlags,
                BlockOffsets = Solver.DirtyBodyBlockOffsets,
                DirtyBodies = Repair.DirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                Bodies = Body.Bodies,
                NavigationStates = Body.NavigationStates,
                MotionIntents = Body.MotionIntents,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                BodyStatistics = Solver.ParallelBodyResults,
                BodyCount = Body.Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new FinalizeEnvelopeEscapesJob
            {
                DirtyBodies =
                    Repair.DirtyBodies,
                ParallelBodyStatistics =
                    Solver.ParallelBodyResults,
                RuntimeState = runtimeState,
                EnableTimestepContactSetCache = (byte)(
                    Configuration.EnableTimestepContactSetCache ? 1 : 0),
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = Diagnostics.ContactStatistics,
                IncrementalStatistics =
                    Diagnostics.IncrementalStatistics,
#endif
                SubstepIndex = substepIndex
            }.Schedule(handle);
            SchedulePersistentRepairStages(
                ref handle,
                runtimeState,
                substepIndex,
                substepCount,
                substepDeltaTime,
                true);
            SchedulePredictiveContactActivation(
                ref handle,
                runtimeState,
                substepIndex,
                substepCount);
            ScheduleActiveConstraintIncidentIndex(ref handle);
            handle = new ValidateConsumerViewsJob
            {
                Configuration = Configuration,
                Bodies = Body.Bodies,
                SoftAvoidancePairs =
                    NarrowPhaseConstraints.SoftInteractions,
                TimestepContactPairs =
                    NarrowPhaseConstraints.HardContacts,
                PredictiveContactSchedule =
                    Certificate.Schedule,
                DirtyBodies =
                    Repair.DirtyBodies,
                InteractionCertificate =
                    Certificate.Certificate,
                InteractionCertificateViolations =
                    Certificate.Violations,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = Diagnostics.ContactStatistics,
#endif
                RequireDirtyBodies = (byte)(
                    Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray()
            }.Schedule(NarrowPhaseConstraints.HardContacts, SoftPairBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndValidationRepair,
                handle);
#endif

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                ConstraintSolverJob beginIteration =
                    CreateConstraintSolverJob();
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
                    Bodies = Body.Bodies,
                    StepStates = Body.StepStates,
                    Grid = Obstacles.Cells,
                    GridOrigin = Obstacles.Geometry.Origin,
                    GridDimensions = Obstacles.Geometry.Dimensions,
                    CellRadius = Obstacles.Geometry.CellRadius,
                    CorrectedBodyFlags = Solver.CorrectedBodyFlags,
                    BodyStatistics = Solver.ParallelBodyResults
                }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

                handle = new CountAndReduceWallBlocksJob
                {
                    CorrectedBodyFlags = Solver.CorrectedBodyFlags,
                    BodyStatistics = Solver.ParallelBodyResults,
                    BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixCorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = Solver.DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = Solver.CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterCorrectedBodiesJob
                {
                    CorrectedBodyFlags = Solver.CorrectedBodyFlags,
                    BlockOffsets = Solver.DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = Solver.CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new FinalizeWallIterationJob
                {
                    Configuration = Configuration,
                    Bodies = Body.Bodies,
                    MotionEvidence = Body.MotionEvidence,
                    StepStates = Body.StepStates,
                    DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                    DirtyBodies = Repair.DirtyBodies,
                    InteractionCertificate = Certificate.Certificate,
                    CertificateViolations = Certificate.Violations,
                    CorrectedBodyFlags =
                        Solver.CorrectedBodyFlags,
                    CorrectedBodyIndices =
                        Solver.CorrectedBodyIndices,
                    ParallelBodyStatistics =
                        Solver.ParallelBodyResults,
#if RTS_CONTACT_DIAGNOSTICS
                    IterationState = Execution.SolverIterationState,
                    BlockStatistics =
                        Execution.JacobiBlockStatistics,
                    IncrementalStatistics = Diagnostics.IncrementalStatistics,
                    Statistics = Diagnostics.ContactStatistics,
#endif
                    RuntimeState = runtimeState,
                    SubstepIndex = substepIndex,
                    BodyBlockCount = escapeBlockCount
                }.Schedule(handle);
                SchedulePersistentRepairStages(
                    ref handle,
                    runtimeState,
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    false);
                ScheduleActiveConstraintIncidentIndex(ref handle);

                if (!useJacobiSolver)
                {
                    ConstraintSolverJob solveGaussSeidel =
                        CreateConstraintSolverJob();
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
                            Bodies = Body.Bodies,
                            StepStates = Body.StepStates,
                            Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
                            Corrections = Solver.JacobiPairCorrections.AsDeferredJobArray(),
                            DiagnosticPairCandidates =
                                Diagnostics.ParallelPairCandidates.AsDeferredJobArray(),
                            DiagnosticSelectedEntity = DiagnosticSelectedEntity
                        }.Schedule(NarrowPhaseConstraints.HardContacts, JacobiPairBatchSize, handle);
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
                            Bodies = Body.Bodies,
                            StepStates = Body.StepStates,
                            Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
                            Corrections = Solver.JacobiPairCorrections.AsDeferredJobArray()
                        }.Schedule(NarrowPhaseConstraints.HardContacts, JacobiPairBatchSize, handle);
                    }

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = Solver.JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

#if RTS_CONTACT_DIAGNOSTICS
                    if (captureSelectedPairs)
                    {
                        handle = new CountParallelSimulationDebuggerPairBlocksJob
                        {
                            Candidates =
                                Diagnostics.ParallelPairCandidates,
                            Blocks = blockStatistics
                        }.Schedule(blockStatistics, 1, handle);

                        handle = new PrefixParallelSimulationDebuggerPairsJob
                        {
                            Blocks = blockStatistics,
                            Scratch = Diagnostics.ParallelPairScratch
                        }.Schedule(handle);

                        handle = new ScatterParallelSimulationDebuggerPairsJob
                        {
                            Candidates =
                                Diagnostics.ParallelPairCandidates,
                            Blocks = blockStatistics,
                            Scratch = Diagnostics.ParallelPairScratch
                        }.Schedule(blockStatistics, 1, handle);

                        ConstraintSolverJob mergeDebuggerPairs =
                            CreateConstraintSolverJob();
                        mergeDebuggerPairs.Operation =
                            ConstraintSolverOperation.MergeParallelDebuggerPairs;
                        handle = mergeDebuggerPairs.Schedule(handle);
                    }
#endif

                    handle = new GatherAndApplyParallelJacobiBodiesJob
                    {
                        RecoveryOnly = 0,
                        RuntimeState = runtimeState,
                        Bodies = Body.Bodies,
                        StepStates = Body.StepStates,
                        Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
                        Corrections = Solver.JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = Solver.ActiveIncidentOffsets,
                        IncidentPairIndices =
                            Solver.ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = Solver.CorrectedBodyFlags
                    }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);
                }

                handle = new FinalizeContactIterationJob
                {
                    Configuration = Configuration,
                    Bodies = Body.Bodies,
                    MotionEvidence = Body.MotionEvidence,
                    StepStates = Body.StepStates,
                    TimestepContactPairs =
                        NarrowPhaseConstraints.HardContacts,
                    DirtyFlagsByBody = Repair.DirtyFlagsByBody,
                    DirtyBodies = Repair.DirtyBodies,
                    InteractionCertificate = Certificate.Certificate,
                    CertificateViolations = Certificate.Violations,
                    CorrectedBodyFlags =
                        Solver.CorrectedBodyFlags,
                    CorrectedBodyIndices =
                        Solver.CorrectedBodyIndices,
#if RTS_CONTACT_DIAGNOSTICS
                    IterationState = Execution.SolverIterationState,
                    BlockStatistics =
                        Execution.JacobiBlockStatistics,
                    IncrementalStatistics = Diagnostics.IncrementalStatistics,
                    Statistics = Diagnostics.ContactStatistics,
                    IterationDiagnostics =
                        Diagnostics.Iterations,
#endif
                    RuntimeState = runtimeState,
                    SubstepIndex = substepIndex,
                    IterationIndex = iterationIndex
                }.Schedule(handle);
                SchedulePersistentRepairStages(
                    ref handle,
                    runtimeState,
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    false);
                ScheduleActiveConstraintIncidentIndex(ref handle);

                if (!useJacobiSolver)
                {
                    ConstraintSolverJob recovery =
                        CreateConstraintSolverJob();
                    recovery.Operation =
                        ConstraintSolverOperation.SolveGaussSeidelRecovery;
                    recovery.RuntimeState = runtimeState;
                    recovery.SubstepIndex = substepIndex;
                    handle = recovery.Schedule(handle);
                }
                else
                {
                    ConstraintSolverJob prepareRecovery =
                        CreateConstraintSolverJob();
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
                        Bodies = Body.Bodies,
                        StepStates = Body.StepStates,
                        Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
                        Corrections =
                            Solver.JacobiPairCorrections.AsDeferredJobArray()
                    }.Schedule(
                        NarrowPhaseConstraints.HardContacts,
                        JacobiPairBatchSize,
                        handle);

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = Solver.JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

                    handle = new GatherAndApplyParallelJacobiBodiesJob
                    {
                        RecoveryOnly = 1,
                        RuntimeState = runtimeState,
                        Bodies = Body.Bodies,
                        StepStates = Body.StepStates,
                        Pairs = NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
                        Corrections =
                            Solver.JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = Solver.ActiveIncidentOffsets,
                        IncidentPairIndices =
                            Solver.ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = Solver.CorrectedBodyFlags
                    }.Schedule(
                        Body.Bodies.Length,
                        ParallelBodyBatchSize,
                        handle);

                    ConstraintSolverJob finalizeRecovery =
                        CreateConstraintSolverJob();
                    finalizeRecovery.Operation =
                        ConstraintSolverOperation.FinalizeJacobiRecovery;
                    finalizeRecovery.RuntimeState = runtimeState;
                    handle = finalizeRecovery.Schedule(handle);
                }
            }

#if RTS_CONTACT_DIAGNOSTICS
            // FinalizeSubstepTelemetry accumulates IterationNanoseconds and
            // constraint counters (no Configuration.EnableDiagnostics runtime gate) so
            // benchmarks with the oracle disabled still capture valid iteration
            // timing.
            {
                ConstraintSolverJob beginFinalizeSubstep =
                    CreateConstraintSolverJob();
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
                Bodies = Body.Bodies,
                StepStates = Body.StepStates,
                BodyStatistics = Solver.ParallelBodyResults,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(Body.Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Velocity-body block reduce + finalize carry timing/counters
            // (no Configuration.EnableDiagnostics runtime gate) for benchmarks with oracle off.
            {
                handle = new ReduceVelocityBodyBlocksJob
                {
                    BodyStatistics = Solver.ParallelBodyResults,
                    BodyCount = Body.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                ConstraintSolverJob finalizeVelocity =
                    CreateConstraintSolverJob();
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
        // and the cross-stage ratios. No Configuration.EnableDiagnostics runtime gate so benchmarks
        // with the oracle disabled still get valid pipeline-total telemetry.
        {
            ConstraintSolverJob finalizePipeline =
                CreateConstraintSolverJob();
            finalizePipeline.Operation = ConstraintSolverOperation.FinalizePipeline;
            finalizePipeline.RuntimeState = runtimeState;
            return finalizePipeline.Schedule(handle);
        }
#else
        return handle;
#endif
        }
        catch
        {
            // A scheduling safety exception can happen after earlier stages were
            // already enqueued. Complete the last successfully scheduled handle
            // before the caller releases the frame-owned NativeContainers.
            handle.Complete();
            throw;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    private JobHandle BeginStageTiming(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        JobHandle dependency) =>
        new ContactPipelineTimingJob
        {
            Operation = ContactPipelineTimingOperation.Begin,
            RuntimeState = runtimeState,
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics = Diagnostics.IncrementalStatistics
        }.Schedule(dependency);

    private JobHandle EndStageTiming(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        ContactPipelineTimingOperation operation,
        JobHandle dependency) =>
        new ContactPipelineTimingJob
        {
            Operation = operation,
            RuntimeState = runtimeState,
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics = Diagnostics.IncrementalStatistics
        }.Schedule(dependency);
#endif

    private void SchedulePersistentRepairStages(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState,
        int substepIndex,
        int substepCount,
        float substepDeltaTime,
        bool forceNonPersistentFullSweep)
    {
        handle = new PrepareRepairPredictionBodiesJob
        {
            Bodies = Body.Bodies,
            NavigationStates = Body.NavigationStates,
            MotionIntents = Body.MotionIntents,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            DirtyBodies =
                Repair.DirtyBodies
                    .AsDeferredJobArray(),
            Duration = math.max(
                substepDeltaTime,
                (substepCount - substepIndex) * substepDeltaTime),
            Skin = Configuration.PredictiveSkin,
            Margin = Configuration.TimestepContactMargin,
            GridOrigin = Obstacles.Geometry.Origin,
            CellRadius = Obstacles.Geometry.CellRadius,
            SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
            SoftAvoidanceResponseRate =
                Configuration.SoftAvoidanceResponseRate,
            SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = Configuration.RvoTimeHorizon,
            Enabled = (byte)(
                Configuration.EnableTimestepContactSetCache ? 1 : 0)
        }.Schedule(
            Repair.DirtyBodies,
            ParallelBodyBatchSize,
            handle);

        handle = new RefreshDirtyBodiesJob
        {
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            DirtyBodies =
                Repair.DirtyBodies
                    .AsDeferredJobArray(),
            DirtyFlagsByBody =
                Repair.DirtyFlagsByBody,
            PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
            PersistentProxyIndexByBody = Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
            CacheState = Persistent.IncrementalCacheState,
            Results = Repair.BodyRefreshResults,
            ObstacleVersion = Configuration.ObstacleVersion,
            GuardMargin = Configuration.GuardEnvelopeMargin,
            PredictiveSkin = Configuration.PredictiveSkin,
            TimestepContactMargin = Configuration.TimestepContactMargin,
            SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
            SoftAvoidanceResponseRate =
                Configuration.SoftAvoidanceResponseRate,
            RvoTimeHorizon = Configuration.RvoTimeHorizon,
            SubstepCount = substepCount,
            SoftAvoidanceVelocitySolver =
                Configuration.SoftAvoidanceVelocitySolver,
            PredictivePairGenerationEnabled = (byte)(
                Configuration.EnablePredictivePairGeneration ? 1 : 0),
            PredictiveContactsEnabled = (byte)(
                Configuration.EnablePredictiveContacts ? 1 : 0),
            Enabled = (byte)(
                Configuration.EnableTimestepContactSetCache &&
                Configuration.EnablePersistentContactCache ? 1 : 0)
        }.Schedule(
            Repair.DirtyBodies,
            ParallelBodyBatchSize,
            handle);
        handle = new ReduceDirtyBodyRefreshJob
        {
            DirtyBodies = Repair.DirtyBodies
                .AsDeferredJobArray(),
            Results = Repair.BodyRefreshResults,
            Summary = Repair.BodyRefreshSummary,
            Enabled = (byte)(
                Configuration.EnableTimestepContactSetCache &&
                Configuration.EnablePersistentContactCache ? 1 : 0)
        }.Schedule(handle);

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            ScheduleDirtyContactScheduleCompaction(ref handle);
            ScheduleFullSweepBroadPhase(
                ref handle,
                true,
                runtimeState);
            SchedulePersistentTopologyPublication(ref handle);
        }
        else
        {
            ScheduleFullSweepBroadPhase(
                ref handle,
                Configuration.EnableTimestepContactSetCache ||
                !forceNonPersistentFullSweep,
                runtimeState,
                !Configuration.EnableTimestepContactSetCache);
            SchedulePersistentTopologyPublication(ref handle);
        }
            ScheduleSubstepRepairPreparation(
                ref handle,
                runtimeState);
        handle = new EvaluatePersistentPairClassificationsJob
        {
            Bodies = Body.Bodies,
            NavigationStates = Body.NavigationStates,
            MotionIntents = Body.MotionIntents,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            RawPairs =
                Classification.BodyPairs
                    .AsDeferredJobArray(),
            PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
            PersistentProxyIndexByBody =
                Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
            PreviousContacts = Persistent.PersistentPredictiveContacts.AsDeferredJobArray(),
            PreviousContactIndex =
                Persistent.PersistentContactIndex,
            DirtyFlagsByBody =
                Repair.DirtyFlagsByBody,
            PhaseState =
                Classification.State,
            Results = Classification.Results.AsDeferredJobArray(),
            PredictiveSkin = Configuration.PredictiveSkin,
            TimestepContactMargin = Configuration.TimestepContactMargin,
            SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
            SoftAvoidanceResponseRate =
                Configuration.SoftAvoidanceResponseRate,
            SoftAvoidanceVelocitySolver =
                Configuration.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = Configuration.RvoTimeHorizon,
            EnablePredictivePairGeneration = (byte)(
                Configuration.EnablePredictivePairGeneration ? 1 : 0),
            EnablePredictiveContacts = (byte)(
                Configuration.EnablePredictiveContacts ? 1 : 0),
            SubstepCount = substepCount,
            ScheduleStartSubstep = substepIndex
        }.Schedule(
            Classification.Results,
            SoftPairBatchSize,
            handle);
        ScheduleClassificationPublication(
            ref handle,
            runtimeState,
            2);
        ScheduleSubstepRepairPublication(
            ref handle,
            runtimeState,
            substepIndex);
        handle = new ValidateConsumerViewsJob
        {
            Configuration = Configuration,
            Bodies = Body.Bodies,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            PredictiveContactSchedule =
                Certificate.Schedule,
            DirtyBodies =
                Repair.DirtyBodies,
            InteractionCertificate =
                Certificate.Certificate,
            InteractionCertificateViolations =
                Certificate.Violations,
            RuntimeState = runtimeState,
            SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
#endif
            RequireDirtyBodies = 1
        }.Schedule(handle);
    }

    private void ScheduleActiveConstraintIncidentIndex(
        ref JobHandle handle)
    {
        byte enabled = (byte)(
            Configuration.ContactPositionSolver ==
            ContactPositionSolverMode.Jacobi
                ? 1
                : 0);
        handle = new PrepareActiveIncidentIndexJob
        {
            Certificate =
                Certificate.Certificate,
            Pairs = NarrowPhaseConstraints.HardContacts,
            BodyWorkset =
                Solver.ActiveIncidentBodyWorkset,
            PairWorkset =
                Solver.ActiveIncidentPairWorkset,
            IncidentPairIndices =
                Solver.ActiveIncidentPairIndices,
            State = Solver.ActiveIncidentIndexState,
            BodyCount = Body.Bodies.Length,
            Enabled = enabled
        }.Schedule(handle);
        handle = new ClearActiveIncidentCountsJob
        {
            Workset = Solver.ActiveIncidentBodyWorkset
                .AsDeferredJobArray(),
            Counts = Solver.ActiveIncidentWriteCursors
        }.Schedule(
            Solver.ActiveIncidentBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        handle = new CountActiveIncidentPairsJob
        {
            Workset = Solver.ActiveIncidentPairWorkset
                .AsDeferredJobArray(),
            Pairs = NarrowPhaseConstraints.HardContacts
                .AsDeferredJobArray(),
            Counts = Solver.ActiveIncidentWriteCursors
        }.Schedule(
            Solver.ActiveIncidentPairWorkset,
            JacobiPairBatchSize,
            handle);
        handle = new PrefixActiveIncidentPairsJob
        {
            CountsAndWriteCursors =
                Solver.ActiveIncidentWriteCursors,
            Offsets = Solver.ActiveIncidentOffsets,
            IncidentPairIndices =
                Solver.ActiveIncidentPairIndices,
            State = Solver.ActiveIncidentIndexState,
            Pairs = NarrowPhaseConstraints.HardContacts,
            BodyCount = Body.Bodies.Length
        }.Schedule(handle);
        handle = new ScatterActiveIncidentPairsJob
        {
            Workset = Solver.ActiveIncidentPairWorkset
                .AsDeferredJobArray(),
            Pairs = NarrowPhaseConstraints.HardContacts
                .AsDeferredJobArray(),
            WriteCursors =
                Solver.ActiveIncidentWriteCursors,
            IncidentPairIndices = Solver.ActiveIncidentPairIndices.AsDeferredJobArray()
        }.Schedule(
            Solver.ActiveIncidentPairWorkset,
            JacobiPairBatchSize,
            handle);
        handle = new SortActiveIncidentRangesJob
        {
            Workset = Solver.ActiveIncidentBodyWorkset
                .AsDeferredJobArray(),
            Offsets = Solver.ActiveIncidentOffsets,
            IncidentPairIndices = Solver.ActiveIncidentPairIndices.AsDeferredJobArray()
        }.Schedule(
            Solver.ActiveIncidentBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        handle = new ResizeParallelContactWorksetsJob
        {
            Pairs = NarrowPhaseConstraints.HardContacts,
            Corrections = Solver.JacobiPairCorrections,
#if RTS_CONTACT_DIAGNOSTICS
            DebuggerPairCandidates = Diagnostics.ParallelPairCandidates,
            Blocks = Execution.JacobiBlockStatistics,
#endif
            Enabled = enabled
        }.Schedule(handle);
    }

    private void ScheduleDirtyContactScheduleCompaction(
        ref JobHandle handle)
    {
        handle = new PrepareDirtyContactScheduleBlocksJob
        {
            Contacts =
                Persistent.PersistentPredictiveContacts,
            Schedule = Certificate.Schedule,
            DirtyBodies =
                Repair.DirtyBodies,
            BlockCounts =
                Repair.ScheduleBlockCounts,
            BlockOffsets =
                Repair.ScheduleBlockOffsets,
            BlockSize = SoftPairBatchSize
        }.Schedule(handle);
        handle = new CountDirtyContactScheduleJob
        {
            Contacts = Persistent.PersistentPredictiveContacts.AsDeferredJobArray(),
            Schedule = Certificate.Schedule.AsDeferredJobArray(),
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            DirtyFlagsByBody =
                Repair.DirtyFlagsByBody,
            BlockCounts = Repair.ScheduleBlockCounts.AsDeferredJobArray(),
            ScheduleCursor =
                Certificate.ScheduleCursor,
            BlockSize = SoftPairBatchSize
        }.Schedule(
            Repair.ScheduleBlockCounts,
            1,
            handle);
        handle = new PrefixDirtyContactScheduleJob
        {
            BlockCounts = Repair.ScheduleBlockCounts.AsDeferredJobArray(),
            BlockOffsets = Repair.ScheduleBlockOffsets.AsDeferredJobArray(),
            ContactScratch =
                Repair.PersistentContactCompactionScratch,
            ScheduleScratch =
                Certificate.ScheduleScratch
        }.Schedule(handle);
        handle = new ScatterDirtyContactScheduleJob
        {
            Contacts = Persistent.PersistentPredictiveContacts.AsDeferredJobArray(),
            Schedule = Certificate.Schedule.AsDeferredJobArray(),
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            DirtyFlagsByBody =
                Repair.DirtyFlagsByBody,
            BlockCounts = Repair.ScheduleBlockCounts.AsDeferredJobArray(),
            BlockOffsets = Repair.ScheduleBlockOffsets.AsDeferredJobArray(),
            ContactScratch =
                Repair.PersistentContactCompactionScratch
                    .AsDeferredJobArray(),
            ScheduleScratch = Certificate.ScheduleScratch.AsDeferredJobArray(),
            ScheduleCursor =
                Certificate.ScheduleCursor,
            BlockSize = SoftPairBatchSize
        }.Schedule(
            Repair.ScheduleBlockCounts,
            1,
            handle);
        handle = new CommitDirtyContactScheduleJob
        {
            ContactScratch =
                Repair.PersistentContactCompactionScratch,
            ScheduleScratch =
                Certificate.ScheduleScratch,
            Contacts =
                Persistent.PersistentPredictiveContacts,
            ContactIndex =
                Persistent.PersistentContactIndex,
            Schedule = Certificate.Schedule,
            ScheduleCursor =
                Certificate.ScheduleCursor,
            DirtyBodies =
                Repair.DirtyBodies
        }.Schedule(handle);
        handle = new BuildDirtyContactIndexJob
        {
            Contacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            ContactIndex =
                Persistent.PersistentContactIndex.AsParallelWriter()
        }.Schedule(
            Persistent.PersistentPredictiveContacts,
            SoftPairBatchSize,
            handle);
    }

    private void ScheduleFullSweepBroadPhase(
        ref JobHandle handle,
        bool requireDirtyBodies,
        NativeReference<ContactPipelineExecutionState> runtimeState,
        bool prepareSolvedTrajectory = false,
        bool requireValidPersistentCache = false)
    {
        handle = new PrepareFullSweepBroadPhaseJob
        {
            DirtyBodies = Repair.DirtyBodies,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            BodyWorkset = BroadPhase.FullSweepBodyWorkset,
            RuntimeState = runtimeState,
            CacheState = Persistent.IncrementalCacheState,
            PersistentProxies =
                Persistent.PersistentSweptProxies,
            PersistentProxyIndexByBody =
                Persistent.PersistentProxyIndexByBody,
            Configuration = Configuration,
            BodyCount = Body.Bodies.Length,
            RequireDirtyBodies = (byte)(requireDirtyBodies ? 1 : 0),
            RequireValidPersistentCache = (byte)(
                requireValidPersistentCache ? 1 : 0)
        }.Schedule(handle);
        if (prepareSolvedTrajectory)
        {
            handle = new PrepareSubstepContactPredictionBodiesJob
            {
                Workset = BroadPhase.FullSweepBodyWorkset
                    .AsDeferredJobArray(),
                Bodies = Body.Bodies,
                MotionEvidence = Body.MotionEvidence,
                StepStates = Body.StepStates,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin =
                    Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SoftSolverMode =
                    Configuration.SoftAvoidanceVelocitySolver
            }.Schedule(
                BroadPhase.FullSweepBodyWorkset,
                ParallelBodyBatchSize,
                handle);
        }
        handle = new CountBodyCellsJob
        {
            Workset = BroadPhase.FullSweepBodyWorkset
                .AsDeferredJobArray(),
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            BodyCellCounts = BroadPhase.BodyCellCounts,
            GridOrigin = Obstacles.Geometry.Origin,
            GridDimensions = Obstacles.Geometry.Dimensions,
            CellRadius = Obstacles.Geometry.CellRadius
        }.Schedule(
            BroadPhase.FullSweepBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        handle = new PrefixBodyCellsJob
        {
            BodyCellCounts = BroadPhase.BodyCellCounts,
            BodyCellOffsets = BroadPhase.BodyCellOffsets,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            SweptCellEntries = BroadPhase.SweptCellEntries,
            CellPairCounts = BroadPhase.CellPairCounts,
            CellPairOffsets = BroadPhase.CellPairOffsets
        }.Schedule(handle);
        handle = new ScatterBodyCellsJob
        {
            Workset = BroadPhase.FullSweepBodyWorkset
                .AsDeferredJobArray(),
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            BodyCellCounts = BroadPhase.BodyCellCounts,
            BodyCellOffsets = BroadPhase.BodyCellOffsets,
            SweptCellEntries =
                BroadPhase.SweptCellEntries.AsDeferredJobArray(),
            GridOrigin = Obstacles.Geometry.Origin,
            GridDimensions = Obstacles.Geometry.Dimensions,
            CellRadius = Obstacles.Geometry.CellRadius
        }.Schedule(
            BroadPhase.FullSweepBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        ScheduleBodyCellSort(ref handle);
        handle = new CountCellPairsJob
        {
            SweptCellEntries =
                BroadPhase.SweptCellEntries.AsDeferredJobArray(),
            MotionEvidence = Body.MotionEvidence,
            CellPairCounts =
                BroadPhase.CellPairCounts.AsDeferredJobArray(),
            GridOrigin = Obstacles.Geometry.Origin,
            GridDimensions = Obstacles.Geometry.Dimensions,
            CellRadius = Obstacles.Geometry.CellRadius
        }.Schedule(
            BroadPhase.CellPairCounts,
            SoftPairBatchSize,
            handle);
        handle = new PrefixCellPairsJob
        {
            CellPairCounts =
                BroadPhase.CellPairCounts.AsDeferredJobArray(),
            CellPairOffsets =
                BroadPhase.CellPairOffsets.AsDeferredJobArray(),
            Pairs = BroadPhase.CollisionPairs
        }.Schedule(handle);
        handle = new ScatterCellPairsJob
        {
            SweptCellEntries =
                BroadPhase.SweptCellEntries.AsDeferredJobArray(),
            MotionEvidence = Body.MotionEvidence,
            CellPairCounts =
                BroadPhase.CellPairCounts.AsDeferredJobArray(),
            CellPairOffsets =
                BroadPhase.CellPairOffsets.AsDeferredJobArray(),
            Pairs = BroadPhase.CollisionPairs.AsDeferredJobArray(),
            GridOrigin = Obstacles.Geometry.Origin,
            GridDimensions = Obstacles.Geometry.Dimensions,
            CellRadius = Obstacles.Geometry.CellRadius
        }.Schedule(
            BroadPhase.CellPairCounts,
            SoftPairBatchSize,
            handle);
        ScheduleBroadPhasePairSortAndPublish(ref handle);

    }

    private void ScheduleBodyCellSort(ref JobHandle handle)
    {
        const int cellSortBlockSize = 256;
        handle = new PrepareBodyCellSortJob
        {
            Entries = BroadPhase.SweptCellEntries,
            BlockWorkset = BroadPhase.CellSortBlockWorkset,
            Scratch = BroadPhase.CellSortScratch,
            BlockSize = cellSortBlockSize
        }.Schedule(handle);
        handle = new SortBodyCellBlocksJob
        {
            Workset =
                BroadPhase.CellSortBlockWorkset.AsDeferredJobArray(),
            Entries =
                BroadPhase.SweptCellEntries.AsDeferredJobArray(),
            BlockSize = cellSortBlockSize
        }.Schedule(
            BroadPhase.CellSortBlockWorkset,
            1,
            handle);

        long maximumEntryCount =
            (long)Body.Bodies.Length *
            math.max(
                1L,
                (long)Obstacles.Geometry.Dimensions.x *
                Obstacles.Geometry.Dimensions.y);
        long maximumBlockCount =
            (maximumEntryCount + cellSortBlockSize - 1L) /
            cellSortBlockSize;
        int mergePassCount = 0;
        for (long width = 1; width < maximumBlockCount; width <<= 1)
            mergePassCount++;
        for (int mergePass = 0;
             mergePass < mergePassCount;
             mergePass++)
        {
            bool sourceIsEntries = (mergePass & 1) == 0;
            handle = new MergeBodyCellBlocksJob
            {
                Workset = BroadPhase.CellSortBlockWorkset
                    .AsDeferredJobArray(),
                Source = sourceIsEntries
                    ? BroadPhase.SweptCellEntries.AsDeferredJobArray()
                    : BroadPhase.CellSortScratch.AsDeferredJobArray(),
                Destination = sourceIsEntries
                    ? BroadPhase.CellSortScratch.AsDeferredJobArray()
                    : BroadPhase.SweptCellEntries.AsDeferredJobArray(),
                BlockSize = cellSortBlockSize,
                MergePass = mergePass
            }.Schedule(
                BroadPhase.CellSortBlockWorkset,
                1,
                handle);
        }
        if ((mergePassCount & 1) != 0)
        {
            handle = new CopyBodyCellSortResultJob
            {
                Workset = BroadPhase.CellSortBlockWorkset
                    .AsDeferredJobArray(),
                Source =
                    BroadPhase.CellSortScratch.AsDeferredJobArray(),
                Destination =
                    BroadPhase.SweptCellEntries.AsDeferredJobArray(),
                BlockSize = cellSortBlockSize
            }.Schedule(
                BroadPhase.CellSortBlockWorkset,
                1,
                handle);
        }
    }

    private void ScheduleBroadPhasePairSortAndPublish(
        ref JobHandle handle)
    {
        const int pairSortBlockSize = 256;
        handle = new PrepareBroadPhasePairSortJob
        {
            Pairs = BroadPhase.CollisionPairs,
            BlockWorkset = BroadPhase.PairSortBlockWorkset,
            Scratch = BroadPhase.PairSortScratch,
            BlockSize = pairSortBlockSize
        }.Schedule(handle);
        handle = new SortBroadPhasePairBlocksJob
        {
            Workset =
                BroadPhase.PairSortBlockWorkset.AsDeferredJobArray(),
            Pairs = BroadPhase.CollisionPairs.AsDeferredJobArray(),
            BlockSize = pairSortBlockSize
        }.Schedule(
            BroadPhase.PairSortBlockWorkset,
            1,
            handle);

        long maximumPairCount =
            (long)Body.Bodies.Length *
            math.max(0, Body.Bodies.Length - 1) / 2L;
        long maximumBlockCount =
            (maximumPairCount + pairSortBlockSize - 1L) /
            pairSortBlockSize;
        int mergePassCount = 0;
        for (long width = 1; width < maximumBlockCount; width <<= 1)
            mergePassCount++;
        for (int mergePass = 0;
             mergePass < mergePassCount;
             mergePass++)
        {
            bool sourceIsPairs = (mergePass & 1) == 0;
            handle = new MergeBroadPhasePairBlocksJob
            {
                Workset = BroadPhase.PairSortBlockWorkset
                    .AsDeferredJobArray(),
                Source = sourceIsPairs
                    ? BroadPhase.CollisionPairs.AsDeferredJobArray()
                    : BroadPhase.PairSortScratch.AsDeferredJobArray(),
                Destination = sourceIsPairs
                    ? BroadPhase.PairSortScratch.AsDeferredJobArray()
                    : BroadPhase.CollisionPairs.AsDeferredJobArray(),
                BlockSize = pairSortBlockSize,
                MergePass = mergePass
            }.Schedule(
                BroadPhase.PairSortBlockWorkset,
                1,
                handle);
        }
        if ((mergePassCount & 1) != 0)
        {
            handle = new CopyBroadPhasePairSortResultJob
            {
                Workset = BroadPhase.PairSortBlockWorkset
                    .AsDeferredJobArray(),
                Source =
                    BroadPhase.PairSortScratch.AsDeferredJobArray(),
                Destination =
                    BroadPhase.CollisionPairs.AsDeferredJobArray(),
                BlockSize = pairSortBlockSize
            }.Schedule(
                BroadPhase.PairSortBlockWorkset,
                1,
                handle);
        }
        handle = new DeduplicateAndPublishBroadPhasePairsJob
        {
            Pairs = BroadPhase.CollisionPairs,
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            RuntimeState = Execution.PipelineRuntimeState
        }.Schedule(handle);
    }

    private void SchedulePersistentTopologyPublication(
        ref JobHandle handle)
    {
        handle = new PreparePersistentTopologyPublicationJob
        {
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            PreviousProxies =
                BroadPhase.PreviousProxies,
            PersistentProxies =
                Persistent.PersistentSweptProxies,
            ProxyIndexByBody =
                Persistent.PersistentProxyIndexByBody,
            PersistentPairs =
                Persistent.PersistentNeighborPairs,
            BodyCount = Body.Bodies.Length
        }.Schedule(handle);
        handle = new BuildPersistentProxiesJob
        {
            Workset = BroadPhase.FullSweepBodyWorkset
                .AsDeferredJobArray(),
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            PreviousProxies = BroadPhase.PreviousProxies.AsDeferredJobArray(),
            PreviousProxyIndexByBody =
                Persistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
            PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
            GuardMargin = Configuration.GuardEnvelopeMargin,
            SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
            SoftAvoidanceResponseRate =
                Configuration.SoftAvoidanceResponseRate,
            SoftAvoidanceVelocitySolver =
                Configuration.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = Configuration.RvoTimeHorizon
        }.Schedule(
            BroadPhase.FullSweepBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        handle = new BuildPersistentProxyIndexJob
        {
            Workset = BroadPhase.FullSweepBodyWorkset
                .AsDeferredJobArray(),
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            PersistentProxies = Persistent.PersistentSweptProxies.AsDeferredJobArray(),
            ProxyIndexByBody = Persistent.PersistentProxyIndexByBody.AsDeferredJobArray()
        }.Schedule(
            BroadPhase.FullSweepBodyWorkset,
            ParallelBodyBatchSize,
            handle);
        handle = new PublishPersistentNeighborPairsJob
        {
            Workset = BroadPhase.CollisionPairs.AsDeferredJobArray(),
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            BodyPairs = BroadPhaseCandidates.Pairs.AsDeferredJobArray(),
            Bodies = Body.Bodies,
            CacheState = Persistent.IncrementalCacheState,
            PersistentPairs = Persistent.PersistentNeighborPairs.AsDeferredJobArray()
        }.Schedule(
            BroadPhase.CollisionPairs,
            SoftPairBatchSize,
            handle);
        handle = new FinalizePersistentTopologyPublicationJob
        {
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            PersistentPairs =
                Persistent.PersistentNeighborPairs,
            CacheState = Persistent.IncrementalCacheState,
            PersistentSpatialMembershipEpoch = Persistent.PersistentSpatialMembershipEpoch,
            PersistentIncidentLookupEpoch = Persistent.PersistentIncidentLookupEpoch,
            Configuration = Configuration,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
            BodyCount = Body.Bodies.Length
        }.Schedule(handle);
    }

    private void SchedulePersistentReusePublication(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        handle = new PreparePersistentReusePublicationJob
        {
            RuntimeState = runtimeState,
            CacheState = Persistent.IncrementalCacheState,
            PersistentProxies =
                Persistent.PersistentSweptProxies,
            PersistentProxyIndexByBody =
                Persistent.PersistentProxyIndexByBody,
            DirtyBodies =
                Repair.DirtyBodies,
            PersistentPairs =
                Persistent.PersistentNeighborPairs,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            PairWorkset =
                BroadPhase.PersistentReusePairWorkset,
            MappedPairs = BroadPhase.CollisionPairs,
            BodyCount = Body.Bodies.Length,
            Configuration = Configuration,
            Enabled = 1
        }.Schedule(handle);
        handle = new MapPersistentReusePairsJob
        {
            Workset = BroadPhase.PersistentReusePairWorkset
                .AsDeferredJobArray(),
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            PersistentPairs = Persistent.PersistentNeighborPairs.AsDeferredJobArray(),
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            MappedPairs = BroadPhase.CollisionPairs.AsDeferredJobArray()
        }.Schedule(
            BroadPhase.PersistentReusePairWorkset,
            SoftPairBatchSize,
            handle);
        ScheduleBroadPhasePairSortAndPublish(ref handle);
        handle = new FinalizePersistentReusePublicationJob
        {
            RuntimeState = runtimeState,
            CacheState = Persistent.IncrementalCacheState,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics
#endif
        }.Schedule(handle);
    }

    private void ScheduleSubstepRepairPreparation(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        JobHandle prepare = new PrepareSubstepRepairBuffersJob
        {
            Configuration = Configuration,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
            PreviousTimestepContactPairs =
                PreviousTimestepContactPairs,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            TimestepInteractionPairs = BroadPhaseCandidates.Pairs,
            ClassificationBodyPairs =
                Classification.BodyPairs,
            Pairs = BroadPhase.CollisionPairs,
            DirtyBodies = Repair.DirtyBodies,
            IncrementalCacheState = Persistent.IncrementalCacheState,
            ClassificationResults =
                Classification.Results,
            ClassificationState =
                Classification.State,
#if RTS_CONTACT_DIAGNOSTICS
            Telemetry =
                Classification.Telemetry,
#endif
            RuntimeState = runtimeState
        }.Schedule(handle);
        handle = prepare;

        JobHandle copyInteractions =
            new CopySubstepRepairInteractionPairsJob
            {
                Source =
                    BroadPhaseCandidates.Pairs.AsDeferredJobArray(),
                Destination = Classification.BodyPairs
                    .AsDeferredJobArray()
            }.Schedule(
                Classification.BodyPairs,
                SoftPairBatchSize,
                prepare);
        handle = copyInteractions;
        JobHandle copyPrevious =
            new CopyPreviousTimestepContactPairsJob
            {
                Source = NarrowPhaseConstraints.HardContacts
                    .AsDeferredJobArray(),
                Destination = PreviousTimestepContactPairs
                    .AsDeferredJobArray()
            }.Schedule(
                PreviousTimestepContactPairs,
                SoftPairBatchSize,
                prepare);
        handle = JobHandle.CombineDependencies(
            copyInteractions,
            copyPrevious);
    }

    private void SchedulePersistentClassificationFinalization(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        handle = new PublishPersistentClassificationStateJob
        {
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            PersistentNeighborPairs =
                Persistent.PersistentNeighborPairs,
            Constraints = BroadPhase.CollisionPairs,
            Schedule = Certificate.Schedule,
            CacheState = Persistent.IncrementalCacheState,
            PhaseState = Classification.State,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
            RuntimeState = runtimeState
        }.Schedule(handle);
#if RTS_CONTACT_DIAGNOSTICS
        handle = new ValidatePersistentClassificationOraclesJob
        {
            Configuration = Configuration,
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            OracleContactPairs =
                Diagnostics.IncrementalOracleContactPairs,
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
            Telemetry = Classification.Telemetry,
            PhaseState = Classification.State,
            RuntimeState = runtimeState
        }.Schedule(handle);
#endif
        handle = new FinalizePersistentClassificationCertificateJob
        {
            Configuration = Configuration,
            Bodies = Body.Bodies,
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            PersistentNeighborPairs =
                Persistent.PersistentNeighborPairs,
            Schedule = Certificate.Schedule,
            CacheState = Persistent.IncrementalCacheState,
            InteractionCertificate = Certificate.Certificate,
            CertificateViolations = Certificate.Violations,
            PhaseState = Classification.State,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
            RuntimeState = runtimeState
        }.Schedule(handle);
    }

    private void ScheduleSubstepRepairPublication(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState,
        int substepIndex)
    {
        handle = new PublishSubstepRepairClassificationJob
        {
            PersistentNeighborPairs =
                Persistent.PersistentNeighborPairs,
            Constraints = BroadPhase.CollisionPairs,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            Schedule = Certificate.Schedule,
            CacheState = Persistent.IncrementalCacheState,
            PhaseState = Classification.State,
            ActiveIncidentIndexState =
                Solver.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
            RuntimeState = runtimeState
        }.Schedule(handle);
        ScheduleRepairContactViewPublication(
            ref handle,
            runtimeState);
        handle = new ClearRepairedEnvelopeEscapeJob
        {
            Workset = Repair.DirtyBodies.AsDeferredJobArray(),
            PhaseState = Classification.State,
            RuntimeState = runtimeState,
            MotionEvidence = Body.MotionEvidence
        }.Schedule(
            Repair.DirtyBodies,
            ParallelBodyBatchSize,
            handle);
        handle = new PreparePersistentIncidentLookupJob
        {
            RuntimeState = runtimeState,
            PhaseState = Classification.State,
            CacheState = Persistent.IncrementalCacheState,
            Pairs = Persistent.PersistentNeighborPairs,
            IncidentPairLookup =
                Persistent.PersistentIncidentPairLookup,
            IncidentLookupEpoch =
                Persistent.PersistentIncidentLookupEpoch,
            PairWorkset = Repair.PersistentIncidentPairWorkset,
            RebuildPairCount =
                Repair.PersistentIncidentRebuildPairCount,
            Enabled = (byte)(
                Configuration.EnablePersistentContactCache ? 1 : 0)
        }.Schedule(handle);
        handle = new ScatterPersistentIncidentLookupJob
        {
            Workset =
                Repair.PersistentIncidentPairWorkset
                    .AsDeferredJobArray(),
            Pairs =
                Persistent.PersistentNeighborPairs
                    .AsDeferredJobArray(),
            IncidentPairLookup =
                Persistent.PersistentIncidentPairLookup
                    .AsParallelWriter()
        }.Schedule(
            Repair.PersistentIncidentPairWorkset,
            SoftPairBatchSize,
            handle);
        handle = new FinalizePersistentIncidentLookupJob
        {
            CacheState = Persistent.IncrementalCacheState,
            Pairs = Persistent.PersistentNeighborPairs,
            RebuildPairCount =
                Repair.PersistentIncidentRebuildPairCount,
            IncidentLookupEpoch =
                Persistent.PersistentIncidentLookupEpoch
        }.Schedule(handle);
        handle = new FinalizeSubstepRepairCertificateJob
        {
            Configuration = Configuration,
            Bodies = Body.Bodies,
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            PersistentNeighborPairs =
                Persistent.PersistentNeighborPairs,
            Schedule = Certificate.Schedule,
            CacheState = Persistent.IncrementalCacheState,
            InteractionCertificate = Certificate.Certificate,
            CertificateViolations = Certificate.Violations,
            PhaseState = Classification.State,
            FullSweepPrepared = BroadPhase.FullSweepPrepared,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
            Telemetry = Classification.Telemetry,
#endif
            RuntimeState = runtimeState,
            SubstepIndex = substepIndex
        }.Schedule(handle);
    }

    private void ScheduleClassificationPublication(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState,
        byte expectedCommitState)
    {
        const int blockSize = 64;
        handle = new PrepareClassificationPublicationJob
        {
            RuntimeState = runtimeState,
            PhaseState = Classification.State,
            Results = Classification.Results,
            Records = Classification.PublicationRecords,
            Blocks = Classification.PublicationBlocks,
            BlockWorkset = Classification.PublicationBlockWorkset,
            PersistentContacts =
                Persistent.PersistentPredictiveContacts,
            Constraints = BroadPhase.CollisionPairs,
            InitialTimestepContacts =
                NarrowPhaseConstraints.HardContacts,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            Schedule = Certificate.Schedule,
            ContactIndex =
                Persistent.PersistentContactIndex,
            ExpectedCommitState = expectedCommitState,
            BlockSize = blockSize
        }.Schedule(handle);
        handle = new MaterializeClassificationPublicationJob
        {
            Results = Classification.Results.AsDeferredJobArray(),
            Records =
                Classification.PublicationRecords.AsDeferredJobArray(),
            PersistentContacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray()
        }.Schedule(
            Classification.PublicationRecords,
            SoftPairBatchSize,
            handle);
        handle = new BuildClassificationContactIndexJob
        {
            Workset =
                Classification.PublicationRecords.AsDeferredJobArray(),
            Contacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            ContactIndex =
                Persistent.PersistentContactIndex.AsParallelWriter()
        }.Schedule(
            Classification.PublicationRecords,
            SoftPairBatchSize,
            handle);
        handle = new CountClassificationPublicationBlocksJob
        {
            Workset =
                Classification.PublicationBlockWorkset
                    .AsDeferredJobArray(),
            Records =
                Classification.PublicationRecords.AsDeferredJobArray(),
            Contacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            Results = Classification.Results.AsDeferredJobArray(),
            Blocks =
                Classification.PublicationBlocks.AsDeferredJobArray(),
            BlockSize = blockSize
        }.Schedule(
            Classification.PublicationBlockWorkset,
            1,
            handle);
        handle = new PrefixClassificationPublicationJob
        {
            Blocks = Classification.PublicationBlocks,
            Constraints = BroadPhase.CollisionPairs,
            InitialTimestepContacts =
                NarrowPhaseConstraints.HardContacts,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            Schedule = Certificate.Schedule,
            ScheduleCursor = Certificate.ScheduleCursor,
            CacheState = Persistent.IncrementalCacheState,
            PublishInitialTimestepContacts =
                (byte)(expectedCommitState == 1 ? 1 : 0),
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
        }.Schedule(handle);
        handle = new ScatterClassificationPublicationBlocksJob
        {
            Workset =
                Classification.PublicationBlockWorkset
                    .AsDeferredJobArray(),
            Records =
                Classification.PublicationRecords.AsDeferredJobArray(),
            Contacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            Blocks =
                Classification.PublicationBlocks.AsDeferredJobArray(),
            Constraints =
                BroadPhase.CollisionPairs.AsDeferredJobArray(),
            InitialTimestepContacts =
                NarrowPhaseConstraints.HardContacts
                    .AsDeferredJobArray(),
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions
                    .AsDeferredJobArray(),
            Schedule = Certificate.Schedule.AsDeferredJobArray(),
            BlockSize = blockSize,
            PublishInitialTimestepContacts =
                (byte)(expectedCommitState == 1 ? 1 : 0)
        }.Schedule(
            Classification.PublicationBlockWorkset,
            1,
            handle);
    }

}
}
