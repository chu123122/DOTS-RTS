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
        if (Configuration.EnableDiagnostics)
        {
            captureSelectedPairs = ConstraintSolver.DiagnosticSelectedEntity != Entity.Null &&
                (ConstraintSolver.SimulationDebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
                (ConstraintSolver.SimulationDebuggerCaptureMask &
                 SimulationDebuggerCaptureMask.SelectedPairs) != 0;
        }
#endif
        int escapeBlockCount =
            (CertificationBody.Bodies.Length + ParallelBodyBatchSize - 1) / ParallelBodyBatchSize;
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
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                PersistentProxies = CertificationPersistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody = CertificationPersistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                PersistentCacheState = CertificationPersistent.IncrementalCacheState,
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                GridOrigin = CertificationEnvironment.GridOrigin,
                CellRadius = CertificationEnvironment.CellRadius,
                FromSolvedPosition = 0,
                DetectPersistentDirty = (byte)(Configuration.EnablePersistentContactCache ? 1 : 0),
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialDirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                    BodyCount = CertificationBody.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialDirtyBodiesJob
                {
                    BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                    DirtyBodies = CertificationPersistent.IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialDirtyBodiesJob
                {
                    DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                    BlockOffsets = CertificationSolver.DirtyBodyBlockOffsets,
                    DirtyBodies = CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                    BodyCount = CertificationBody.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }
        }

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = new PrepareCurrentBodyIndexJob
            {
                CurrentBodyIndexByEntity =
                    CertificationViews.CurrentBodyIndexByEntity,
                BodyCount = CertificationBody.Bodies.Length
            }.Schedule(handle);
            handle = new BuildCurrentBodyIndexJob
            {
                Bodies = CertificationBody.Bodies,
                CurrentBodyIndexByEntity = CertificationViews
                    .CurrentBodyIndexByEntity.AsParallelWriter()
            }.Schedule(
                CertificationBody.Bodies.Length,
                ParallelBodyBatchSize,
                handle);
            handle = ScheduleDirtyContactScheduleCompaction(handle);
        }

        if (Configuration.EnableTimestepContactSetCache)
            handle = ScheduleFullSweepBroadPhase(handle);

        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = new CertificationStageKernel.PreparePersistentClassificationJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState
            }.Schedule(handle);
            handle = new CertificationStageKernel.EvaluatePersistentPairClassificationsJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                RawPairs = CertificationViews.ClassificationBodyPairs.AsDeferredJobArray(),
                PersistentProxies = CertificationPersistent.PersistentSweptProxies.AsDeferredJobArray(),
                PreviousContacts = CertificationPersistent.PersistentPredictiveContacts.AsDeferredJobArray(),
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                PhaseState = CertificationPersistent.PersistentClassificationState,
                Results = CertificationPersistent.PersistentClassificationResults.AsDeferredJobArray(),
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
                CertificationPersistent.PersistentClassificationResults,
                SoftPairBatchSize,
                handle);
            handle = new CertificationStageKernel.CommitPersistentClassificationJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }
        else
        {
            handle = new CertificationStageKernel.BuildInitialContactSetJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState
            }.Schedule(handle);
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
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                SubstepDeltaTime = substepDeltaTime,
                GridOrigin = CertificationEnvironment.GridOrigin,
                CellRadius = CertificationEnvironment.CellRadius
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);
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
                    Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                    Duration = substepDeltaTime,
                    Skin = Configuration.PredictiveSkin,
                    Margin = Configuration.TimestepContactMargin,
                    GridOrigin = CertificationEnvironment.GridOrigin,
                    CellRadius = CertificationEnvironment.CellRadius,
                    FromSolvedPosition = 1,
                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon,
                    DetectPersistentDirty = 0
                }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);
                handle = ScheduleFullSweepBroadPhase(handle);
            }

            handle = new ValidateBaseMotionBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                BodyCount = CertificationBody.Bodies.Length,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies,
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                BlockOffsets = CertificationSolver.DirtyBodyBlockOffsets,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                BodyCount = CertificationBody.Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new CertificationStageKernel.FinalizeEnvelopeEscapesJob
            {
                Configuration = Configuration,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies,
                BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
                IncrementalStatistics = CertificationDiagnostics.IncrementalStatistics,
                Statistics = CertificationDiagnostics.Statistics
#endif
            }.Schedule(handle);

            handle = new PrepareRepairPredictionBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                Duration = math.max(
                    substepDeltaTime,
                    (substepCount - substepIndex) * substepDeltaTime),
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = CertificationEnvironment.GridOrigin,
                CellRadius = CertificationEnvironment.CellRadius,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(CertificationPersistent.IncrementalDirtyBodies, ParallelBodyBatchSize, handle);

            handle = new RefreshDirtyBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                DirtyBodies =
                    CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody =
                    CertificationPersistent.IncrementalDirtyFlagsByBody,
                PersistentProxies =
                    CertificationPersistent.PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody =
                    CertificationPersistent.PersistentProxyIndexByBody.AsDeferredJobArray(),
                CacheState = CertificationPersistent.IncrementalCacheState,
                Results = CertificationPersistent.DirtyBodyRefreshResults,
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
                CertificationPersistent.IncrementalDirtyBodies,
                ParallelBodyBatchSize,
                handle);
            handle = new ReduceDirtyBodyRefreshJob
            {
                DirtyBodies =
                    CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                Results = CertificationPersistent.DirtyBodyRefreshResults,
                Summary = CertificationPersistent.DirtyBodyRefreshSummary,
                Enabled = (byte)(
                    Configuration.EnableTimestepContactSetCache &&
                    Configuration.EnablePersistentContactCache ? 1 : 0)
            }.Schedule(handle);
            if (Configuration.EnableTimestepContactSetCache &&
                Configuration.EnablePersistentContactCache)
                handle = ScheduleDirtyContactScheduleCompaction(handle);

            handle = new CertificationStageKernel.PrepareSubstepRepairJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Solver = CertificationSolver,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);

            handle = new CertificationStageKernel.EvaluatePersistentPairClassificationsJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                RawPairs = CertificationViews.ClassificationBodyPairs.AsDeferredJobArray(),
                PersistentProxies = CertificationPersistent.PersistentSweptProxies.AsDeferredJobArray(),
                PreviousContacts = CertificationPersistent.PersistentPredictiveContacts.AsDeferredJobArray(),
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                PhaseState = CertificationPersistent.PersistentClassificationState,
                Results = CertificationPersistent.PersistentClassificationResults.AsDeferredJobArray(),
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
            }.Schedule(CertificationPersistent.PersistentClassificationResults, SoftPairBatchSize, handle);

            handle = new CertificationStageKernel.CommitSubstepRepairJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Solver = CertificationSolver,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);
            handle = new CertificationStageKernel.ValidateConsumerViewsJob
            {
                Configuration = Configuration,
                Bodies = CertificationBody.Bodies,
                SoftAvoidancePairs = CertificationViews.SoftAvoidancePairs,
                TimestepContactPairs = CertificationViews.TimestepContactPairs,
                PredictiveContactSchedule = CertificationPersistent.PredictiveContactSchedule,
                InteractionCertificate = CertificationPersistent.InteractionCertificate,
                InteractionCertificateViolations =
                    CertificationPersistent.InteractionCertificateViolations,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = CertificationDiagnostics.Statistics
#endif
            }.Schedule(handle);
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
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                Grid = CertificationEnvironment.Grid,
                GridOrigin = CertificationEnvironment.GridOrigin,
                GridDimensions = CertificationEnvironment.GridDimensions,
                CellRadius = CertificationEnvironment.CellRadius,
                SoftShell = Configuration.SoftAvoidanceShell
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

            var evaluateSoftPairsJob = new EvaluateSoftAvoidancePairsJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                Pairs = CertificationViews.SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftAvoidance.SoftPairContributions.AsDeferredJobArray(),
                SolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftShell = Configuration.SoftAvoidanceShell,
                RvoTimeHorizon = Configuration.RvoTimeHorizon,
                SubstepDeltaTime = substepDeltaTime
            };
            handle = evaluateSoftPairsJob.Schedule(
                CertificationViews.SoftAvoidancePairs,
                SoftPairBatchSize,
                handle);

#if RTS_CONTACT_DIAGNOSTICS
            // ReduceSoftAvoidanceBlocksJob carries block telemetry counters
            // (no Configuration.EnableDiagnostics runtime gate) so benchmarks with the oracle
            // off still capture valid soft-avoidance stats.
            {
                handle = new ReduceSoftAvoidanceBlocksJob
                {
                    Contributions = SoftAvoidance.SoftPairContributions.AsDeferredJobArray(),
                    Blocks = blockStatistics.AsDeferredJobArray()
                }.Schedule(blockStatistics, 1, handle);
            }
#endif

            handle = new GatherSoftAvoidanceBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                Pairs = CertificationViews.SoftAvoidancePairs.AsDeferredJobArray(),
                Contributions = SoftAvoidance.SoftPairContributions.AsDeferredJobArray(),
                IncidentOffsets = SoftAvoidance.SoftIncidentOffsets,
                IncidentPairIndices = SoftAvoidance.SoftIncidentPairIndices.AsDeferredJobArray(),
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime,
                PredictiveSkin = Configuration.PredictiveSkin,
                TimestepContactMargin = Configuration.TimestepContactMargin,
                SoftShell = Configuration.SoftAvoidanceShell,
                ClampToEnvelope = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Soft-avoidance finalize carries timing/escape counters (no
            // Configuration.EnableDiagnostics runtime gate) so benchmarks with the oracle
            // disabled still capture valid soft-avoidance telemetry.
            {
                handle = new ReduceSoftEscapeBlocksJob
                {
                    EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                    EscapeCountsByBlock = CertificationSolver.DirtyBodyBlockOffsets,
                    BodyCount = CertificationBody.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                SoftAvoidanceJob finalizeSoft = SoftAvoidance;
                finalizeSoft.Operation = SoftAvoidanceOperation.FinalizeParallel;
                finalizeSoft.RuntimeState = runtimeState;
                finalizeSoft.BlockStatistics = blockStatistics;
                finalizeSoft.EscapeCountsByBlock = CertificationSolver.DirtyBodyBlockOffsets;
                finalizeSoft.EscapeBlockCount = escapeBlockCount;
                handle = finalizeSoft.Schedule(handle);
            }
#endif

#if RTS_CONTACT_DIAGNOSTICS
            handle = BeginStageTiming(runtimeState, handle);
#endif
            handle = new PredictUnconstrainedBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SettledMultiplier = Configuration.SettledSoftAvoidanceMultiplier,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);
#if RTS_CONTACT_DIAGNOSTICS
            handle = EndStageTiming(
                runtimeState,
                ContactPipelineTimingOperation.EndMotion,
                handle);
            handle = BeginStageTiming(runtimeState, handle);
#endif

            handle = new ValidatePredictedContactEnvelopeBodiesJob
            {
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                PredictiveSkin = Configuration.PredictiveSkin
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

            handle = new CountEnvelopeEscapeBlocksJob
            {
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                BodyCount = CertificationBody.Bodies.Length,
                Enabled = 1
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new PrefixEnvelopeEscapesJob
            {
                BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies,
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                BlockCount = escapeBlockCount
            }.Schedule(handle);

            handle = new ScatterEnvelopeEscapesJob
            {
                EscapeFlags = CertificationSolver.EnvelopeEscapeFlags,
                BlockOffsets = CertificationSolver.DirtyBodyBlockOffsets,
                DirtyBodies = CertificationPersistent.IncrementalDirtyBodies.AsDeferredJobArray(),
                DirtyFlagsByBody = CertificationPersistent.IncrementalDirtyFlagsByBody,
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                BodyCount = CertificationBody.Bodies.Length
            }.Schedule(escapeBlockCount, 1, handle);

            handle = new CertificationStageKernel.FinalizePreparedSubstepJob
            {
                Environment = CertificationEnvironment,
                Body = CertificationBody,
                Views = CertificationViews,
                Persistent = CertificationPersistent,
                Solver = CertificationSolver,
                Diagnostics = CertificationDiagnostics,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex
            }.Schedule(handle);
            handle = new CertificationStageKernel.ValidateConsumerViewsJob
            {
                Configuration = Configuration,
                Bodies = CertificationBody.Bodies,
                SoftAvoidancePairs = CertificationViews.SoftAvoidancePairs,
                TimestepContactPairs = CertificationViews.TimestepContactPairs,
                PredictiveContactSchedule = CertificationPersistent.PredictiveContactSchedule,
                InteractionCertificate = CertificationPersistent.InteractionCertificate,
                InteractionCertificateViolations =
                    CertificationPersistent.InteractionCertificateViolations,
                RuntimeState = runtimeState,
                SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = CertificationDiagnostics.Statistics
#endif
            }.Schedule(handle);

            handle = new ResetContactPairStateJob
            {
                Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray()
            }.Schedule(CertificationViews.TimestepContactPairs, SoftPairBatchSize, handle);
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
                    Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                    Grid = CertificationEnvironment.Grid,
                    GridOrigin = CertificationEnvironment.GridOrigin,
                    GridDimensions = CertificationEnvironment.GridDimensions,
                    CellRadius = CertificationEnvironment.CellRadius,
                    CorrectedBodyFlags = CertificationSolver.CorrectedBodyFlags,
                    BodyStatistics = CertificationSolver.ParallelBodyStatistics
                }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

                handle = new CountAndReduceWallBlocksJob
                {
                    CorrectedBodyFlags = CertificationSolver.CorrectedBodyFlags,
                    BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                    BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                    BodyCount = CertificationBody.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new PrefixCorrectedBodiesJob
                {
                    BlockOffsetsAndCounts = CertificationSolver.DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CertificationSolver.CorrectedBodyIndices,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);

                handle = new ScatterCorrectedBodiesJob
                {
                    CorrectedBodyFlags = CertificationSolver.CorrectedBodyFlags,
                    BlockOffsets = CertificationSolver.DirtyBodyBlockOffsets,
                    CorrectedBodyIndices = CertificationSolver.CorrectedBodyIndices.AsDeferredJobArray(),
                    BodyCount = CertificationBody.Bodies.Length
                }.Schedule(escapeBlockCount, 1, handle);

                handle = new CertificationStageKernel.FinalizeWallIterationJob
                {
                    Environment = CertificationEnvironment,
                    Body = CertificationBody,
                    Views = CertificationViews,
                    Persistent = CertificationPersistent,
                    Solver = CertificationSolver,
                    Diagnostics = CertificationDiagnostics,
                    RuntimeState = runtimeState,
                    SubstepIndex = substepIndex,
                    BodyBlockCount = escapeBlockCount
                }.Schedule(handle);

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
                            Bodies = CertificationBody.Bodies,
                            NavigationStates = CertificationBody.NavigationStates,
                            MotionIntents = CertificationBody.MotionIntents,
                            MotionEvidence = CertificationBody.MotionEvidence,
                            StepStates = CertificationBody.StepStates,
                            Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray(),
                            Corrections = CertificationSolver.JacobiPairCorrections.AsDeferredJobArray(),
                            DiagnosticPairCandidates =
                                CertificationDiagnostics.ParallelSimulationDebuggerPairCandidates.AsDeferredJobArray(),
                            DiagnosticSelectedEntity = ConstraintSolver.DiagnosticSelectedEntity
                        }.Schedule(CertificationViews.TimestepContactPairs, JacobiPairBatchSize, handle);
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
                            Bodies = CertificationBody.Bodies,
                            NavigationStates = CertificationBody.NavigationStates,
                            MotionIntents = CertificationBody.MotionIntents,
                            MotionEvidence = CertificationBody.MotionEvidence,
                            StepStates = CertificationBody.StepStates,
                            Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray(),
                            Corrections = CertificationSolver.JacobiPairCorrections.AsDeferredJobArray()
                        }.Schedule(CertificationViews.TimestepContactPairs, JacobiPairBatchSize, handle);
                    }

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = CertificationSolver.JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

#if RTS_CONTACT_DIAGNOSTICS
                    if (captureSelectedPairs)
                    {
                        handle = new CountParallelSimulationDebuggerPairBlocksJob
                        {
                            Candidates =
                                CertificationDiagnostics.ParallelSimulationDebuggerPairCandidates,
                            Blocks = blockStatistics
                        }.Schedule(blockStatistics, 1, handle);

                        handle = new PrefixParallelSimulationDebuggerPairsJob
                        {
                            Blocks = blockStatistics,
                            Scratch = ConstraintSolver.ParallelSimulationDebuggerPairScratch
                        }.Schedule(handle);

                        handle = new ScatterParallelSimulationDebuggerPairsJob
                        {
                            Candidates =
                                CertificationDiagnostics.ParallelSimulationDebuggerPairCandidates,
                            Blocks = blockStatistics,
                            Scratch = ConstraintSolver.ParallelSimulationDebuggerPairScratch
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
                        Bodies = CertificationBody.Bodies,
                        NavigationStates = CertificationBody.NavigationStates,
                        MotionIntents = CertificationBody.MotionIntents,
                        MotionEvidence = CertificationBody.MotionEvidence,
                        StepStates = CertificationBody.StepStates,
                        Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray(),
                        Corrections = CertificationSolver.JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = CertificationSolver.ActiveIncidentOffsets,
                        IncidentPairIndices =
                            CertificationSolver.ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = CertificationSolver.CorrectedBodyFlags
                    }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);
                }

                handle = new CertificationStageKernel.FinalizeContactIterationJob
                {
                    Environment = CertificationEnvironment,
                    Body = CertificationBody,
                    Views = CertificationViews,
                    Persistent = CertificationPersistent,
                    Solver = CertificationSolver,
                    Diagnostics = CertificationDiagnostics,
                    RuntimeState = runtimeState,
                    SubstepIndex = substepIndex,
                    IterationIndex = iterationIndex
                }.Schedule(handle);

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
                        Bodies = CertificationBody.Bodies,
                        NavigationStates = CertificationBody.NavigationStates,
                        MotionIntents = CertificationBody.MotionIntents,
                        MotionEvidence = CertificationBody.MotionEvidence,
                        StepStates = CertificationBody.StepStates,
                        Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray(),
                        Corrections =
                            CertificationSolver.JacobiPairCorrections.AsDeferredJobArray()
                    }.Schedule(
                        CertificationViews.TimestepContactPairs,
                        JacobiPairBatchSize,
                        handle);

#if RTS_CONTACT_DIAGNOSTICS
                    handle = new ReduceParallelJacobiBlocksJob
                    {
                        Corrections = CertificationSolver.JacobiPairCorrections,
                        Blocks = blockStatistics.AsDeferredJobArray()
                    }.Schedule(blockStatistics, 1, handle);
#endif

                    handle = new GatherAndApplyParallelJacobiBodiesJob
                    {
                        RecoveryOnly = 1,
                        RuntimeState = runtimeState,
                        Bodies = CertificationBody.Bodies,
                        NavigationStates = CertificationBody.NavigationStates,
                        MotionIntents = CertificationBody.MotionIntents,
                        MotionEvidence = CertificationBody.MotionEvidence,
                        StepStates = CertificationBody.StepStates,
                        Pairs = CertificationViews.TimestepContactPairs.AsDeferredJobArray(),
                        Corrections =
                            CertificationSolver.JacobiPairCorrections.AsDeferredJobArray(),
                        IncidentOffsets = CertificationSolver.ActiveIncidentOffsets,
                        IncidentPairIndices =
                            CertificationSolver.ActiveIncidentPairIndices.AsDeferredJobArray(),
                        CorrectedBodyFlags = CertificationSolver.CorrectedBodyFlags
                    }.Schedule(
                        CertificationBody.Bodies.Length,
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
            // constraint counters (no Configuration.EnableDiagnostics runtime gate) so
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
                Bodies = CertificationBody.Bodies,
                NavigationStates = CertificationBody.NavigationStates,
                MotionIntents = CertificationBody.MotionIntents,
                MotionEvidence = CertificationBody.MotionEvidence,
                StepStates = CertificationBody.StepStates,
                BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                SubstepDeltaTime = substepDeltaTime
            }.Schedule(CertificationBody.Bodies.Length, ParallelBodyBatchSize, handle);

#if RTS_CONTACT_DIAGNOSTICS
            // Velocity-body block reduce + finalize carry timing/counters
            // (no Configuration.EnableDiagnostics runtime gate) for benchmarks with oracle off.
            {
                handle = new ReduceVelocityBodyBlocksJob
                {
                    BodyStatistics = CertificationSolver.ParallelBodyStatistics,
                    BodyCount = CertificationBody.Bodies.Length
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
        // and the cross-stage ratios. No Configuration.EnableDiagnostics runtime gate so benchmarks
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
            Statistics = CertificationDiagnostics.Statistics,
            IncrementalStatistics = CertificationDiagnostics.IncrementalStatistics
        }.Schedule(dependency);

    private JobHandle EndStageTiming(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        ContactPipelineTimingOperation operation,
        JobHandle dependency) =>
        new ContactPipelineTimingJob
        {
            Operation = operation,
            RuntimeState = runtimeState,
            Statistics = CertificationDiagnostics.Statistics,
            IncrementalStatistics = CertificationDiagnostics.IncrementalStatistics
        }.Schedule(dependency);
#endif

    private JobHandle ScheduleDirtyContactScheduleCompaction(
        JobHandle handle)
    {
        handle = new PrepareDirtyContactScheduleBlocksJob
        {
            Contacts =
                CertificationPersistent.PersistentPredictiveContacts,
            Schedule = CertificationPersistent.PredictiveContactSchedule,
            BlockCounts =
                CertificationPersistent.DirtyContactScheduleBlockCounts,
            BlockOffsets =
                CertificationPersistent.DirtyContactScheduleBlockOffsets,
            BlockSize = SoftPairBatchSize
        }.Schedule(handle);
        handle = new CountDirtyContactScheduleJob
        {
            Contacts = CertificationPersistent
                .PersistentPredictiveContacts.AsDeferredJobArray(),
            Schedule = CertificationPersistent
                .PredictiveContactSchedule.AsDeferredJobArray(),
            CurrentBodyIndexByEntity =
                CertificationViews.CurrentBodyIndexByEntity,
            DirtyFlagsByBody =
                CertificationPersistent.IncrementalDirtyFlagsByBody,
            BlockCounts = CertificationPersistent
                .DirtyContactScheduleBlockCounts.AsDeferredJobArray(),
            ScheduleCursor =
                CertificationPersistent.PredictiveContactScheduleCursor,
            BlockSize = SoftPairBatchSize,
            Enabled = 1
        }.Schedule(
            CertificationPersistent.DirtyContactScheduleBlockCounts,
            1,
            handle);
        handle = new PrefixDirtyContactScheduleJob
        {
            BlockCounts = CertificationPersistent
                .DirtyContactScheduleBlockCounts.AsDeferredJobArray(),
            BlockOffsets = CertificationPersistent
                .DirtyContactScheduleBlockOffsets.AsDeferredJobArray(),
            ContactScratch =
                CertificationPersistent.PredictiveContactScratch,
            ScheduleScratch =
                CertificationPersistent.PredictiveContactScheduleScratch
        }.Schedule(handle);
        handle = new ScatterDirtyContactScheduleJob
        {
            Contacts = CertificationPersistent
                .PersistentPredictiveContacts.AsDeferredJobArray(),
            Schedule = CertificationPersistent
                .PredictiveContactSchedule.AsDeferredJobArray(),
            CurrentBodyIndexByEntity =
                CertificationViews.CurrentBodyIndexByEntity,
            DirtyFlagsByBody =
                CertificationPersistent.IncrementalDirtyFlagsByBody,
            BlockOffsets = CertificationPersistent
                .DirtyContactScheduleBlockOffsets.AsDeferredJobArray(),
            ContactScratch = CertificationPersistent
                .PredictiveContactScratch.AsDeferredJobArray(),
            ScheduleScratch = CertificationPersistent
                .PredictiveContactScheduleScratch.AsDeferredJobArray(),
            ScheduleCursor =
                CertificationPersistent.PredictiveContactScheduleCursor,
            BlockSize = SoftPairBatchSize
        }.Schedule(
            CertificationPersistent.DirtyContactScheduleBlockCounts,
            1,
            handle);
        handle = new CommitDirtyContactScheduleJob
        {
            ContactScratch =
                CertificationPersistent.PredictiveContactScratch,
            ScheduleScratch =
                CertificationPersistent.PredictiveContactScheduleScratch,
            ContactIndex =
                CertificationPersistent.PersistentContactIndex,
            Schedule = CertificationPersistent.PredictiveContactSchedule,
            ScheduleCursor =
                CertificationPersistent.PredictiveContactScheduleCursor
        }.Schedule(handle);
        return handle;
    }

    private JobHandle ScheduleFullSweepBroadPhase(JobHandle handle)
    {
        handle = new CountBodyCellsJob
        {
            Bodies = CertificationBody.Bodies,
            MotionEvidence = CertificationBody.MotionEvidence,
            BodyCellCounts = CertificationViews.BodyCellCounts,
            GridOrigin = CertificationEnvironment.GridOrigin,
            GridDimensions = CertificationEnvironment.GridDimensions,
            CellRadius = CertificationEnvironment.CellRadius
        }.Schedule(
            CertificationBody.Bodies.Length,
            ParallelBodyBatchSize,
            handle);
        handle = new PrefixBodyCellsJob
        {
            BodyCellCounts = CertificationViews.BodyCellCounts,
            BodyCellOffsets = CertificationViews.BodyCellOffsets,
            SweptCellEntries = CertificationViews.SweptCellEntries,
            CellPairCounts = CertificationViews.CellPairCounts,
            CellPairOffsets = CertificationViews.CellPairOffsets
        }.Schedule(handle);
        handle = new ScatterBodyCellsJob
        {
            Bodies = CertificationBody.Bodies,
            MotionEvidence = CertificationBody.MotionEvidence,
            BodyCellCounts = CertificationViews.BodyCellCounts,
            BodyCellOffsets = CertificationViews.BodyCellOffsets,
            SweptCellEntries =
                CertificationViews.SweptCellEntries.AsDeferredJobArray(),
            GridOrigin = CertificationEnvironment.GridOrigin,
            GridDimensions = CertificationEnvironment.GridDimensions,
            CellRadius = CertificationEnvironment.CellRadius
        }.Schedule(
            CertificationBody.Bodies.Length,
            ParallelBodyBatchSize,
            handle);
        handle = new SortBodyCellsJob
        {
            SweptCellEntries = CertificationViews.SweptCellEntries
        }.Schedule(handle);
        handle = new CountCellPairsJob
        {
            SweptCellEntries =
                CertificationViews.SweptCellEntries.AsDeferredJobArray(),
            CellPairCounts =
                CertificationViews.CellPairCounts.AsDeferredJobArray()
        }.Schedule(
            CertificationViews.CellPairCounts,
            SoftPairBatchSize,
            handle);
        handle = new PrefixCellPairsJob
        {
            CellPairCounts =
                CertificationViews.CellPairCounts.AsDeferredJobArray(),
            CellPairOffsets =
                CertificationViews.CellPairOffsets.AsDeferredJobArray(),
            Pairs = CertificationViews.Pairs
        }.Schedule(handle);
        handle = new ScatterCellPairsJob
        {
            SweptCellEntries =
                CertificationViews.SweptCellEntries.AsDeferredJobArray(),
            CellPairCounts =
                CertificationViews.CellPairCounts.AsDeferredJobArray(),
            CellPairOffsets =
                CertificationViews.CellPairOffsets.AsDeferredJobArray(),
            Pairs = CertificationViews.Pairs.AsDeferredJobArray()
        }.Schedule(
            CertificationViews.CellPairCounts,
            SoftPairBatchSize,
            handle);
        handle = new SortAndDeduplicateBroadPhasePairsJob
        {
            Pairs = CertificationViews.Pairs,
            TimestepInteractionPairs =
                CertificationViews.TimestepInteractionPairs,
            FullSweepPrepared = CertificationViews.FullSweepPrepared
        }.Schedule(handle);

        return handle;
    }

}
}
