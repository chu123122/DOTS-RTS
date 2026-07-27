using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private SimulationDebuggerFrameSnapshot _simulationDebuggerSnapshotA;
    private SimulationDebuggerFrameSnapshot _simulationDebuggerSnapshotB;
    private bool _simulationDebuggerWriteA;
    private ulong _simulationDebuggerFrameId;
    private ulong _simulationDebuggerUpdateCounter;
    private int _simulationDebuggerTraceSignature = int.MinValue;
    private uint _simulationDebuggerLastTraceStep;
    private int _simulationDebuggerPublishTraceSignature = int.MinValue;
    private uint _simulationDebuggerLastPublishTraceStep;
    private bool _hasSimulationDebuggerPublishTraceSignature;

    private void PublishSimulationDebuggerSnapshot(
        ulong worldId,
        FlowFieldGrid gridComponent)
    {
        if (!Dependency.IsCompleted)
            return;
        Dependency.Complete();
        if (!TryGetCompletedIncrementalContactSnapshot(
                out IncrementalContactPipelineSnapshot completedPipeline))
            return;

        CompletedSimulationStepMetadata completed = completedPipeline.CompletedStep;
        if (completed.WorldId != worldId ||
            completed.SimulationStepId != completedPipeline.Statistics.Timestep)
            return;

        SimulationDebuggerCaptureMask captureMask = completed.CaptureMask;
        if (captureMask == SimulationDebuggerCaptureMask.None)
            return;

        _simulationDebuggerUpdateCounter++;
        bool spatial = (captureMask & (
            SimulationDebuggerCaptureMask.OverviewHeatmap |
            SimulationDebuggerCaptureMask.AabbHeatmap |
            SimulationDebuggerCaptureMask.ContactSetHeatmap |
            SimulationDebuggerCaptureMask.SelectedUnit |
            SimulationDebuggerCaptureMask.SelectedPairs)) != 0;
        int interval = spatial
            ? math.max(1, SimulationDebuggerRuntime.SpatialSampleIntervalFramesFor(worldId))
            : math.max(1, SimulationDebuggerRuntime.SummarySampleIntervalFramesFor(worldId));
        if ((_simulationDebuggerUpdateCounter - 1) % (ulong)interval != 0)
            return;

        SimulationDebuggerFrameSnapshot snapshot = AcquireSimulationDebuggerWriteSnapshot();
        snapshot.ClearCollections();
        snapshot.WorldId = worldId;
        snapshot.FrameId = ++_simulationDebuggerFrameId;
        snapshot.SimulationStepId = completed.SimulationStepId;
        snapshot.ElapsedTime = completed.ElapsedTime;
        snapshot.DeltaTime = completed.DeltaTime;
        snapshot.SubstepCount = math.max(1, completed.EffectiveSettings.SubstepCount);
        snapshot.IterationCount = math.max(1, completed.EffectiveSettings.IterationCount);
        snapshot.CapturedMask = captureMask;
        snapshot.EffectiveSettings = completed.EffectiveSettings;
        snapshot.Experiment = completed.Experiment;

        PredictiveDiscContactStatistics contactStatistics =
            completedPipeline.SolverStatistics;
        snapshot.Overview = BuildOverviewMetrics(
            completed.UnitCount,
            true,
            contactStatistics,
            completedPipeline.Statistics);
        snapshot.BroadPhase = BuildRetiredBroadPhaseMetrics(
            contactStatistics.ContactPairCount);
        snapshot.ContactSet = BuildContactSetMetrics(
            true,
            contactStatistics,
            completedPipeline.Statistics,
            snapshot.SubstepCount,
            snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0,
            snapshot.EffectiveSettings.EnableDiagnostics != 0);

        SimulationDebuggerSpatialReadback.Capture(
            snapshot,
            gridComponent,
            captureMask,
            EntityManager,
            _incrementalDiagnosticsEntity,
            _candidateStore.SweptProxies);
        CaptureSelectedEntityDetails(
            snapshot,
            completed.SelectedEntity,
            completed.MaximumVisualizedPairs);
        TraceSimulationDebuggerTelemetry(
            worldId,
            completedPipeline,
            snapshot,
            contactStatistics);
        ulong generationBefore =
            PublishedSimulationDiagnosticsRuntime.GetGeneration(worldId);
        bool frozenBeforePublish =
            SimulationDebuggerRuntime.FreezeSnapshotFor(worldId);
        SimulationDebuggerRuntime.Publish(worldId, snapshot, completedPipeline);
        TraceSimulationDebuggerPublication(
            worldId,
            snapshot,
            generationBefore,
            frozenBeforePublish);
    }

    private void TraceSimulationDebuggerTelemetry(
        ulong worldId,
        IncrementalContactPipelineSnapshot pipeline,
        SimulationDebuggerFrameSnapshot snapshot,
        PredictiveDiscContactStatistics raw)
    {
#if RTS_CONTACT_DIAGNOSTICS
        bool diagnosticsEnabled =
            snapshot.EffectiveSettings.EnableDiagnostics != 0;
        bool configurationEnabled =
            pipeline.Configuration.DiagnosticsEnabled != 0;
        bool settingsMismatch =
            diagnosticsEnabled != configurationEnabled;
        bool rawCoreTelemetryZero =
            raw.SolverNanoseconds == 0 &&
            raw.PairGenerationNanoseconds == 0 &&
            raw.IterationNanoseconds == 0 &&
            raw.CandidatePairCount == 0 &&
            raw.ContactPairCount == 0 &&
            raw.TimestepContactSetBuildCount == 0;
        int expectedContactSetSize = raw.TimestepContactSetUniquePairCount;
        int expectedActiveContactCount =
            raw.TimestepContactSetUniqueActivatedPairCount;
        bool mappingMismatch =
            snapshot.Overview.IterationNanoseconds != raw.IterationNanoseconds ||
            snapshot.Overview.PairGenerationNanoseconds !=
                raw.PairGenerationNanoseconds ||
            snapshot.ContactSet.ContactSetSize != expectedContactSetSize ||
            snapshot.ContactSet.ActiveContactCount !=
                expectedActiveContactCount ||
            snapshot.ContactSet.PredictiveContactCount !=
                pipeline.Statistics.CurrentPredictivePairCount ||
            snapshot.ContactSet.FullRebuildCount !=
                raw.TimestepContactSetFullRebuildCount ||
            snapshot.ContactSet.FallbackAddedPairCount !=
                raw.TimestepContactSetFallbackAddedPairCount;
        bool targetWorldMismatch =
            SimulationDebuggerRuntime.TargetWorldId != worldId;
        bool solverWorkMissing =
            raw.IterationNanoseconds == 0 &&
            raw.TimestepContactSetUniquePairCount > 0;
        bool activationMissing =
            raw.TimestepContactSetUniqueActivatedPairCount == 0 &&
            pipeline.Statistics.CurrentActualPairCount > 0;
        bool suspicious =
            settingsMismatch ||
            mappingMismatch ||
            targetWorldMismatch ||
            solverWorkMissing ||
            activationMissing ||
            raw.SolverSkipReason != ContactSolverSkipReason.None ||
            (diagnosticsEnabled && rawCoreTelemetryZero);
        int signature =
            (diagnosticsEnabled ? 1 : 0) |
            (configurationEnabled ? 1 << 1 : 0) |
            (rawCoreTelemetryZero ? 1 << 2 : 0) |
            (mappingMismatch ? 1 << 3 : 0) |
            (targetWorldMismatch ? 1 << 4 : 0) |
            (solverWorkMissing ? 1 << 5 : 0) |
            (activationMissing ? 1 << 6 : 0) |
            ((int)raw.SolverSkipReason << 8);
        uint step = snapshot.SimulationStepId;
        bool stateChanged =
            signature != _simulationDebuggerTraceSignature;
        bool repeatSuspicious =
            suspicious &&
            step - _simulationDebuggerLastTraceStep >= 120;
        if (!stateChanged && !repeatSuspicious)
            return;

        _simulationDebuggerTraceSignature = signature;
        _simulationDebuggerLastTraceStep = step;
        string reason = raw.SolverSkipReason != ContactSolverSkipReason.None
            ? $"SOLVER_SKIPPED_{raw.SolverSkipReason}"
            : solverWorkMissing
                ? "SOLVER_WORK_ZERO"
                : activationMissing
                    ? "ACTIVATION_ZERO"
                    : !diagnosticsEnabled
            ? "DIAGNOSTICS_DISABLED"
            : settingsMismatch
                ? "SETTINGS_MISMATCH"
                : mappingMismatch
                    ? "PUBLISH_MAPPING_MISMATCH"
                    : targetWorldMismatch
                        ? "TARGET_WORLD_MISMATCH"
                        : rawCoreTelemetryZero
                            ? "RAW_SOURCE_ZERO"
                            : "HEALTHY";
        string message =
            $"[CONTACT-DIAG-TRACE] reason={reason} world={worldId} " +
            $"targetWorld={SimulationDebuggerRuntime.TargetWorldId} step={step} " +
            $"settings(effective={snapshot.EffectiveSettings.EnableDiagnostics}," +
            $"pipeline={pipeline.Configuration.DiagnosticsEnabled},dt={snapshot.DeltaTime}," +
            $"solver={snapshot.EffectiveSettings.ContactPositionSolver}," +
            $"substeps={snapshot.SubstepCount},iterations={snapshot.IterationCount}," +
            $"timestepCache={snapshot.EffectiveSettings.EnableTimestepContactSetCache}) " +
            $"raw(solverNs={raw.SolverNanoseconds},pairNs={raw.PairGenerationNanoseconds}," +
            $"iterationNs={raw.IterationNanoseconds},motionNs={raw.MotionNanoseconds}," +
            $"validationNs={raw.ValidationRepairNanoseconds}," +
            $"diagnosticsNs={raw.DiagnosticsNanoseconds}," +
            $"otherNs={snapshot.Overview.OtherStageNanoseconds}," +
            $"broadNs={snapshot.Overview.BroadPhaseNanoseconds}," +
            $"narrowNs={snapshot.Overview.NarrowPhaseNanoseconds}," +
            $"activationNs={snapshot.Overview.ContactActivationNanoseconds}," +
            $"softNs={raw.SoftAvoidanceNanoseconds}," +
            $"overlapNs={snapshot.Overview.OverlappingStageNanoseconds}," +
            $"candidates={raw.CandidatePairCount}," +
            $"contacts={raw.ContactPairCount},predictive={raw.PredictivePairCount}," +
            $"active={raw.ActiveConstraintCount},unique=" +
            $"{raw.TimestepContactSetUniqueActivatedPairCount}/" +
            $"{raw.TimestepContactSetUniquePairCount},rebuild=" +
            $"{raw.TimestepContactSetFullRebuildCount},fallback=" +
            $"{raw.TimestepContactSetFallbackAddedPairCount},substepUses=" +
            $"{raw.TimestepContactSetSubstepUseCount},skip=" +
            $"{raw.SolverSkipReason}/{raw.SolverSkippedSubstepCount}) " +
            $"lifecycle(currentPredictive=" +
            $"{pipeline.Statistics.CurrentPredictivePairCount},currentActual=" +
            $"{pipeline.Statistics.CurrentActualPairCount},currentApproaching=" +
            $"{pipeline.Statistics.CurrentApproachingPairCount},currentDormant=" +
            $"{pipeline.Statistics.CurrentDormantPairCount}) " +
            $"mapped(iterationNs={snapshot.Overview.IterationNanoseconds}," +
            $"set={snapshot.ContactSet.ContactSetSize},active=" +
            $"{snapshot.ContactSet.ActiveContactCount},predictive=" +
            $"{snapshot.ContactSet.PredictiveContactCount},rebuild=" +
            $"{snapshot.ContactSet.FullRebuildCount},fallback=" +
            $"{snapshot.ContactSet.FallbackAddedPairCount})";
        if (suspicious)
            UnityEngine.Debug.LogWarning(message);
        else
            UnityEngine.Debug.Log(message);
#endif
    }

    private void TraceSimulationDebuggerPublication(
        ulong worldId,
        SimulationDebuggerFrameSnapshot submitted,
        ulong generationBefore,
        bool frozenBeforePublish)
    {
#if RTS_CONTACT_DIAGNOSTICS
        ulong generationAfter =
            PublishedSimulationDiagnosticsRuntime.GetGeneration(worldId);
        bool hasLatest =
            PublishedSimulationDiagnosticsRuntime.TryGetLatest(
                worldId,
                out PublishedSimulationDiagnosticsSnapshot published);
        SimulationDebuggerFrameSnapshot latest =
            hasLatest ? published.Frame : null;
        uint latestStep = hasLatest ? published.SimulationStepId : 0;
        bool generationAdvanced = generationAfter > generationBefore;
        bool submittedStepPublished =
            generationAdvanced &&
            latestStep == submitted.SimulationStepId;
        bool publishedValuesMatch =
            submittedStepPublished &&
            latest != null &&
            latest.Overview.IterationNanoseconds ==
                submitted.Overview.IterationNanoseconds &&
            latest.ContactSet.PredictiveContactCount ==
                submitted.ContactSet.PredictiveContactCount &&
            latest.ContactSet.FullRebuildCount ==
                submitted.ContactSet.FullRebuildCount &&
            latest.ContactSet.FallbackAddedPairCount ==
                submitted.ContactSet.FallbackAddedPairCount;

        int result;
        string reason;
        if (frozenBeforePublish)
        {
            result = 1;
            reason = "FROZEN";
        }
        else if (submittedStepPublished && !publishedValuesMatch)
        {
            result = 2;
            reason = "PUBLISHED_FRAME_MISMATCH";
        }
        else if (submittedStepPublished)
        {
            result = 0;
            reason = "PUBLISHED";
        }
        else if (!hasLatest)
        {
            result = 3;
            reason = "NO_LATEST";
        }
        else if (latestStep >= submitted.SimulationStepId)
        {
            result = 4;
            reason = "STALE_OR_DUPLICATE_STEP";
        }
        else
        {
            result = 5;
            reason = "PUBLICATION_REJECTED";
        }

        bool suspicious = result != 0;
        int signature =
            result |
            (SimulationDebuggerRuntime.TargetWorldId == worldId ? 0 : 1 << 4);
        uint step = submitted.SimulationStepId;
        bool stateChanged =
            !_hasSimulationDebuggerPublishTraceSignature ||
            signature != _simulationDebuggerPublishTraceSignature;
        bool repeatSuspicious =
            suspicious &&
            step - _simulationDebuggerLastPublishTraceStep >= 120;
        if (!stateChanged && !repeatSuspicious)
            return;

        _simulationDebuggerPublishTraceSignature = signature;
        _simulationDebuggerLastPublishTraceStep = step;
        _hasSimulationDebuggerPublishTraceSignature = true;
        string message =
            $"[CONTACT-DIAG-PUBLISH] result={reason} world={worldId} " +
            $"targetWorld={SimulationDebuggerRuntime.TargetWorldId} " +
            $"submittedStep={submitted.SimulationStepId} latestStep={latestStep} " +
            $"frozen={(frozenBeforePublish ? 1 : 0)} generation=" +
            $"{generationBefore}->{generationAfter} " +
            $"submitted(iterationNs={submitted.Overview.IterationNanoseconds}," +
            $"predictive={submitted.ContactSet.PredictiveContactCount}," +
            $"rebuild={submitted.ContactSet.FullRebuildCount},fallback=" +
            $"{submitted.ContactSet.FallbackAddedPairCount}) " +
            $"published(iterationNs={latest?.Overview.IterationNanoseconds ?? 0}," +
            $"predictive={latest?.ContactSet.PredictiveContactCount ?? 0}," +
            $"rebuild={latest?.ContactSet.FullRebuildCount ?? 0},fallback=" +
            $"{latest?.ContactSet.FallbackAddedPairCount ?? 0})";
        if (suspicious)
            UnityEngine.Debug.LogWarning(message);
        else
            UnityEngine.Debug.Log(message);
#endif
    }

    private SimulationDebuggerFrameSnapshot AcquireSimulationDebuggerWriteSnapshot()
    {
        _simulationDebuggerSnapshotA ??= new SimulationDebuggerFrameSnapshot();
        _simulationDebuggerSnapshotB ??= new SimulationDebuggerFrameSnapshot();
        _simulationDebuggerWriteA = !_simulationDebuggerWriteA;
        return _simulationDebuggerWriteA
            ? _simulationDebuggerSnapshotA
            : _simulationDebuggerSnapshotB;
    }

    private static SimulationOverviewMetrics BuildOverviewMetrics(
        int unitCount,
        bool hasStatistics,
        PredictiveDiscContactStatistics statistics,
        IncrementalContactPipelineStatistics incremental)
    {
        bool telemetryAvailable = false;
#if RTS_CONTACT_DIAGNOSTICS
        telemetryAvailable = hasStatistics;
#endif
        var result = new SimulationOverviewMetrics
        {
            UnitCount = unitCount,
            TimingAvailable = (byte)(telemetryAvailable ? 1 : 0),
            StageTimingAvailable = (byte)(telemetryAvailable ? 1 : 0),
            WorkloadAvailable = (byte)(telemetryAvailable ? 1 : 0),
            StabilityAvailable = (byte)(telemetryAvailable ? 1 : 0),
            Health = unitCount > 0 && telemetryAvailable
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled
        };
        if (!telemetryAvailable)
            return result;

        result.SolverNanoseconds = statistics.SolverNanoseconds;
        result.SoftAvoidanceNanoseconds = statistics.SoftAvoidanceNanoseconds;
        result.PairGenerationNanoseconds = statistics.PairGenerationNanoseconds;
        result.BroadPhaseNanoseconds =
            incremental.ProxyValidationNanoseconds +
            incremental.FullSweepSourceNanoseconds +
            incremental.PersistentPairMappingNanoseconds +
            incremental.LocalBroadPhaseNanoseconds +
            incremental.PairDiffNanoseconds +
            incremental.FallbackNanoseconds;
        result.NarrowPhaseNanoseconds =
            incremental.SweptClassificationNanoseconds;
        result.ContactActivationNanoseconds =
            incremental.ContactActivationNanoseconds;
        result.IterationNanoseconds = statistics.IterationNanoseconds;
        result.AverageIterationNanoseconds = statistics.AverageIterationNanoseconds;
        result.MotionNanoseconds = statistics.MotionNanoseconds;
        result.ValidationRepairNanoseconds =
            statistics.ValidationRepairNanoseconds;
        result.DiagnosticsNanoseconds = statistics.DiagnosticsNanoseconds;
        result.SolverSkipReason = statistics.SolverSkipReason;
        result.SolverSkippedSubstepCount = statistics.SolverSkippedSubstepCount;
        result.CandidatePairCount = statistics.CandidatePairCount;
        result.ContactPairCount = statistics.ContactPairCount;
        result.CurrentActualPairCount = incremental.CurrentActualPairCount;
        result.CurrentPredictivePairCount = incremental.CurrentPredictivePairCount;
        result.CurrentApproachingPairCount = incremental.CurrentApproachingPairCount;
        result.CurrentDormantPairCount = incremental.CurrentDormantPairCount;
        result.MaxContactCorrection = statistics.MaxContactPositionCorrection;
        result.MaxWallCorrection = statistics.MaxWallPositionCorrection;
        result.MaxVelocityChange = statistics.MaxVelocityChange;

        if (result.SolverSkipReason != ContactSolverSkipReason.None ||
            result.MaxContactCorrection > 0.25f ||
            result.SolverMilliseconds > 4f)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.MaxContactCorrection > 0.08f || result.SolverMilliseconds > 2f)
            result.Health = SimulationDebuggerHealth.Warning;
        return result;
    }

    private static PersistentBroadPhaseMetrics BuildRetiredBroadPhaseMetrics(
        int finalContactPairCount)
    {
        // The old Fat/Adaptive broad-phase panel remains as a disabled compatibility
        // placeholder. IncrementalContactPipelineSnapshot is the only authoritative
        // topology and lifecycle diagnostic source.
        return new PersistentBroadPhaseMetrics
        {
            Enabled = 0,
            Health = SimulationDebuggerHealth.Disabled,
            FinalContactPairCount = finalContactPairCount
        };
    }

    private static TimestepContactSetMetrics BuildContactSetMetrics(
        bool hasStatistics,
        PredictiveDiscContactStatistics statistics,
        IncrementalContactPipelineStatistics incremental,
        int substepCount,
        bool cacheEnabled,
        bool oracleAvailable)
    {
        var result = new TimestepContactSetMetrics
        {
            CacheEnabled = (byte)(cacheEnabled ? 1 : 0),
            MetricsAvailable = (byte)(hasStatistics ? 1 : 0),
            ActivationAvailable = (byte)(hasStatistics && cacheEnabled ? 1 : 0),
            OracleAvailable = (byte)(hasStatistics && oracleAvailable ? 1 : 0),
            ContactGenerationCount = hasStatistics
                ? statistics.TimestepContactSetBuildCount
                : (cacheEnabled ? 1 : substepCount),
            FullRebuildCount = hasStatistics
                ? statistics.TimestepContactSetFullRebuildCount
                : 0,
            FallbackAddedPairCount = hasStatistics
                ? statistics.TimestepContactSetFallbackAddedPairCount
                : 0,
            SubstepCount = substepCount,
            BuildNanoseconds = hasStatistics
                ? statistics.TimestepContactSetBuildNanoseconds
                : 0L,
            ClassificationNanoseconds = hasStatistics
                ? incremental.SweptClassificationNanoseconds
                : 0L,
            ActivationNanoseconds = hasStatistics
                ? incremental.ContactActivationNanoseconds
                : 0L,
            FallbackNanoseconds = hasStatistics
                ? statistics.TimestepContactSetFallbackNanoseconds
                : 0L,
            Health = hasStatistics
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled
        };
        if (!hasStatistics)
            return result;

        // These two values always describe the final committed contact view. With
        // cross-substep caching enabled that view is the timestep-wide unique set;
        // with the cache disabled it is only the latest substep view, so utilization
        // is explicitly marked unavailable rather than changing the denominator.
        result.ContactSetSize = statistics.TimestepContactSetUniquePairCount;
        result.ActiveContactCount =
            statistics.TimestepContactSetUniqueActivatedPairCount;
        result.InactiveContactCount = math.max(
            0,
            result.ContactSetSize - result.ActiveContactCount);
        result.PredictiveContactCount = incremental.CurrentPredictivePairCount;
        result.PredictiveActivatedCount = statistics.PredictiveActivatedCount;
        result.ActualContactCount = incremental.CurrentActualPairCount;
        result.ActivationRatio =
            result.ActivationAvailable != 0 && result.ContactSetSize > 0
            ? math.saturate(result.ActiveContactCount / (float)result.ContactSetSize)
            : 0f;
        result.PredictiveActivationRatio = statistics.PredictivePairCount > 0
            ? statistics.PredictiveActivatedCount / (float)statistics.PredictivePairCount
            : 0f;
        result.AvoidedContactGenerationCount =
            cacheEnabled && result.ContactGenerationCount > 0
            ? math.max(0, substepCount - result.ContactGenerationCount)
            : 0;

        if (result.FallbackAddedPairCount > 0)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.FullRebuildCount > 0)
            result.Health = SimulationDebuggerHealth.Warning;
        return result;
    }

    private void CaptureSelectedEntityDetails(
        SimulationDebuggerFrameSnapshot snapshot,
        Entity selected,
        int maximumVisualizedPairs)
    {

        if (_simulationDebuggerSelectedUnitValid.IsCreated &&
            _simulationDebuggerSelectedUnitValid.Value != 0)
        {
            SimulationDebuggerUnitSample selectedUnit = _simulationDebuggerSelectedUnit.Value;
            if (selected == Entity.Null || selectedUnit.Entity == selected)
            {
                snapshot.SelectedUnit = selectedUnit;
                snapshot.HasSelectedUnit = true;
            }
        }

        if ((snapshot.CapturedMask & SimulationDebuggerCaptureMask.SelectedPairs) != 0)
        {
            int limit = math.max(1, maximumVisualizedPairs);
            int count = math.min(limit, _simulationDebuggerSelectedPairs.Length);
            for (int i = 0; i < count; i++)
                snapshot.SelectedPairs.Add(_simulationDebuggerSelectedPairs[i]);
        }

        if (snapshot.HasSelectedUnit || selected == Entity.Null)
            return;
    }
}
}
