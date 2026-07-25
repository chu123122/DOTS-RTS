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
            contactStatistics);
        snapshot.BroadPhase = BuildRetiredBroadPhaseMetrics(
            contactStatistics.ContactPairCount);
        snapshot.ContactSet = BuildContactSetMetrics(
            true,
            contactStatistics,
            snapshot.SubstepCount,
            snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0);

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
        SimulationDebuggerRuntime.Publish(worldId, snapshot, completedPipeline);
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
        PredictiveDiscContactStatistics statistics)
    {
        var result = new SimulationOverviewMetrics
        {
            UnitCount = unitCount,
            Health = unitCount > 0
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled
        };
        if (!hasStatistics)
            return result;

        result.SolverNanoseconds = statistics.SolverNanoseconds;
        result.SoftAvoidanceNanoseconds = statistics.SoftAvoidanceNanoseconds;
        result.PairGenerationNanoseconds = statistics.PairGenerationNanoseconds;
        result.IterationNanoseconds = statistics.IterationNanoseconds;
        result.CandidatePairCount = statistics.CandidatePairCount;
        result.ContactPairCount = statistics.ContactPairCount;
        result.MaxContactCorrection = statistics.MaxContactPositionCorrection;
        result.MaxWallCorrection = statistics.MaxWallPositionCorrection;
        result.MaxVelocityChange = statistics.MaxVelocityChange;

        if (result.MaxContactCorrection > 0.25f || result.SolverMilliseconds > 4f)
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
        int substepCount,
        bool cacheEnabled)
    {
        var result = new TimestepContactSetMetrics
        {
            CacheEnabled = (byte)(cacheEnabled ? 1 : 0),
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
            Health = hasStatistics
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled
        };
        if (!hasStatistics)
            return result;

        // 跨子步缓存开启时，同一 Pair 可能在多个 substep 中反复激活；默认面板
        // 应展示“唯一接触拓扑”的覆盖率。关闭缓存时则展示各子步生成工作的总量。
        result.ContactSetSize = cacheEnabled
            ? statistics.TimestepContactSetUniquePairCount
            : statistics.ContactPairCount;
        result.ActiveContactCount = cacheEnabled
            ? statistics.TimestepContactSetUniqueActivatedPairCount
            : statistics.ActiveConstraintCount;
        result.InactiveContactCount = math.max(
            0,
            result.ContactSetSize - result.ActiveContactCount);
        result.PredictiveContactCount = statistics.PredictivePairCount;
        result.PredictiveActivatedCount = statistics.PredictiveActivatedCount;
        result.ActualContactCount = math.max(
            0,
            result.ContactSetSize - statistics.PredictivePairCount);
        result.ActivationRatio = result.ContactSetSize > 0
            ? math.saturate(result.ActiveContactCount / (float)result.ContactSetSize)
            : 0f;
        result.PredictiveActivationRatio = statistics.PredictivePairCount > 0
            ? statistics.PredictiveActivatedCount / (float)statistics.PredictivePairCount
            : 0f;
        result.AvoidedContactGenerationCount = cacheEnabled && result.ContactSetSize > 0
            ? math.max(0, substepCount - 1)
            : 0;

        if (result.FallbackAddedPairCount > 0)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.FullRebuildCount > 0 ||
                 (result.ContactSetSize > 0 && result.ActivationRatio < 0.25f))
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
