using Unity.Entities;
using Unity.Mathematics;
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
        FlowFieldGrid grid,
        UnitContactSolverSettings solverSettings)
    {
        SimulationDebuggerCaptureMask captureMask = SimulationDebuggerRuntime.CaptureMask;
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
            ? math.max(1, SimulationDebuggerRuntime.SpatialSampleIntervalFrames)
            : math.max(1, SimulationDebuggerRuntime.SummarySampleIntervalFrames);
        if ((_simulationDebuggerUpdateCounter - 1) % (ulong)interval != 0)
            return;

        // Diagnostics are explicitly opt-in. Completing here keeps the snapshot internally
        // consistent without reintroducing a synchronization point when all debugger views
        // are closed.
        Dependency.Complete();

        SimulationDebuggerFrameSnapshot snapshot = AcquireSimulationDebuggerWriteSnapshot();
        snapshot.ClearCollections();
        snapshot.FrameId = ++_simulationDebuggerFrameId;
        snapshot.ElapsedTime = SystemAPI.Time.ElapsedTime;
        snapshot.DeltaTime = SystemAPI.Time.DeltaTime;
        snapshot.SubstepCount = math.max(1, solverSettings.SubstepCount);
        snapshot.IterationCount = math.max(1, solverSettings.IterationCount);
        snapshot.CapturedMask = captureMask;
        snapshot.EffectiveSettings = BuildEffectiveSettings(
            SystemAPI.GetSingleton<FlowFieldSettings>(),
            solverSettings,
            AdaptiveFatAabbSettings.Default);
        snapshot.Experiment = SimulationDebuggerRuntime.UpdateExperimentIdentity(
            snapshot.EffectiveSettings);

        PredictiveDiscContactStatistics contactStatistics = default;
        ShadowNeighborCacheStatistics shadowStatistics = default;
        bool hasContactStatistics =
            SystemAPI.TryGetSingleton(out contactStatistics);
        bool hasShadowStatistics =
            SystemAPI.TryGetSingleton(out shadowStatistics);

        int unitCount = _movementQuery.CalculateEntityCount();
        snapshot.Overview = BuildOverviewMetrics(
            unitCount,
            hasContactStatistics,
            contactStatistics);
        snapshot.BroadPhase = BuildBroadPhaseMetrics(
            false,
            default,
            hasContactStatistics ? contactStatistics.ContactPairCount : 0,
            false);
        snapshot.ContactSet = BuildContactSetMetrics(
            hasContactStatistics,
            contactStatistics,
            snapshot.SubstepCount,
            hasShadowStatistics ? shadowStatistics.FullBroadPhaseFallbackCount : 0,
            snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0);



        CaptureSelectedEntityDetails(snapshot);
        SimulationDebuggerRuntime.Publish(snapshot);
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

    private static PersistentBroadPhaseMetrics BuildBroadPhaseMetrics(
        bool hasStatistics,
        ShadowNeighborCacheStatistics statistics,
        int finalContactPairCount,
        bool enabled)
    {
        var result = new PersistentBroadPhaseMetrics
        {
            Enabled = (byte)(enabled ? 1 : 0),
            Health = enabled
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled,
            FinalContactPairCount = finalContactPairCount
        };
        if (!enabled || !hasStatistics)
            return result;

        int attempts = statistics.CacheReuseCount + statistics.CacheRebuildCount;
        result.Valid = statistics.CacheValidAtFrameEnd;
        result.CacheAgeFrames = statistics.CacheAgeFrames;
        result.ReuseCount = statistics.CacheReuseCount;
        result.RebuildCount = statistics.CacheRebuildCount;
        result.FallbackCount = statistics.FullBroadPhaseFallbackCount;
        result.InvalidationCount = statistics.CacheInvalidationCount;
        result.CachedCandidatePairCount = statistics.CachedCandidatePairCount;
        result.CacheBuildNanoseconds = statistics.CacheBuildNanoseconds;
        result.CacheValidationNanoseconds = statistics.ValidationNanoseconds;
        result.CachePairMappingNanoseconds = statistics.CachePairMappingNanoseconds;
        result.ReuseRatio = attempts > 0
            ? statistics.CacheReuseCount / (float)attempts
            : 0f;
        result.CandidateExpansion = statistics.CachedCandidatePairCount /
                                    (float)math.max(1, finalContactPairCount);

        float expansionPenalty = math.saturate((result.CandidateExpansion - 2f) / 4f);
        float rebuildPenalty = attempts > 0 ? 1f - result.ReuseRatio : 0f;
        float fallbackPenalty = math.saturate(result.FallbackCount);
        result.EstimatedBenefitScore = math.clamp(
            result.ReuseRatio - expansionPenalty - rebuildPenalty * 0.5f - fallbackPenalty,
            -1f,
            1f);

        if (result.FallbackCount > 0 || result.EstimatedBenefitScore < -0.1f)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.RebuildCount > result.ReuseCount || result.CandidateExpansion > 4f)
            result.Health = SimulationDebuggerHealth.Warning;
        return result;
    }

    private static TimestepContactSetMetrics BuildContactSetMetrics(
        bool hasStatistics,
        PredictiveDiscContactStatistics statistics,
        int substepCount,
        int fallbackCount,
        bool cacheEnabled)
    {
        var result = new TimestepContactSetMetrics
        {
            CacheEnabled = (byte)(cacheEnabled ? 1 : 0),
            ContactGenerationCount = hasStatistics
                ? statistics.TimestepContactSetBuildCount
                : (cacheEnabled ? 1 : substepCount) + math.max(0, fallbackCount),
            SubstepCount = substepCount,
            SupplementOrFallbackCount = fallbackCount,
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

        if (fallbackCount > 0)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.ContactSetSize > 0 && result.ActivationRatio < 0.25f)
            result.Health = SimulationDebuggerHealth.Warning;
        return result;
    }

    private void CaptureSelectedEntityDetails(SimulationDebuggerFrameSnapshot snapshot)
    {
        Entity selected = SimulationDebuggerRuntime.SelectedEntity;
        if (SystemAPI.TryGetSingleton(out Stage3ContactDiagnosticSelection selection) &&
            selection.SelectedEntity != Entity.Null)
        {
            selected = selection.SelectedEntity;
            SimulationDebuggerRuntime.SelectedEntity = selected;
        }

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
            int limit = math.max(1, SimulationDebuggerRuntime.MaximumVisualizedPairs);
            int count = math.min(limit, _simulationDebuggerSelectedPairs.Length);
            for (int i = 0; i < count; i++)
                snapshot.SelectedPairs.Add(_simulationDebuggerSelectedPairs[i]);
        }

        if (snapshot.HasSelectedUnit || selected == Entity.Null)
            return;

    }
}
}
