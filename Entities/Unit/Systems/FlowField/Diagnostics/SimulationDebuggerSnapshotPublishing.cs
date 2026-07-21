using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private SimulationDebuggerFrameSnapshot _simulationDebuggerSnapshotA;
    private SimulationDebuggerFrameSnapshot _simulationDebuggerSnapshotB;
    private bool _simulationDebuggerWriteA;
    private ulong _simulationDebuggerFrameId;

    private void PublishSimulationDebuggerSnapshot(
        FlowFieldGrid grid,
        UnitContactSolverSettings solverSettings)
    {
        SimulationDebuggerCaptureMask captureMask = SimulationDebuggerRuntime.CaptureMask;
        if (captureMask == SimulationDebuggerCaptureMask.None)
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
            hasShadowStatistics,
            shadowStatistics,
            hasContactStatistics ? contactStatistics.ContactPairCount : 0,
            solverSettings.EnableFatAabbCache);
        snapshot.ContactSet = BuildContactSetMetrics(
            hasContactStatistics,
            contactStatistics,
            snapshot.SubstepCount,
            hasShadowStatistics ? shadowStatistics.FullBroadPhaseFallbackCount : 0);

        bool wantsSpatial = (captureMask & (
            SimulationDebuggerCaptureMask.OverviewHeatmap |
            SimulationDebuggerCaptureMask.AabbHeatmap |
            SimulationDebuggerCaptureMask.ContactSetHeatmap)) != 0;
        if (wantsSpatial)
            CopySimulationDebuggerCells(snapshot);

        if ((captureMask & SimulationDebuggerCaptureMask.Regions) != 0)
            CopySimulationDebuggerRegions(snapshot);

        if ((captureMask & SimulationDebuggerCaptureMask.Proxies) != 0 ||
            (captureMask & SimulationDebuggerCaptureMask.SelectedUnit) != 0)
            CopySimulationDebuggerProxies(snapshot);

        CaptureSelectedEntityBounds(snapshot);
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
        int fallbackCount)
    {
        var result = new TimestepContactSetMetrics
        {
            SubstepCount = substepCount,
            SupplementOrFallbackCount = fallbackCount,
            Health = hasStatistics
                ? SimulationDebuggerHealth.Healthy
                : SimulationDebuggerHealth.Disabled
        };
        if (!hasStatistics)
            return result;

        result.ContactSetSize = statistics.ContactPairCount;
        result.ActiveContactCount = statistics.ActiveConstraintCount;
        result.InactiveContactCount = math.max(
            0,
            statistics.ContactPairCount - statistics.ActiveConstraintCount);
        result.PredictiveContactCount = statistics.PredictivePairCount;
        result.PredictiveActivatedCount = statistics.PredictiveActivatedCount;
        result.ActualContactCount = math.max(
            0,
            statistics.ContactPairCount - statistics.PredictivePairCount);
        result.ActivationRatio = statistics.ContactPairCount > 0
            ? statistics.ActiveConstraintCount / (float)statistics.ContactPairCount
            : 0f;
        result.PredictiveActivationRatio = statistics.PredictivePairCount > 0
            ? statistics.PredictiveActivatedCount / (float)statistics.PredictivePairCount
            : 0f;
        result.AvoidedContactGenerationCount = result.ContactSetSize > 0
            ? math.max(0, substepCount - 1)
            : 0;

        if (fallbackCount > 0)
            result.Health = SimulationDebuggerHealth.Critical;
        else if (result.ContactSetSize > 0 && result.ActivationRatio < 0.25f)
            result.Health = SimulationDebuggerHealth.Warning;
        return result;
    }

    private void CopySimulationDebuggerCells(SimulationDebuggerFrameSnapshot snapshot)
    {
        AdaptiveFatAabbCacheFeedback feedback = _adaptiveCacheFeedback.IsCreated
            ? _adaptiveCacheFeedback.Value
            : default;
        for (int i = 0; i < _adaptiveDebugCells.Length; i++)
        {
            AdaptiveFatAabbDebugCell cell = _adaptiveDebugCells[i];
            float contactActivation = math.saturate(
                cell.PressureScore * (1f - cell.EscapeRiskScore * 0.5f));
            snapshot.Cells.Add(new SimulationDebuggerCellSample
            {
                Coordinate = CellCoordinateFromBounds(cell.Min),
                Min = cell.Min,
                Max = cell.Max,
                UnitCount = cell.UnitCount,
                ActiveRegion = cell.Active,
                OverallPressure = math.saturate(math.max(
                    cell.DensityScore,
                    math.max(cell.PressureScore, cell.AverageCorrection))),
                Density = cell.DensityScore,
                SolverCorrection = math.saturate(cell.PressureScore),
                AabbBenefit = math.saturate(
                    0.5f + 0.5f * (feedback.ReuseRatio - cell.CachePenalty)),
                AabbSlack = 0f,
                CandidateExpansion = math.saturate(
                    feedback.CandidateExpansionRatio / math.max(1f, AdaptiveFatAabbSettings.Default.CandidateExpansionLimit)),
                EscapeRisk = math.saturate(math.max(
                    cell.EscapeRiskScore,
                    cell.CachePenalty)),
                ContactActivation = contactActivation,
                ContactWaste = 1f - contactActivation,
                ContactSupplementRisk = math.saturate(
                    cell.EscapeRiskScore + feedback.EscapePenalty)
            });
        }
    }

    private void CopySimulationDebuggerRegions(SimulationDebuggerFrameSnapshot snapshot)
    {
        for (int i = 0; i < _adaptiveDebugRegions.Length; i++)
        {
            AdaptiveFatAabbDebugRegion region = _adaptiveDebugRegions[i];
            snapshot.Regions.Add(new SimulationDebuggerRegionSample
            {
                StableId = region.StableId,
                CoreMin = region.CoreMin,
                CoreMax = region.CoreMax,
                HaloMin = region.HaloMin,
                HaloMax = region.HaloMax,
                UnitCount = region.UnitCount,
                Score = region.Score,
                Active = region.Active
            });
        }
    }

    private void CopySimulationDebuggerProxies(SimulationDebuggerFrameSnapshot snapshot)
    {
        for (int i = 0; i < _adaptiveDebugProxies.Length; i++)
        {
            AdaptiveFatAabbDebugProxy proxy = _adaptiveDebugProxies[i];
            snapshot.Proxies.Add(new SimulationDebuggerProxySample
            {
                Entity = proxy.Entity,
                SweptMin = proxy.CoreMin,
                SweptMax = proxy.CoreMax,
                FatMin = proxy.FatMin,
                FatMax = proxy.FatMax,
                RegionId = proxy.RegionIndex,
                MinimumSlack = proxy.MinimumSlack,
                Escaped = proxy.Escaped
            });
        }
    }

    private void CaptureSelectedEntityBounds(SimulationDebuggerFrameSnapshot snapshot)
    {
        Entity selected = SimulationDebuggerRuntime.SelectedEntity;
        if (selected == Entity.Null)
            return;

        for (int i = 0; i < snapshot.Proxies.Count; i++)
        {
            SimulationDebuggerProxySample proxy = snapshot.Proxies[i];
            if (proxy.Entity != selected)
                continue;

            snapshot.SelectedUnit = new SimulationDebuggerUnitSample
            {
                Entity = selected,
                SweptMin = proxy.SweptMin,
                SweptMax = proxy.SweptMax,
                FatMin = proxy.FatMin,
                FatMax = proxy.FatMax,
                HasFatBounds = 1
            };
            snapshot.HasSelectedUnit = true;
            return;
        }
    }

    private int2 CellCoordinateFromBounds(float2 min)
    {
        float worldCellSize = math.max(
            0.0001f,
            SystemAPI.GetSingleton<FlowFieldGrid>().CellRadius * 2f * math.max(1, _adaptiveCellSpan));
        return (int2)math.floor((min - SystemAPI.GetSingleton<FlowFieldGrid>().GridOrigin.xz) / worldCellSize);
    }
}
}
