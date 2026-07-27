using System;
using System.Collections.Generic;
using Unity.Entities;

namespace RTS.Unit.FlowField.Diagnostics
{
public static class SimulationDebuggerWorldIdentity
{
    public static ulong FromSequenceNumber(ulong sequenceNumber) =>
        unchecked(sequenceNumber + 1UL);
}

#if RTS_CONTACT_DIAGNOSTICS
/// <summary>
/// Managed diagnostics control plane. Authoritative simulation configuration
/// remains in each World's ECS singletons; this registry only routes UI requests,
/// presentation state and completed snapshots to an explicit World id.
/// </summary>
public static class SimulationDebuggerRuntime
{
    private const int HistorySize = 300;
    private const int ComparisonHistorySize = 120;
    private const int ComparisonMinimumSamples = 30;
    public static int CacheComparisonMinimumSamples =>
        ComparisonMinimumSamples;

    private sealed class CacheComparisonBucket
    {
        public readonly SimulationDebuggerHistory BaselinePipeline =
            new SimulationDebuggerHistory(ComparisonHistorySize);
        public readonly SimulationDebuggerHistory EnabledPipeline =
            new SimulationDebuggerHistory(ComparisonHistorySize);
        public readonly SimulationDebuggerHistory BaselinePairs =
            new SimulationDebuggerHistory(ComparisonHistorySize);
        public readonly SimulationDebuggerHistory EnabledPairs =
            new SimulationDebuggerHistory(ComparisonHistorySize);

        public void Push(bool enabled, SimulationDebuggerFrameSnapshot snapshot)
        {
            float milliseconds = snapshot.Overview.SolverNanoseconds / 1_000_000f;
            float pairCount = snapshot.Overview.CandidatePairCount;
            if (enabled)
            {
                EnabledPipeline.PushValue(milliseconds);
                EnabledPairs.PushValue(pairCount);
            }
            else
            {
                BaselinePipeline.PushValue(milliseconds);
                BaselinePairs.PushValue(pairCount);
            }
        }

        public SimulationDebuggerCacheComparison Build(
            bool eligible,
            bool targetEnabled) =>
            new SimulationDebuggerCacheComparison(
                eligible,
                targetEnabled,
                ComparisonMinimumSamples,
                BaselinePipeline.Count,
                EnabledPipeline.Count,
                BaselinePipeline.GetPercentile(0.5f),
                EnabledPipeline.GetPercentile(0.5f),
                BaselinePipeline.GetPercentile(0.95f),
                EnabledPipeline.GetPercentile(0.95f),
                BaselinePairs.GetPercentile(0.5f),
                EnabledPairs.GetPercentile(0.5f));
    }

    private sealed class WorldState
    {
        public SimulationDebuggerEffectiveSettings BaselineSettings;
        public bool HasBaselineSettings;
        public SimulationDebuggerEffectiveSettings PendingSettings;
        public bool HasPendingSettings;
        public bool ResetSettingsRequested;
        public bool ResetContactCachesRequested;
        public int LastSubmittedDiagnostics = -1;
        public uint ExperimentConfigurationId;
        public int ExperimentFramesSinceChanged;
        public int ExperimentLastKey = int.MinValue;
        public SimulationDebuggerCaptureMask CaptureMask = SimulationDebuggerCaptureMask.Summary;
        public SimulationDebuggerCaptureMask LocalRecordingCaptureMask;
        public SimulationDebuggerView ActiveView = SimulationDebuggerView.Overview;
        public SimulationDebuggerHeatmap ActiveHeatmap = SimulationDebuggerHeatmap.None;
        public SimulationDebuggerHeatmap WorldHeatmap = SimulationDebuggerHeatmap.None;
        public SimulationDebuggerView WorldOverlayView = SimulationDebuggerView.Overview;
        public Entity SelectedEntity = Entity.Null;
        public bool OverlayEnabled = true;
        public bool FreezeSnapshot;
        public int MaximumVisualizedPairs = 32;
        public int SummarySampleIntervalFrames = 1;
        public int SpatialSampleIntervalFrames = 2;
        public int ExperimentWarmupFrames = 45;
        public float HeatmapOpacity = 0.28f;
        public float SlowTimeScale = 0.1f;
        public uint HistoryConfigurationId = uint.MaxValue;
        public readonly SimulationDebuggerHistory SolverHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory BroadPhaseHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory NarrowPhaseHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory XpbdHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory SoftAvoidanceHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory CorrectionHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory CacheHitHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory PersistentMaintenanceHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory PersistentCandidateHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory PersistentDirtyRatioHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory PersistentMissingHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory ContactPairHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory ActiveContactHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory ContactSetBuildHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory ContactSetSizeHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly SimulationDebuggerHistory ContactSetActivationHistory = new SimulationDebuggerHistory(HistorySize);
        public readonly Dictionary<int, CacheComparisonBucket> TimestepComparisons =
            new Dictionary<int, CacheComparisonBucket>();
        public readonly Dictionary<int, CacheComparisonBucket> SubstepComparisons =
            new Dictionary<int, CacheComparisonBucket>();
        public int CurrentTimestepComparisonKey = int.MinValue;
        public int CurrentSubstepComparisonKey = int.MinValue;
        public bool CurrentTimestepComparisonEligible;
        public bool CurrentSubstepComparisonEligible;
        public bool CurrentTimestepCacheEnabled;
        public bool CurrentSubstepCacheEnabled;

        public void ClearVisibleHistories()
        {
            SolverHistory.Clear();
            BroadPhaseHistory.Clear();
            NarrowPhaseHistory.Clear();
            XpbdHistory.Clear();
            SoftAvoidanceHistory.Clear();
            CorrectionHistory.Clear();
            CacheHitHistory.Clear();
            PersistentMaintenanceHistory.Clear();
            PersistentCandidateHistory.Clear();
            PersistentDirtyRatioHistory.Clear();
            PersistentMissingHistory.Clear();
            ContactPairHistory.Clear();
            ActiveContactHistory.Clear();
            ContactSetBuildHistory.Clear();
            ContactSetSizeHistory.Clear();
            ContactSetActivationHistory.Clear();
        }
    }

    private static readonly object Gate = new object();
    private static readonly Dictionary<ulong, WorldState> Worlds =
        new Dictionary<ulong, WorldState>();
    private static ulong _targetWorldId;

    private static WorldState GetStateLocked(ulong worldId)
    {
        if (!Worlds.TryGetValue(worldId, out WorldState state))
        {
            state = new WorldState();
            Worlds.Add(worldId, state);
        }
        return state;
    }

    private static ulong ResolveTargetWorldIdLocked()
    {
        if (_targetWorldId != 0 && Worlds.ContainsKey(_targetWorldId))
            return _targetWorldId;
        foreach (ulong worldId in Worlds.Keys)
        {
            if (worldId == 0)
                continue;
            _targetWorldId = worldId;
            return worldId;
        }
        return 0;
    }

    private static WorldState GetTargetStateLocked() =>
        GetStateLocked(ResolveTargetWorldIdLocked());

    public static ulong TargetWorldId
    {
        get { lock (Gate) return ResolveTargetWorldIdLocked(); }
        set
        {
            lock (Gate)
            {
                if (value == 0)
                    return;
                GetStateLocked(value);
                _targetWorldId = value;
            }
        }
    }

    public static void RegisterWorld(ulong worldId)
    {
        if (worldId == 0)
            return;
        lock (Gate)
        {
            GetStateLocked(worldId);
            if (_targetWorldId == 0)
                _targetWorldId = worldId;
        }
    }

    public static void UnregisterWorld(ulong worldId)
    {
        if (worldId == 0)
            return;
        lock (Gate)
        {
            Worlds.Remove(worldId);
            if (_targetWorldId == worldId)
                _targetWorldId = 0;
            ResolveTargetWorldIdLocked();
        }
        PublishedSimulationDiagnosticsRuntime.RemoveWorld(worldId);
    }

    public static ulong[] GetRegisteredWorldIds()
    {
        lock (Gate)
        {
            var result = new List<ulong>(Worlds.Count);
            foreach (ulong worldId in Worlds.Keys)
            {
                if (worldId != 0)
                    result.Add(worldId);
            }
            result.Sort();
            return result.ToArray();
        }
    }

    public static bool SelectNextWorld()
    {
        lock (Gate)
        {
            var ids = new List<ulong>();
            foreach (ulong worldId in Worlds.Keys)
            {
                if (worldId != 0)
                    ids.Add(worldId);
            }
            if (ids.Count <= 1)
                return false;
            ids.Sort();
            int index = ids.IndexOf(ResolveTargetWorldIdLocked());
            _targetWorldId = ids[(index + 1 + ids.Count) % ids.Count];
            return true;
        }
    }

    public static SimulationDebuggerCaptureMask CaptureMask
    {
        get { lock (Gate) { WorldState s=GetTargetStateLocked(); return s.CaptureMask | s.LocalRecordingCaptureMask; } }
        set { lock (Gate) GetTargetStateLocked().CaptureMask = value; }
    }

    public static SimulationDebuggerCaptureMask CaptureMaskFor(ulong worldId)
    {
        lock (Gate) { WorldState s=GetStateLocked(worldId); return s.CaptureMask | s.LocalRecordingCaptureMask; }
    }

    public static void SetLocalRecordingCapture(bool enabled) =>
        SetLocalRecordingCapture(TargetWorldId, enabled);

    public static void SetLocalRecordingCapture(ulong worldId, bool enabled)
    {
        lock (Gate)
        {
            GetStateLocked(worldId).LocalRecordingCaptureMask = enabled
                ? SimulationDebuggerCaptureMask.Summary | SimulationDebuggerCaptureMask.Regions
                : SimulationDebuggerCaptureMask.None;
        }
    }

    public static SimulationDebuggerView ActiveView
    {
        get { lock (Gate) return GetTargetStateLocked().ActiveView; }
        set { lock (Gate) GetTargetStateLocked().ActiveView = value; }
    }
    public static SimulationDebuggerHeatmap ActiveHeatmap
    {
        get { lock (Gate) return GetTargetStateLocked().ActiveHeatmap; }
        set { lock (Gate) GetTargetStateLocked().ActiveHeatmap = value; }
    }
    public static SimulationDebuggerHeatmap WorldHeatmap
    {
        get { lock (Gate) return GetTargetStateLocked().WorldHeatmap; }
        set { lock (Gate) GetTargetStateLocked().WorldHeatmap = value; }
    }
    public static SimulationDebuggerView WorldOverlayView
    {
        get { lock (Gate) return GetTargetStateLocked().WorldOverlayView; }
        set { lock (Gate) GetTargetStateLocked().WorldOverlayView = value; }
    }
    public static Entity SelectedEntity
    {
        get { lock (Gate) return GetTargetStateLocked().SelectedEntity; }
        set { lock (Gate) GetTargetStateLocked().SelectedEntity = value; }
    }
    public static Entity SelectedEntityFor(ulong worldId)
    {
        lock (Gate) return GetStateLocked(worldId).SelectedEntity;
    }
    public static void SetSelectedEntityFor(ulong worldId, Entity entity)
    {
        lock (Gate) GetStateLocked(worldId).SelectedEntity = entity;
    }
    public static bool OverlayEnabled
    {
        get { lock (Gate) return GetTargetStateLocked().OverlayEnabled; }
        set { lock (Gate) GetTargetStateLocked().OverlayEnabled = value; }
    }
    public static bool FreezeSnapshot
    {
        get { lock (Gate) return GetTargetStateLocked().FreezeSnapshot; }
        set { lock (Gate) GetTargetStateLocked().FreezeSnapshot = value; }
    }
    public static bool FreezeSnapshotFor(ulong worldId)
    {
        lock (Gate) return GetStateLocked(worldId).FreezeSnapshot;
    }
    public static int MaximumVisualizedPairs
    {
        get { lock (Gate) return GetTargetStateLocked().MaximumVisualizedPairs; }
        set { lock (Gate) GetTargetStateLocked().MaximumVisualizedPairs = value; }
    }
    public static int MaximumVisualizedPairsFor(ulong worldId)
    {
        lock (Gate) return GetStateLocked(worldId).MaximumVisualizedPairs;
    }
    public static int SummarySampleIntervalFrames
    {
        get { lock (Gate) return GetTargetStateLocked().SummarySampleIntervalFrames; }
        set { lock (Gate) GetTargetStateLocked().SummarySampleIntervalFrames = value; }
    }
    public static int SummarySampleIntervalFramesFor(ulong worldId)
    {
        lock (Gate) return GetStateLocked(worldId).SummarySampleIntervalFrames;
    }
    public static int SpatialSampleIntervalFrames
    {
        get { lock (Gate) return GetTargetStateLocked().SpatialSampleIntervalFrames; }
        set { lock (Gate) GetTargetStateLocked().SpatialSampleIntervalFrames = value; }
    }
    public static int SpatialSampleIntervalFramesFor(ulong worldId)
    {
        lock (Gate) return GetStateLocked(worldId).SpatialSampleIntervalFrames;
    }
    public static int ExperimentWarmupFrames
    {
        get { lock (Gate) return GetTargetStateLocked().ExperimentWarmupFrames; }
        set { lock (Gate) GetTargetStateLocked().ExperimentWarmupFrames = value; }
    }
    public static float HeatmapOpacity
    {
        get { lock (Gate) return GetTargetStateLocked().HeatmapOpacity; }
        set { lock (Gate) GetTargetStateLocked().HeatmapOpacity = value; }
    }
    public static float SlowTimeScale
    {
        get { lock (Gate) return GetTargetStateLocked().SlowTimeScale; }
        set { lock (Gate) GetTargetStateLocked().SlowTimeScale = value; }
    }

    [Obsolete("Use UnitContactSolverSettings.EnableTimestepContactSetCache")]
    public static bool TimestepContactSetCacheEnabled { get => true; set { } }

    public static SimulationExperimentMetrics UpdateExperimentIdentity(
        ulong worldId,
        SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            int key = (settings.EnablePersistentContactCache != 0 ? 1 : 0) |
                      (settings.EnableTimestepContactSetCache != 0 ? 2 : 0) |
                      ((settings.SoftAvoidanceVelocitySolver & 1) << 2) |
                      ((settings.ContactPositionSolver & 1) << 3);
            if (state.ExperimentLastKey != key)
            {
                state.ExperimentLastKey = key;
                state.ExperimentConfigurationId++;
                state.ExperimentFramesSinceChanged = 0;
            }
            else
            {
                state.ExperimentFramesSinceChanged++;
            }
            return new SimulationExperimentMetrics
            {
                PersistentBroadPhaseCache = settings.EnablePersistentContactCache,
                TimestepContactSetCache = settings.EnableTimestepContactSetCache,
                SoftAvoidanceSolver = settings.SoftAvoidanceVelocitySolver,
                ContactPositionSolver = settings.ContactPositionSolver,
                ConfigurationId = state.ExperimentConfigurationId,
                FramesSinceChanged = state.ExperimentFramesSinceChanged,
                IsWarmup = (byte)(state.ExperimentFramesSinceChanged < state.ExperimentWarmupFrames ? 1 : 0)
            };
        }
    }

    public static SimulationExperimentMetrics UpdateExperimentIdentity(
        SimulationDebuggerEffectiveSettings settings) =>
        UpdateExperimentIdentity(TargetWorldId, settings);

    public static ulong PublishedVersion =>
        PublishedSimulationDiagnosticsRuntime.GetGeneration(TargetWorldId);
    public static ulong PublishedVersionFor(ulong worldId) =>
        PublishedSimulationDiagnosticsRuntime.GetGeneration(worldId);

    public static void CaptureBaselineSettings(SimulationDebuggerEffectiveSettings settings) =>
        CaptureBaselineSettings(TargetWorldId, settings);
    public static void CaptureBaselineSettings(
        ulong worldId,
        SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            if (state.HasBaselineSettings)
                return;
            state.BaselineSettings = settings;
            state.HasBaselineSettings = true;
        }
    }

    public static bool TryGetBaselineSettings(out SimulationDebuggerEffectiveSettings settings) =>
        TryGetBaselineSettings(TargetWorldId, out settings);
    public static bool TryGetBaselineSettings(
        ulong worldId,
        out SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            settings = state.BaselineSettings;
            return state.HasBaselineSettings;
        }
    }

    public static void SubmitSettings(SimulationDebuggerEffectiveSettings settings) =>
        SubmitSettings(TargetWorldId, settings);
    public static void SubmitSettings(
        ulong worldId,
        SimulationDebuggerEffectiveSettings settings)
    {
        int previousDiagnostics;
        bool diagnosticsChanged;
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            previousDiagnostics = state.LastSubmittedDiagnostics;
            diagnosticsChanged =
                previousDiagnostics != settings.EnableDiagnostics;
            state.LastSubmittedDiagnostics = settings.EnableDiagnostics;
            state.PendingSettings = settings;
            state.HasPendingSettings = true;
            state.ResetSettingsRequested = false;
        }
        if (diagnosticsChanged)
        {
            UnityEngine.Debug.Log(
                $"[CONTACT-DIAG-SETTINGS] world={worldId} " +
                $"diagnostics={previousDiagnostics}->{settings.EnableDiagnostics} " +
                $"source={ResolveSettingsSubmitter()}");
        }
    }

    private static string ResolveSettingsSubmitter()
    {
        var trace = new System.Diagnostics.StackTrace(1, false);
        for (int frameIndex = 0;
             frameIndex < trace.FrameCount;
             frameIndex++)
        {
            System.Reflection.MethodBase method =
                trace.GetFrame(frameIndex)?.GetMethod();
            Type declaringType = method?.DeclaringType;
            if (declaringType == null ||
                declaringType == typeof(SimulationDebuggerRuntime))
                continue;
            return $"{declaringType.FullName}.{method.Name}";
        }
        return "unknown";
    }

    public static void RequestSettingsReset() => RequestSettingsReset(TargetWorldId);
    public static void RequestSettingsReset(ulong worldId)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            state.ResetSettingsRequested = true;
            state.HasPendingSettings = false;
        }
    }

    public static bool TryConsumeSettingsRequest(
        out SimulationDebuggerEffectiveSettings settings,
        out bool reset) =>
        TryConsumeSettingsRequest(TargetWorldId, out settings, out reset);
    public static bool TryConsumeSettingsRequest(
        ulong worldId,
        out SimulationDebuggerEffectiveSettings settings,
        out bool reset)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            reset = state.ResetSettingsRequested;
            if (reset && state.HasBaselineSettings)
            {
                settings = state.BaselineSettings;
                state.ResetSettingsRequested = false;
                return true;
            }
            if (state.HasPendingSettings)
            {
                settings = state.PendingSettings;
                state.HasPendingSettings = false;
                return true;
            }
            settings = default;
            return false;
        }
    }

    public static void RequestContactCacheReset() =>
        RequestContactCacheReset(TargetWorldId);
    public static void RequestContactCacheReset(ulong worldId)
    {
        lock (Gate) GetStateLocked(worldId).ResetContactCachesRequested = true;
    }
    public static bool TryConsumeContactCacheReset() =>
        TryConsumeContactCacheReset(TargetWorldId);
    public static bool TryConsumeContactCacheReset(ulong worldId)
    {
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            bool requested = state.ResetContactCachesRequested;
            state.ResetContactCachesRequested = false;
            return requested;
        }
    }

    public static void Publish(
        ulong worldId,
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline)
    {
        bool frozen;
        lock (Gate) frozen = GetStateLocked(worldId).FreezeSnapshot;
        if (snapshot == null || frozen ||
            !PublishedSimulationDiagnosticsRuntime.PublishComplete(worldId, snapshot, pipeline))
            return;
        lock (Gate)
        {
            WorldState state = GetStateLocked(worldId);
            if (state.HistoryConfigurationId !=
                snapshot.Experiment.ConfigurationId)
            {
                state.HistoryConfigurationId =
                    snapshot.Experiment.ConfigurationId;
                state.ClearVisibleHistories();
            }
            if (snapshot.Overview.TimingAvailable != 0)
            {
                state.SolverHistory.PushValue(
                    snapshot.Overview.SolverNanoseconds / 1_000_000f);
                state.BroadPhaseHistory.PushValue(
                    snapshot.Overview.BroadPhaseNanoseconds / 1_000_000f);
                state.NarrowPhaseHistory.PushValue(
                    snapshot.Overview.NarrowPhaseNanoseconds / 1_000_000f);
                state.XpbdHistory.PushValue(
                    snapshot.Overview.IterationNanoseconds / 1_000_000f);
                state.SoftAvoidanceHistory.PushValue(
                    snapshot.Overview.SoftAvoidanceNanoseconds / 1_000_000f);
            }
            if (snapshot.Overview.StabilityAvailable != 0)
                state.CorrectionHistory.PushValue(
                    snapshot.Overview.MaxContactCorrection);
            IncrementalContactPipelineStatistics statistics =
                pipeline.Statistics;
            int classified =
                statistics.ReclassifiedPairEvaluationCount +
                statistics.ClassificationReuseCount +
                statistics.ClassificationSkippedCount;
            float reuseRatio = classified > 0
                ? (statistics.ClassificationReuseCount +
                   statistics.ClassificationSkippedCount) / (float)classified
                : 0f;
            state.CacheHitHistory.PushValue(reuseRatio);
            long maintenanceNanoseconds =
                statistics.ProxyValidationNanoseconds +
                statistics.PersistentPairMappingNanoseconds +
                statistics.LocalBroadPhaseNanoseconds +
                statistics.PairDiffNanoseconds +
                statistics.FallbackNanoseconds;
            state.PersistentMaintenanceHistory.PushValue(
                maintenanceNanoseconds / 1_000_000f);
            state.PersistentCandidateHistory.PushValue(
                statistics.PersistentNeighborPairCount);
            state.PersistentDirtyRatioHistory.PushValue(
                pipeline.TopologyDirtyRatio);
            if (snapshot.EffectiveSettings.EnableDiagnostics != 0)
            {
                state.PersistentMissingHistory.PushValue(
                    statistics.OracleMissingPairCount);
            }
            if (snapshot.Overview.WorkloadAvailable != 0)
            {
                state.ContactPairHistory.PushValue(
                    snapshot.Overview.CurrentContactCount);
                state.ActiveContactHistory.PushValue(snapshot.ContactSet.ActiveContactCount);
            }
            if (snapshot.ContactSet.MetricsAvailable != 0)
            {
                state.ContactSetBuildHistory.PushValue(
                    snapshot.ContactSet.BuildNanoseconds / 1_000_000f);
                state.ContactSetSizeHistory.PushValue(
                    snapshot.ContactSet.ContactSetSize);
                if (snapshot.ContactSet.ActivationAvailable != 0)
                {
                    state.ContactSetActivationHistory.PushValue(
                        snapshot.ContactSet.ActivationRatio);
                }
            }
            TrackCacheComparisons(state, snapshot);
        }
    }

    private static void TrackCacheComparisons(
        WorldState state,
        SimulationDebuggerFrameSnapshot snapshot)
    {
        bool telemetryAvailable = snapshot.Overview.TimingAvailable != 0;
        bool persistentEnabled =
            snapshot.EffectiveSettings.EnablePersistentContactCache != 0;
        bool substepEnabled =
            snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0;

        state.CurrentTimestepComparisonEligible =
            telemetryAvailable && substepEnabled;
        state.CurrentTimestepCacheEnabled = persistentEnabled;
        state.CurrentTimestepComparisonKey =
            state.CurrentTimestepComparisonEligible
                ? BuildComparisonKey(snapshot, ignorePersistentCache: true)
                : int.MinValue;

        state.CurrentSubstepComparisonEligible =
            telemetryAvailable && !persistentEnabled;
        state.CurrentSubstepCacheEnabled = substepEnabled;
        state.CurrentSubstepComparisonKey =
            state.CurrentSubstepComparisonEligible
                ? BuildComparisonKey(snapshot, ignoreSubstepCache: true)
                : int.MinValue;

        if (snapshot.Experiment.IsWarmup != 0)
            return;

        if (state.CurrentTimestepComparisonEligible)
        {
            CacheComparisonBucket bucket = GetComparisonBucket(
                state.TimestepComparisons,
                state.CurrentTimestepComparisonKey);
            bucket.Push(persistentEnabled, snapshot);
        }
        if (state.CurrentSubstepComparisonEligible)
        {
            CacheComparisonBucket bucket = GetComparisonBucket(
                state.SubstepComparisons,
                state.CurrentSubstepComparisonKey);
            bucket.Push(substepEnabled, snapshot);
        }
    }

    private static CacheComparisonBucket GetComparisonBucket(
        Dictionary<int, CacheComparisonBucket> buckets,
        int key)
    {
        if (!buckets.TryGetValue(key, out CacheComparisonBucket bucket))
        {
            bucket = new CacheComparisonBucket();
            buckets.Add(key, bucket);
        }
        return bucket;
    }

    private static int BuildComparisonKey(
        SimulationDebuggerFrameSnapshot snapshot,
        bool ignorePersistentCache = false,
        bool ignoreSubstepCache = false)
    {
        SimulationDebuggerEffectiveSettings settings =
            snapshot.EffectiveSettings;
        unchecked
        {
            int hash = 17;
            AddHash(ref hash, snapshot.Overview.UnitCount);
            AddHash(ref hash, snapshot.DeltaTime.GetHashCode());
            AddHash(ref hash, settings.SubstepCount);
            AddHash(ref hash, settings.IterationCount);
            AddHash(ref hash, settings.ContactPositionSolver);
            AddHash(ref hash, settings.Compliance.GetHashCode());
            AddHash(ref hash, settings.PredictiveSkin.GetHashCode());
            AddHash(ref hash, settings.EnablePredictivePairGeneration);
            AddHash(ref hash, settings.EnablePredictiveContacts);
            if (!ignorePersistentCache)
                AddHash(ref hash, settings.EnablePersistentContactCache);
            if (!ignoreSubstepCache)
                AddHash(ref hash, settings.EnableTimestepContactSetCache);
            AddHash(ref hash, settings.PersistentGuardEnvelopeMargin.GetHashCode());
            AddHash(ref hash, settings.TimestepContactMargin.GetHashCode());
            AddHash(ref hash, settings.EnableDiagnostics);
            AddHash(ref hash, settings.SoftAvoidanceResponseRate.GetHashCode());
            AddHash(ref hash, settings.SoftAvoidanceShell.GetHashCode());
            AddHash(ref hash, settings.SettledSoftAvoidanceMultiplier.GetHashCode());
            AddHash(ref hash, settings.SoftAvoidanceVelocitySolver);
            AddHash(ref hash, settings.RvoTimeHorizon.GetHashCode());
            return hash;
        }
    }

    private static void AddHash(ref int hash, int value)
    {
        unchecked
        {
            hash = hash * 31 + value;
        }
    }
    public static void Publish(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline) =>
        Publish(snapshot?.WorldId ?? TargetWorldId, snapshot, pipeline);

    public static SimulationDebuggerTrend GetSolverTrend(int windowFrames = 60)
    { lock (Gate) return GetTargetStateLocked().SolverHistory.GetTrend(windowFrames); }
    public static SimulationDebuggerTrend GetCorrectionTrend(int windowFrames = 60)
    { lock (Gate) return GetTargetStateLocked().CorrectionHistory.GetTrend(windowFrames); }
    public static SimulationDebuggerTrend GetCacheHitTrend(int windowFrames = 60)
    { lock (Gate) return GetTargetStateLocked().CacheHitHistory.GetTrend(windowFrames); }
    public static SimulationDebuggerTrend GetContactPairTrend(int windowFrames = 60)
    { lock (Gate) return GetTargetStateLocked().ContactPairHistory.GetTrend(windowFrames); }
    public static SimulationDebuggerTrend GetActiveContactTrend(int windowFrames = 60)
    { lock (Gate) return GetTargetStateLocked().ActiveContactHistory.GetTrend(windowFrames); }
    public static SimulationDebuggerTrend GetBroadPhaseTrend(int windowSamples = 60)
    { lock (Gate) return GetTargetStateLocked().BroadPhaseHistory.GetTrend(windowSamples); }
    public static SimulationDebuggerTrend GetNarrowPhaseTrend(int windowSamples = 60)
    { lock (Gate) return GetTargetStateLocked().NarrowPhaseHistory.GetTrend(windowSamples); }
    public static SimulationDebuggerTrend GetXpbdTrend(int windowSamples = 60)
    { lock (Gate) return GetTargetStateLocked().XpbdHistory.GetTrend(windowSamples); }
    public static SimulationDebuggerTrend GetPersistentMaintenanceTrend(int windowSamples = 60)
    { lock (Gate) return GetTargetStateLocked().PersistentMaintenanceHistory.GetTrend(windowSamples); }
    public static SimulationDebuggerTrend GetContactSetBuildTrend(int windowSamples = 60)
    { lock (Gate) return GetTargetStateLocked().ContactSetBuildHistory.GetTrend(windowSamples); }
    public static void CopyHistoryTo(SimulationDebuggerHistory target, float[] buffer)
    { if (target != null) target.CopyTo(buffer, buffer.Length); }
    public static SimulationDebuggerHistory GetSolverHistory()
    { lock (Gate) return GetTargetStateLocked().SolverHistory; }
    public static SimulationDebuggerHistory GetCorrectionHistory()
    { lock (Gate) return GetTargetStateLocked().CorrectionHistory; }
    public static SimulationDebuggerHistory GetCacheHitHistory()
    { lock (Gate) return GetTargetStateLocked().CacheHitHistory; }
    public static SimulationDebuggerHistory GetContactPairHistory()
    { lock (Gate) return GetTargetStateLocked().ContactPairHistory; }
    public static SimulationDebuggerHistory GetActiveContactHistoryObj()
    { lock (Gate) return GetTargetStateLocked().ActiveContactHistory; }
    public static SimulationDebuggerHistory GetBroadPhaseHistory()
    { lock (Gate) return GetTargetStateLocked().BroadPhaseHistory; }
    public static SimulationDebuggerHistory GetNarrowPhaseHistory()
    { lock (Gate) return GetTargetStateLocked().NarrowPhaseHistory; }
    public static SimulationDebuggerHistory GetXpbdHistory()
    { lock (Gate) return GetTargetStateLocked().XpbdHistory; }
    public static SimulationDebuggerHistory GetSoftAvoidanceHistory()
    { lock (Gate) return GetTargetStateLocked().SoftAvoidanceHistory; }
    public static SimulationDebuggerHistory GetPersistentMaintenanceHistory()
    { lock (Gate) return GetTargetStateLocked().PersistentMaintenanceHistory; }
    public static SimulationDebuggerHistory GetPersistentCandidateHistory()
    { lock (Gate) return GetTargetStateLocked().PersistentCandidateHistory; }
    public static SimulationDebuggerHistory GetPersistentDirtyRatioHistory()
    { lock (Gate) return GetTargetStateLocked().PersistentDirtyRatioHistory; }
    public static SimulationDebuggerHistory GetPersistentMissingHistory()
    { lock (Gate) return GetTargetStateLocked().PersistentMissingHistory; }
    public static SimulationDebuggerHistory GetContactSetBuildHistory()
    { lock (Gate) return GetTargetStateLocked().ContactSetBuildHistory; }
    public static SimulationDebuggerHistory GetContactSetSizeHistory()
    { lock (Gate) return GetTargetStateLocked().ContactSetSizeHistory; }
    public static SimulationDebuggerHistory GetContactSetActivationHistory()
    { lock (Gate) return GetTargetStateLocked().ContactSetActivationHistory; }

    public static SimulationDebuggerCacheComparison GetTimestepCacheComparison()
    {
        lock (Gate)
        {
            WorldState state = GetTargetStateLocked();
            if (!state.CurrentTimestepComparisonEligible ||
                !state.TimestepComparisons.TryGetValue(
                    state.CurrentTimestepComparisonKey,
                    out CacheComparisonBucket bucket))
            {
                return new SimulationDebuggerCacheComparison(
                    state.CurrentTimestepComparisonEligible,
                    state.CurrentTimestepCacheEnabled,
                    ComparisonMinimumSamples,
                    0,
                    0,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f);
            }
            return bucket.Build(
                true,
                state.CurrentTimestepCacheEnabled);
        }
    }

    public static SimulationDebuggerCacheComparison GetSubstepCacheComparison()
    {
        lock (Gate)
        {
            WorldState state = GetTargetStateLocked();
            if (!state.CurrentSubstepComparisonEligible ||
                !state.SubstepComparisons.TryGetValue(
                    state.CurrentSubstepComparisonKey,
                    out CacheComparisonBucket bucket))
            {
                return new SimulationDebuggerCacheComparison(
                    state.CurrentSubstepComparisonEligible,
                    state.CurrentSubstepCacheEnabled,
                    ComparisonMinimumSamples,
                    0,
                    0,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f,
                    0f);
            }
            return bucket.Build(
                true,
                state.CurrentSubstepCacheEnabled);
        }
    }

    public static void ClearCacheComparisons()
    {
        lock (Gate)
        {
            WorldState state = GetTargetStateLocked();
            state.TimestepComparisons.Clear();
            state.SubstepComparisons.Clear();
        }
    }

    public static bool TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot) =>
        TryGetLatest(TargetWorldId, out snapshot);
    public static bool TryGetLatest(
        ulong worldId,
        out SimulationDebuggerFrameSnapshot snapshot)
    {
        if (PublishedSimulationDiagnosticsRuntime.TryGetLatest(
                worldId,
                out PublishedSimulationDiagnosticsSnapshot unified))
        {
            snapshot = unified.Frame;
            return snapshot != null;
        }
        snapshot = null;
        return false;
    }

    public static void Reset() => ResetWorld(TargetWorldId);
    public static void ResetWorld(ulong worldId)
    {
        lock (Gate) Worlds[worldId] = new WorldState();
        PublishedSimulationDiagnosticsRuntime.Reset(worldId);
    }
}
#else
public static class SimulationDebuggerRuntime
{
    public static int CacheComparisonMinimumSamples => 0;
    public static ulong TargetWorldId { get => 0; set { } }
    public static void RegisterWorld(ulong worldId) { }
    public static void UnregisterWorld(ulong worldId) { }
    public static ulong[] GetRegisteredWorldIds() => Array.Empty<ulong>();
    public static bool SelectNextWorld() => false;
    public static SimulationDebuggerCaptureMask CaptureMask { get => SimulationDebuggerCaptureMask.None; set { } }
    public static SimulationDebuggerCaptureMask CaptureMaskFor(ulong worldId) => SimulationDebuggerCaptureMask.None;
    public static void SetLocalRecordingCapture(bool enabled) { }
    public static void SetLocalRecordingCapture(ulong worldId, bool enabled) { }
    public static SimulationDebuggerView ActiveView { get => SimulationDebuggerView.Overview; set { } }
    public static SimulationDebuggerHeatmap ActiveHeatmap { get => SimulationDebuggerHeatmap.None; set { } }
    public static SimulationDebuggerHeatmap WorldHeatmap { get => SimulationDebuggerHeatmap.None; set { } }
    public static SimulationDebuggerView WorldOverlayView { get => SimulationDebuggerView.Overview; set { } }
    public static Entity SelectedEntity { get => Entity.Null; set { } }
    public static Entity SelectedEntityFor(ulong worldId) => Entity.Null;
    public static void SetSelectedEntityFor(ulong worldId, Entity entity) { }
    public static bool OverlayEnabled { get => false; set { } }
    public static bool FreezeSnapshot { get => false; set { } }
    public static bool FreezeSnapshotFor(ulong worldId) => false;
    public static int MaximumVisualizedPairs { get => 0; set { } }
    public static int MaximumVisualizedPairsFor(ulong worldId) => 0;
    public static int SummarySampleIntervalFrames { get => int.MaxValue; set { } }
    public static int SummarySampleIntervalFramesFor(ulong worldId) => int.MaxValue;
    public static int SpatialSampleIntervalFrames { get => int.MaxValue; set { } }
    public static int SpatialSampleIntervalFramesFor(ulong worldId) => int.MaxValue;
    public static int ExperimentWarmupFrames { get => 0; set { } }
    public static float HeatmapOpacity { get => 0f; set { } }
    public static float SlowTimeScale { get => 1f; set { } }
    [Obsolete("Use UnitContactSolverSettings.EnableTimestepContactSetCache")]
    public static bool TimestepContactSetCacheEnabled { get => true; set { } }
    public static SimulationExperimentMetrics UpdateExperimentIdentity(ulong worldId, SimulationDebuggerEffectiveSettings settings) => default;
    public static SimulationExperimentMetrics UpdateExperimentIdentity(SimulationDebuggerEffectiveSettings settings) => default;
    public static ulong PublishedVersion => 0;
    public static ulong PublishedVersionFor(ulong worldId) => 0;
    public static void CaptureBaselineSettings(SimulationDebuggerEffectiveSettings settings) { }
    public static void CaptureBaselineSettings(ulong worldId, SimulationDebuggerEffectiveSettings settings) { }
    public static bool TryGetBaselineSettings(out SimulationDebuggerEffectiveSettings settings) { settings=default; return false; }
    public static bool TryGetBaselineSettings(ulong worldId, out SimulationDebuggerEffectiveSettings settings) { settings=default; return false; }
    public static void SubmitSettings(SimulationDebuggerEffectiveSettings settings) { }
    public static void SubmitSettings(ulong worldId, SimulationDebuggerEffectiveSettings settings) { }
    public static void RequestSettingsReset() { }
    public static void RequestSettingsReset(ulong worldId) { }
    public static bool TryConsumeSettingsRequest(out SimulationDebuggerEffectiveSettings settings, out bool reset) { settings=default; reset=false; return false; }
    public static bool TryConsumeSettingsRequest(ulong worldId, out SimulationDebuggerEffectiveSettings settings, out bool reset) { settings=default; reset=false; return false; }
    public static void RequestContactCacheReset() { }
    public static void RequestContactCacheReset(ulong worldId) { }
    public static bool TryConsumeContactCacheReset() => false;
    public static bool TryConsumeContactCacheReset(ulong worldId) => false;
    public static void Publish(ulong worldId, SimulationDebuggerFrameSnapshot snapshot, IncrementalContactPipelineSnapshot pipeline) { }
    public static void Publish(SimulationDebuggerFrameSnapshot snapshot, IncrementalContactPipelineSnapshot pipeline) { }
    public static SimulationDebuggerTrend GetSolverTrend(int windowFrames=60) => default;
    public static SimulationDebuggerTrend GetCorrectionTrend(int windowFrames=60) => default;
    public static SimulationDebuggerTrend GetCacheHitTrend(int windowFrames=60) => default;
    public static SimulationDebuggerTrend GetContactPairTrend(int windowFrames=60) => default;
    public static SimulationDebuggerTrend GetActiveContactTrend(int windowFrames=60) => default;
    public static SimulationDebuggerTrend GetBroadPhaseTrend(int windowSamples=60) => default;
    public static SimulationDebuggerTrend GetNarrowPhaseTrend(int windowSamples=60) => default;
    public static SimulationDebuggerTrend GetXpbdTrend(int windowSamples=60) => default;
    public static SimulationDebuggerTrend GetPersistentMaintenanceTrend(int windowSamples=60) => default;
    public static SimulationDebuggerTrend GetContactSetBuildTrend(int windowSamples=60) => default;
    public static void CopyHistoryTo(SimulationDebuggerHistory target, float[] buffer) { }
    public static SimulationDebuggerHistory GetSolverHistory() => null;
    public static SimulationDebuggerHistory GetCorrectionHistory() => null;
    public static SimulationDebuggerHistory GetCacheHitHistory() => null;
    public static SimulationDebuggerHistory GetContactPairHistory() => null;
    public static SimulationDebuggerHistory GetActiveContactHistoryObj() => null;
    public static SimulationDebuggerHistory GetBroadPhaseHistory() => null;
    public static SimulationDebuggerHistory GetNarrowPhaseHistory() => null;
    public static SimulationDebuggerHistory GetXpbdHistory() => null;
    public static SimulationDebuggerHistory GetSoftAvoidanceHistory() => null;
    public static SimulationDebuggerHistory GetPersistentMaintenanceHistory() => null;
    public static SimulationDebuggerHistory GetPersistentCandidateHistory() => null;
    public static SimulationDebuggerHistory GetPersistentDirtyRatioHistory() => null;
    public static SimulationDebuggerHistory GetPersistentMissingHistory() => null;
    public static SimulationDebuggerHistory GetContactSetBuildHistory() => null;
    public static SimulationDebuggerHistory GetContactSetSizeHistory() => null;
    public static SimulationDebuggerHistory GetContactSetActivationHistory() => null;
    public static SimulationDebuggerCacheComparison GetTimestepCacheComparison() => default;
    public static SimulationDebuggerCacheComparison GetSubstepCacheComparison() => default;
    public static void ClearCacheComparisons() { }
    public static bool TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot) { snapshot=null; return false; }
    public static bool TryGetLatest(ulong worldId, out SimulationDebuggerFrameSnapshot snapshot) { snapshot=null; return false; }
    public static void Reset() { }
    public static void ResetWorld(ulong worldId) { }
}
#endif
}
