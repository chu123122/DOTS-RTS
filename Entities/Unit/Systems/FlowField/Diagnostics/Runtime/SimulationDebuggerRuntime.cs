using Unity.Entities;

namespace RTS.Unit.FlowField.Diagnostics
{
#if RTS_CONTACT_DIAGNOSTICS
/// <summary>
/// Managed presentation bridge. Only main-thread systems and MonoBehaviours may access it.
/// Solver jobs never write managed state directly.
/// </summary>
public static class SimulationDebuggerRuntime
{
    private static readonly object Gate = new object();
    private static SimulationDebuggerEffectiveSettings _baselineSettings;
    private static bool _hasBaselineSettings;
    private static SimulationDebuggerEffectiveSettings _pendingSettings;
    private static bool _hasPendingSettings;
    private static bool _resetSettingsRequested;
    private static bool _resetContactCachesRequested;
    private static uint _experimentConfigurationId;
    private static int _experimentFramesSinceChanged;
    private static int _experimentLastKey = int.MinValue;
    private static SimulationDebuggerCaptureMask _captureMask =
        SimulationDebuggerCaptureMask.Summary;
    private static SimulationDebuggerCaptureMask _localRecordingCaptureMask;

    public static SimulationDebuggerCaptureMask CaptureMask
    {
        get
        {
            lock (Gate)
                return _captureMask | _localRecordingCaptureMask;
        }
        set
        {
            lock (Gate)
                _captureMask = value;
        }
    }

    /// <summary>
    /// 本地记录期间保留最小快照，避免关闭面板或切换窗口时丢失采样数据。
    /// 该租约不会覆盖面板自身的 CaptureMask。
    /// </summary>
    public static void SetLocalRecordingCapture(bool enabled)
    {
        lock (Gate)
        {
            _localRecordingCaptureMask = enabled
                ? SimulationDebuggerCaptureMask.Summary | SimulationDebuggerCaptureMask.Regions
                : SimulationDebuggerCaptureMask.None;
        }
    }

    public static SimulationDebuggerView ActiveView { get; set; } =
        SimulationDebuggerView.Overview;

    // ActiveHeatmap/ActiveView are retained for compatibility with older callers.
    public static SimulationDebuggerHeatmap ActiveHeatmap { get; set; } =
        SimulationDebuggerHeatmap.None;

    public static SimulationDebuggerHeatmap WorldHeatmap { get; set; } =
        SimulationDebuggerHeatmap.None;

    public static SimulationDebuggerView WorldOverlayView { get; set; } =
        SimulationDebuggerView.Overview;

    public static Entity SelectedEntity { get; set; } = Entity.Null;
    public static bool OverlayEnabled { get; set; } = true;
    public static bool FreezeSnapshot { get; set; }
    public static int MaximumVisualizedPairs { get; set; } = 32;
    public static int SummarySampleIntervalFrames { get; set; } = 1;
    public static int SpatialSampleIntervalFrames { get; set; } = 2;
    public static int ExperimentWarmupFrames { get; set; } = 45;
    public static float HeatmapOpacity { get; set; } = 0.28f;
    public static float SlowTimeScale { get; set; } = 0.1f;

    [System.Obsolete("Use UnitContactSolverSettings.EnableTimestepContactSetCache")]
    public static bool TimestepContactSetCacheEnabled { get; set; } = true;

    public static SimulationExperimentMetrics UpdateExperimentIdentity(
        SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            int key = (settings.EnableFatAabbCache != 0 ? 1 : 0) |
                      (settings.EnableTimestepContactSetCache != 0 ? 2 : 0) |
                      ((settings.SoftAvoidanceVelocitySolver & 1) << 2) |
                      ((settings.ContactPositionSolver & 1) << 3);
            if (_experimentLastKey != key)
            {
                _experimentLastKey = key;
                _experimentConfigurationId++;
                _experimentFramesSinceChanged = 0;
            }
            else
            {
                _experimentFramesSinceChanged++;
            }

            return new SimulationExperimentMetrics
            {
                PersistentBroadPhaseCache = settings.EnableFatAabbCache,
                TimestepContactSetCache = settings.EnableTimestepContactSetCache,
                SoftAvoidanceSolver = settings.SoftAvoidanceVelocitySolver,
                ContactPositionSolver = settings.ContactPositionSolver,
                ConfigurationId = _experimentConfigurationId,
                FramesSinceChanged = _experimentFramesSinceChanged,
                IsWarmup = (byte)(_experimentFramesSinceChanged < ExperimentWarmupFrames ? 1 : 0)
            };
        }
    }

    public static ulong PublishedVersion => SimulationDiagnosticsSnapshotRuntime.Generation;

    public static void CaptureBaselineSettings(SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            if (_hasBaselineSettings)
                return;
            _baselineSettings = settings;
            _hasBaselineSettings = true;
        }
    }

    public static bool TryGetBaselineSettings(out SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            settings = _baselineSettings;
            return _hasBaselineSettings;
        }
    }

    public static void SubmitSettings(SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            _pendingSettings = settings;
            _hasPendingSettings = true;
            _resetSettingsRequested = false;
        }
    }

    public static void RequestSettingsReset()
    {
        lock (Gate)
        {
            _resetSettingsRequested = true;
            _hasPendingSettings = false;
        }
    }

    public static bool TryConsumeSettingsRequest(
        out SimulationDebuggerEffectiveSettings settings,
        out bool reset)
    {
        lock (Gate)
        {
            reset = _resetSettingsRequested;
            if (reset && _hasBaselineSettings)
            {
                settings = _baselineSettings;
                _resetSettingsRequested = false;
                return true;
            }

            if (_hasPendingSettings)
            {
                settings = _pendingSettings;
                _hasPendingSettings = false;
                return true;
            }

            settings = default;
            return false;
        }
    }

    /// <summary>
    /// 仅用于基准测试：在下一次移动系统更新前清空跨帧接触缓存，
    /// 保证每个 A/B trial 都从同一空缓存开始。
    /// </summary>
    public static void RequestContactCacheReset()
    {
        lock (Gate)
            _resetContactCachesRequested = true;
    }

    public static bool TryConsumeContactCacheReset()
    {
        lock (Gate)
        {
            bool requested = _resetContactCachesRequested;
            _resetContactCachesRequested = false;
            return requested;
        }
    }

    private const int HistorySize = 300;
    private static readonly SimulationDebuggerHistory _solverHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _correctionHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _cacheHitHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _contactPairHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _activeContactHistory = new(HistorySize);

    public static void Publish(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline)
    {
        if (snapshot == null || FreezeSnapshot)
            return;

        SimulationDiagnosticsSnapshotRuntime.PublishComplete(snapshot, pipeline);
        _solverHistory.PushValue(snapshot.Overview.SolverNanoseconds / 1_000_000f);
        _correctionHistory.PushValue(snapshot.Overview.MaxContactCorrection);
        _cacheHitHistory.PushValue(snapshot.BroadPhase.ReuseRatio);
        _contactPairHistory.PushValue(snapshot.ContactSet.ContactSetSize);
        _activeContactHistory.PushValue(snapshot.ContactSet.ActiveContactCount);
    }

    public static SimulationDebuggerTrend GetSolverTrend(int windowFrames = 60)
        => _solverHistory.GetTrend(windowFrames);

    public static SimulationDebuggerTrend GetCorrectionTrend(int windowFrames = 60)
        => _correctionHistory.GetTrend(windowFrames);

    public static SimulationDebuggerTrend GetCacheHitTrend(int windowFrames = 60)
        => _cacheHitHistory.GetTrend(windowFrames);

    public static SimulationDebuggerTrend GetContactPairTrend(int windowFrames = 60)
        => _contactPairHistory.GetTrend(windowFrames);

    public static SimulationDebuggerTrend GetActiveContactTrend(int windowFrames = 60)
        => _activeContactHistory.GetTrend(windowFrames);

    public static void CopyHistoryTo(SimulationDebuggerHistory target, float[] buffer)
    {
        if (target != null)
            target.CopyTo(buffer, buffer.Length);
    }

    public static SimulationDebuggerHistory GetSolverHistory() => _solverHistory;
    public static SimulationDebuggerHistory GetCorrectionHistory() => _correctionHistory;
    public static SimulationDebuggerHistory GetCacheHitHistory() => _cacheHitHistory;
    public static SimulationDebuggerHistory GetContactPairHistory() => _contactPairHistory;
    public static SimulationDebuggerHistory GetActiveContactHistoryObj() => _activeContactHistory;

    public static bool TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot)
    {
        if (SimulationDiagnosticsSnapshotRuntime.TryGetLatest(out SimulationDiagnosticsSnapshot unified))
        {
            snapshot = unified.Frame;
            return snapshot != null;
        }
        snapshot = null;
        return false;
    }

    public static void Reset()
    {
        lock (Gate)
        {
            _localRecordingCaptureMask = SimulationDebuggerCaptureMask.None;
        }
        SimulationDiagnosticsSnapshotRuntime.Reset();
        CaptureMask = SimulationDebuggerCaptureMask.Summary;
        ActiveView = SimulationDebuggerView.Overview;
        ActiveHeatmap = SimulationDebuggerHeatmap.None;
        WorldHeatmap = SimulationDebuggerHeatmap.None;
        WorldOverlayView = SimulationDebuggerView.Overview;
        SelectedEntity = Entity.Null;
        OverlayEnabled = true;
        FreezeSnapshot = false;
        MaximumVisualizedPairs = 32;
        SummarySampleIntervalFrames = 1;
        SpatialSampleIntervalFrames = 2;
        ExperimentWarmupFrames = 45;
        HeatmapOpacity = 0.28f;
        lock (Gate)
        {
            _baselineSettings = default;
            _hasBaselineSettings = false;
            _pendingSettings = default;
            _hasPendingSettings = false;
            _resetSettingsRequested = false;
            _resetContactCachesRequested = false;
            TimestepContactSetCacheEnabled = true;
            _experimentConfigurationId = 0;
            _experimentFramesSinceChanged = 0;
            _experimentLastKey = int.MinValue;
        }
    }
}
#else
/// <summary>
/// Gameplay-only facade. The API remains source-compatible so runtime systems do
/// not need a second call graph, but no managed snapshot, history, selection, or
/// experiment state is retained when diagnostics are not compiled.
/// </summary>
public static class SimulationDebuggerRuntime
{
    public static SimulationDebuggerCaptureMask CaptureMask
    {
        get => SimulationDebuggerCaptureMask.None;
        set { }
    }

    public static void SetLocalRecordingCapture(bool enabled) { }

    public static SimulationDebuggerView ActiveView
    {
        get => SimulationDebuggerView.Overview;
        set { }
    }

    public static SimulationDebuggerHeatmap ActiveHeatmap
    {
        get => SimulationDebuggerHeatmap.None;
        set { }
    }

    public static SimulationDebuggerHeatmap WorldHeatmap
    {
        get => SimulationDebuggerHeatmap.None;
        set { }
    }

    public static SimulationDebuggerView WorldOverlayView
    {
        get => SimulationDebuggerView.Overview;
        set { }
    }

    public static Entity SelectedEntity
    {
        get => Entity.Null;
        set { }
    }

    public static bool OverlayEnabled { get => false; set { } }
    public static bool FreezeSnapshot { get => false; set { } }
    public static int MaximumVisualizedPairs { get => 0; set { } }
    public static int SummarySampleIntervalFrames { get => int.MaxValue; set { } }
    public static int SpatialSampleIntervalFrames { get => int.MaxValue; set { } }
    public static int ExperimentWarmupFrames { get => 0; set { } }
    public static float HeatmapOpacity { get => 0f; set { } }
    public static float SlowTimeScale { get => 1f; set { } }

    // Timestep contact-set reuse is gameplay behavior today. Keep the existing
    // default enabled until its authority is moved out of the debugger facade.
    [System.Obsolete("Use UnitContactSolverSettings.EnableTimestepContactSetCache")]
    public static bool TimestepContactSetCacheEnabled { get; set; } = true;

    public static SimulationExperimentMetrics UpdateExperimentIdentity(
        SimulationDebuggerEffectiveSettings settings) => default;

    public static ulong PublishedVersion => 0;
    public static void CaptureBaselineSettings(SimulationDebuggerEffectiveSettings settings) { }

    public static bool TryGetBaselineSettings(out SimulationDebuggerEffectiveSettings settings)
    {
        settings = default;
        return false;
    }

    public static void SubmitSettings(SimulationDebuggerEffectiveSettings settings) { }
    public static void RequestSettingsReset() { }

    public static bool TryConsumeSettingsRequest(
        out SimulationDebuggerEffectiveSettings settings,
        out bool reset)
    {
        settings = default;
        reset = false;
        return false;
    }

    public static void RequestContactCacheReset() { }
    public static bool TryConsumeContactCacheReset() => false;
    public static void Publish(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline) { }
    public static SimulationDebuggerTrend GetSolverTrend(int windowFrames = 60) => default;
    public static SimulationDebuggerTrend GetCorrectionTrend(int windowFrames = 60) => default;
    public static SimulationDebuggerTrend GetCacheHitTrend(int windowFrames = 60) => default;
    public static SimulationDebuggerTrend GetContactPairTrend(int windowFrames = 60) => default;
    public static SimulationDebuggerTrend GetActiveContactTrend(int windowFrames = 60) => default;
    public static void CopyHistoryTo(SimulationDebuggerHistory target, float[] buffer) { }
    public static SimulationDebuggerHistory GetSolverHistory() => null;
    public static SimulationDebuggerHistory GetCorrectionHistory() => null;
    public static SimulationDebuggerHistory GetCacheHitHistory() => null;
    public static SimulationDebuggerHistory GetContactPairHistory() => null;
    public static SimulationDebuggerHistory GetActiveContactHistoryObj() => null;

    public static bool TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot)
    {
        snapshot = null;
        return false;
    }

    public static void Reset() { }
}
#endif
}
