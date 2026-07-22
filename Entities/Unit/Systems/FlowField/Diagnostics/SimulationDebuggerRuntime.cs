using Unity.Entities;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Managed presentation bridge. Only main-thread systems and MonoBehaviours may access it.
/// Solver jobs never write managed state directly.
/// </summary>
public static class SimulationDebuggerRuntime
{
    private static readonly object Gate = new object();
    private static SimulationDebuggerFrameSnapshot _latest;
    private static ulong _publishedVersion;
    private static SimulationDebuggerEffectiveSettings _baselineSettings;
    private static bool _hasBaselineSettings;
    private static SimulationDebuggerEffectiveSettings _pendingSettings;
    private static bool _hasPendingSettings;
    private static bool _resetSettingsRequested;
    private static byte _timestepContactSetCacheEnabled = 1;
    private static uint _experimentConfigurationId;
    private static int _experimentFramesSinceChanged;
    private static int _experimentLastKey = int.MinValue;

    public static SimulationDebuggerCaptureMask CaptureMask { get; set; } =
        SimulationDebuggerCaptureMask.Summary;

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


    public static bool TimestepContactSetCacheEnabled
    {
        get
        {
            lock (Gate)
                return _timestepContactSetCacheEnabled != 0;
        }
        set
        {
            lock (Gate)
                _timestepContactSetCacheEnabled = (byte)(value ? 1 : 0);
        }
    }

    public static SimulationExperimentMetrics UpdateExperimentIdentity(
        SimulationDebuggerEffectiveSettings settings)
    {
        lock (Gate)
        {
            int key = (settings.EnableFatAabbCache != 0 ? 1 : 0) |
                      (settings.EnableTimestepContactSetCache != 0 ? 2 : 0) |
                      ((settings.SoftAvoidanceVelocitySolver & 1) << 2);
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
                ConfigurationId = _experimentConfigurationId,
                FramesSinceChanged = _experimentFramesSinceChanged,
                IsWarmup = (byte)(_experimentFramesSinceChanged < ExperimentWarmupFrames ? 1 : 0)
            };
        }
    }

    public static ulong PublishedVersion
    {
        get
        {
            lock (Gate)
                return _publishedVersion;
        }
    }


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

    private const int HistorySize = 300;
    private static readonly SimulationDebuggerHistory _solverHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _correctionHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _cacheHitHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _contactPairHistory = new(HistorySize);
    private static readonly SimulationDebuggerHistory _activeContactHistory = new(HistorySize);

    public static void Publish(SimulationDebuggerFrameSnapshot snapshot)
    {
        if (snapshot == null || FreezeSnapshot)
            return;

        lock (Gate)
        {
            _latest = snapshot;
            _publishedVersion++;
        }
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
        lock (Gate)
        {
            snapshot = _latest;
            return snapshot != null;
        }
    }

    public static void Reset()
    {
        lock (Gate)
        {
            _latest = null;
            _publishedVersion = 0;
        }
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
            _timestepContactSetCacheEnabled = 1;
            _experimentConfigurationId = 0;
            _experimentFramesSinceChanged = 0;
            _experimentLastKey = int.MinValue;
        }
    }
}
}
