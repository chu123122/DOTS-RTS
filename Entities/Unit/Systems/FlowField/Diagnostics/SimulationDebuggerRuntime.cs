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

    public static SimulationDebuggerCaptureMask CaptureMask { get; set; } =
        SimulationDebuggerCaptureMask.Summary;

    public static SimulationDebuggerView ActiveView { get; set; } =
        SimulationDebuggerView.Overview;

    public static SimulationDebuggerHeatmap ActiveHeatmap { get; set; } =
        SimulationDebuggerHeatmap.None;

    public static Entity SelectedEntity { get; set; } = Entity.Null;
    public static bool OverlayEnabled { get; set; } = true;
    public static bool FreezeSnapshot { get; set; }
    public static int MaximumVisualizedPairs { get; set; } = 32;
    public static int SummarySampleIntervalFrames { get; set; } = 1;
    public static int SpatialSampleIntervalFrames { get; set; } = 2;
    public static float HeatmapOpacity { get; set; } = 0.28f;

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

    public static void Publish(SimulationDebuggerFrameSnapshot snapshot)
    {
        if (snapshot == null || FreezeSnapshot)
            return;

        lock (Gate)
        {
            _latest = snapshot;
            _publishedVersion++;
        }
    }

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
        SelectedEntity = Entity.Null;
        OverlayEnabled = true;
        FreezeSnapshot = false;
        MaximumVisualizedPairs = 32;
        SummarySampleIntervalFrames = 1;
        SpatialSampleIntervalFrames = 2;
        HeatmapOpacity = 0.28f;
        lock (Gate)
        {
            _baselineSettings = default;
            _hasBaselineSettings = false;
            _pendingSettings = default;
            _hasPendingSettings = false;
            _resetSettingsRequested = false;
        }
    }
}
}
