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

    public static ulong PublishedVersion
    {
        get
        {
            lock (Gate)
                return _publishedVersion;
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
    }
}
}
