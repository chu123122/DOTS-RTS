namespace RTS.Unit.FlowField.Diagnostics
{
#if RTS_CONTACT_DIAGNOSTICS
/// <summary>
/// Unified current-frame diagnostics handoff. It joins the latest completed
/// solver frame and incremental-pipeline snapshot without retaining history.
/// </summary>
public sealed class SimulationDiagnosticsSnapshot
{
    public ulong Generation;
    public SimulationDebuggerFrameSnapshot Frame;
    public IncrementalContactPipelineSnapshot Pipeline;
    public byte HasFrame;
    public byte HasPipeline;

    internal void CopyFrom(SimulationDiagnosticsSnapshot source)
    {
        if (source == null)
        {
            Frame = null;
            Pipeline = default;
            HasFrame = 0;
            HasPipeline = 0;
            return;
        }
        Frame = source.Frame;
        Pipeline = source.Pipeline;
        HasFrame = source.HasFrame;
        HasPipeline = source.HasPipeline;
    }
}

/// <summary>
/// Double-slot publication for the latest completed diagnostics state. This is
/// a current-frame bridge, not a RingBuffer or long-term monitoring store.
/// </summary>
public static class SimulationDiagnosticsSnapshotRuntime
{
    private static readonly object Gate = new object();
    private static readonly SimulationDiagnosticsSnapshot SlotA = new SimulationDiagnosticsSnapshot();
    private static readonly SimulationDiagnosticsSnapshot SlotB = new SimulationDiagnosticsSnapshot();
    private static SimulationDiagnosticsSnapshot _latest;
    private static bool _writeA;
    private static ulong _generation;

    public static ulong Generation
    {
        get { lock (Gate) return _generation; }
    }

    public static void PublishFrame(SimulationDebuggerFrameSnapshot frame)
    {
        if (frame == null) return;
        lock (Gate)
        {
            SimulationDiagnosticsSnapshot slot = AcquireWriteSlot();
            slot.CopyFrom(_latest);
            slot.Frame = frame;
            slot.HasFrame = 1;
            slot.Generation = ++_generation;
            _latest = slot;
        }
    }

    public static void PublishPipeline(IncrementalContactPipelineSnapshot pipeline)
    {
        lock (Gate)
        {
            SimulationDiagnosticsSnapshot slot = AcquireWriteSlot();
            slot.CopyFrom(_latest);
            slot.Pipeline = pipeline;
            slot.HasPipeline = 1;
            slot.Generation = ++_generation;
            _latest = slot;
        }
    }

    public static bool TryGetLatest(out SimulationDiagnosticsSnapshot snapshot)
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
            SlotA.CopyFrom(null);
            SlotB.CopyFrom(null);
            SlotA.Generation = 0;
            SlotB.Generation = 0;
            _latest = null;
            _generation = 0;
            _writeA = false;
        }
    }

    private static SimulationDiagnosticsSnapshot AcquireWriteSlot()
    {
        _writeA = !_writeA;
        return _writeA ? SlotA : SlotB;
    }
}
#else
public sealed class SimulationDiagnosticsSnapshot { }
public static class SimulationDiagnosticsSnapshotRuntime
{
    public static ulong Generation => 0;
    public static void PublishFrame(SimulationDebuggerFrameSnapshot frame) { }
    public static void PublishPipeline(IncrementalContactPipelineSnapshot pipeline) { }
    public static bool TryGetLatest(out SimulationDiagnosticsSnapshot snapshot)
    {
        snapshot = null;
        return false;
    }
    public static void Reset() { }
}
#endif
}
