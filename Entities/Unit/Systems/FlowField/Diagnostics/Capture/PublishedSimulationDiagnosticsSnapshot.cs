namespace RTS.Unit.FlowField.Diagnostics
{
#if RTS_CONTACT_DIAGNOSTICS
/// <summary>
/// Immutable publication of one completed simulation step. It is a current-state
/// handoff only; no historical RingBuffer is retained.
/// </summary>
public sealed class PublishedSimulationDiagnosticsSnapshot
{
    public ulong Generation { get; }
    public uint SimulationStepId { get; }
    public SimulationDebuggerFrameSnapshot Frame { get; }
    public IncrementalContactPipelineSnapshot Pipeline { get; }

    internal PublishedSimulationDiagnosticsSnapshot(
        ulong generation,
        uint simulationStepId,
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline)
    {
        Generation = generation;
        SimulationStepId = simulationStepId;
        Frame = frame;
        Pipeline = pipeline;
    }
}

public static class PublishedSimulationDiagnosticsRuntime
{
    private static readonly object Gate = new object();
    private static PublishedSimulationDiagnosticsSnapshot _latest;
    private static ulong _generation;

    public static ulong Generation
    {
        get { lock (Gate) return _generation; }
    }

    public static void PublishComplete(
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline)
    {
        if (frame == null || pipeline.Statistics.Timestep == 0)
            return;

        SimulationDebuggerFrameSnapshot frozenFrame = frame.DeepCopy();
        uint stepId = pipeline.Statistics.Timestep;
        frozenFrame.SimulationStepId = stepId;

        lock (Gate)
        {
            _latest = new PublishedSimulationDiagnosticsSnapshot(
                ++_generation,
                stepId,
                frozenFrame,
                pipeline);
        }
    }

    public static bool TryGetLatest(out PublishedSimulationDiagnosticsSnapshot snapshot)
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
            _generation = 0;
        }
    }
}
#else
public sealed class PublishedSimulationDiagnosticsSnapshot { }
public static class PublishedSimulationDiagnosticsRuntime
{
    public static ulong Generation => 0;
    public static void PublishComplete(
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline) { }
    public static bool TryGetLatest(out PublishedSimulationDiagnosticsSnapshot snapshot)
    {
        snapshot = null;
        return false;
    }
    public static void Reset() { }
}
#endif
}
