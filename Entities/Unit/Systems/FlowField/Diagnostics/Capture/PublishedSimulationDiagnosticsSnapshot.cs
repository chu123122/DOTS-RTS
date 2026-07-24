using System;

namespace RTS.Unit.FlowField.Diagnostics
{
#if RTS_CONTACT_DIAGNOSTICS
/// <summary>
/// Atomic publication of one completed simulation step. The stored frame is
/// detached from capture builders; callers receive a defensive copy so one
/// consumer cannot mutate another consumer's view.
/// </summary>
public sealed class PublishedSimulationDiagnosticsSnapshot
{
    private readonly SimulationDebuggerFrameSnapshot _frame;

    public ulong WorldId { get; }
    public ulong Generation { get; }
    public uint SimulationStepId { get; }
    public SimulationDebuggerFrameSnapshot Frame => _frame.DeepCopy();
    public IncrementalContactPipelineSnapshot Pipeline { get; }

    internal PublishedSimulationDiagnosticsSnapshot(
        ulong worldId,
        ulong generation,
        uint simulationStepId,
        SimulationDebuggerFrameSnapshot frozenFrame,
        IncrementalContactPipelineSnapshot pipeline)
    {
        WorldId = worldId;
        Generation = generation;
        SimulationStepId = simulationStepId;
        _frame = frozenFrame;
        Pipeline = pipeline;
    }
}

public static class PublishedSimulationDiagnosticsRuntime
{
    private static readonly object Gate = new object();
    private static PublishedSimulationDiagnosticsSnapshot _latest;
    private static ulong _generation;
    private static uint _lastPublishedStepId;

    public static ulong Generation
    {
        get { lock (Gate) return _generation; }
    }

    public static bool PublishComplete(
        ulong worldId,
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline)
    {
        if (worldId == 0 || frame == null || pipeline.Statistics.Timestep == 0)
            return false;

        uint stepId = pipeline.Statistics.Timestep;
        CompletedSimulationStepMetadata metadata = pipeline.CompletedStep;
        if (frame.WorldId != worldId || metadata.WorldId != worldId ||
            frame.SimulationStepId != stepId || metadata.SimulationStepId != stepId)
        {
            throw new InvalidOperationException(
                $"Diagnostics publication identity mismatch: world={worldId}, " +
                $"frameWorld={frame.WorldId}, metadataWorld={metadata.WorldId}, " +
                $"frameStep={frame.SimulationStepId}, metadataStep={metadata.SimulationStepId}, " +
                $"pipelineStep={stepId}.");
        }

        SimulationDebuggerFrameSnapshot frozenFrame = frame.DeepCopy();
        lock (Gate)
        {
            if (stepId <= _lastPublishedStepId)
                return false;
            _lastPublishedStepId = stepId;
            _latest = new PublishedSimulationDiagnosticsSnapshot(
                worldId,
                ++_generation,
                stepId,
                frozenFrame,
                pipeline);
            return true;
        }
    }

    public static bool TryGetLatest(
        ulong worldId,
        out PublishedSimulationDiagnosticsSnapshot snapshot)
    {
        lock (Gate)
        {
            snapshot = _latest;
            return snapshot != null && snapshot.WorldId == worldId;
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

    public static void Reset(ulong worldId)
    {
        lock (Gate)
        {
            if (_latest == null || _latest.WorldId != worldId)
                return;
            _latest = null;
            _generation = 0;
            _lastPublishedStepId = 0;
        }
    }

    public static void Reset()
    {
        lock (Gate)
        {
            _latest = null;
            _generation = 0;
            _lastPublishedStepId = 0;
        }
    }
}
#else
public sealed class PublishedSimulationDiagnosticsSnapshot { }
public static class PublishedSimulationDiagnosticsRuntime
{
    public static ulong Generation => 0;
    public static bool PublishComplete(
        ulong worldId,
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline) => false;
    public static bool TryGetLatest(
        ulong worldId,
        out PublishedSimulationDiagnosticsSnapshot snapshot)
    {
        snapshot = null;
        return false;
    }
    public static bool TryGetLatest(out PublishedSimulationDiagnosticsSnapshot snapshot)
    {
        snapshot = null;
        return false;
    }
    public static void Reset(ulong worldId) { }
    public static void Reset() { }
}
#endif
}
