using System;
using System.Collections.Generic;

namespace RTS.Unit.FlowField.Diagnostics
{
#if RTS_CONTACT_DIAGNOSTICS
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
        WorldId=worldId;
        Generation=generation;
        SimulationStepId=simulationStepId;
        _frame=frozenFrame;
        Pipeline=pipeline;
    }
}

public static class PublishedSimulationDiagnosticsRuntime
{
    private sealed class WorldPublicationState
    {
        public PublishedSimulationDiagnosticsSnapshot Latest;
        public ulong Generation;
        public uint LastPublishedStepId;
    }

    private static readonly object Gate = new object();
    private static readonly Dictionary<ulong,WorldPublicationState> Worlds =
        new Dictionary<ulong,WorldPublicationState>();

    private static WorldPublicationState GetStateLocked(ulong worldId)
    {
        if (!Worlds.TryGetValue(worldId,out WorldPublicationState state))
        {
            state=new WorldPublicationState();
            Worlds.Add(worldId,state);
        }
        return state;
    }

    public static ulong Generation => GetGeneration(SimulationDebuggerRuntime.TargetWorldId);
    public static ulong GetGeneration(ulong worldId)
    {
        lock(Gate) return worldId!=0 && Worlds.TryGetValue(worldId,out WorldPublicationState state)
            ? state.Generation : 0;
    }

    public static bool PublishComplete(
        ulong worldId,
        SimulationDebuggerFrameSnapshot frame,
        IncrementalContactPipelineSnapshot pipeline)
    {
        if(worldId==0 || frame==null || pipeline.Statistics.Timestep==0)
            return false;
        uint stepId=pipeline.Statistics.Timestep;
        CompletedSimulationStepMetadata metadata=pipeline.CompletedStep;
        if(frame.WorldId!=worldId || metadata.WorldId!=worldId ||
           frame.SimulationStepId!=stepId || metadata.SimulationStepId!=stepId)
        {
            throw new InvalidOperationException(
                $"Diagnostics publication identity mismatch: world={worldId}, "+
                $"frameWorld={frame.WorldId}, metadataWorld={metadata.WorldId}, "+
                $"frameStep={frame.SimulationStepId}, metadataStep={metadata.SimulationStepId}, "+
                $"pipelineStep={stepId}.");
        }
        SimulationDebuggerFrameSnapshot frozen=frame.DeepCopy();
        lock(Gate)
        {
            WorldPublicationState state=GetStateLocked(worldId);
            if(stepId<=state.LastPublishedStepId)
                return false;
            state.LastPublishedStepId=stepId;
            state.Latest=new PublishedSimulationDiagnosticsSnapshot(
                worldId,++state.Generation,stepId,frozen,pipeline);
            return true;
        }
    }

    public static bool TryGetLatest(ulong worldId,out PublishedSimulationDiagnosticsSnapshot snapshot)
    {
        lock(Gate)
        {
            if(worldId!=0 && Worlds.TryGetValue(worldId,out WorldPublicationState state))
            {
                snapshot=state.Latest;
                return snapshot!=null;
            }
            snapshot=null;
            return false;
        }
    }
    public static bool TryGetLatest(out PublishedSimulationDiagnosticsSnapshot snapshot) =>
        TryGetLatest(SimulationDebuggerRuntime.TargetWorldId,out snapshot);

    public static void Reset(ulong worldId)
    {
        lock(Gate)
        {
            if(worldId!=0)
                Worlds[worldId]=new WorldPublicationState();
        }
    }
    public static void Reset() => Reset(SimulationDebuggerRuntime.TargetWorldId);
    public static void RemoveWorld(ulong worldId)
    {
        lock(Gate) Worlds.Remove(worldId);
    }
}
#else
public sealed class PublishedSimulationDiagnosticsSnapshot { }
public static class PublishedSimulationDiagnosticsRuntime
{
    public static ulong Generation=>0;
    public static ulong GetGeneration(ulong worldId)=>0;
    public static bool PublishComplete(ulong worldId,SimulationDebuggerFrameSnapshot frame,IncrementalContactPipelineSnapshot pipeline)=>false;
    public static bool TryGetLatest(ulong worldId,out PublishedSimulationDiagnosticsSnapshot snapshot){snapshot=null;return false;}
    public static bool TryGetLatest(out PublishedSimulationDiagnosticsSnapshot snapshot){snapshot=null;return false;}
    public static void Reset(ulong worldId){ }
    public static void Reset(){ }
    public static void RemoveWorld(ulong worldId){ }
}
#endif
}
