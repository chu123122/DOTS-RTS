using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Systems
{

#if RTS_CONTACT_DIAGNOSTICS
[UpdateInGroup(typeof(CrowdPhysicsSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(LocalUnitFlowMovementSystem))]
public partial class IncrementalContactPipelineDiagnosticsSystem : SystemBase
{
    private ulong _lastRecordedGeneration;

    protected override void OnUpdate()
    {
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        if (!PublishedSimulationDiagnosticsRuntime.TryGetLatest(
                worldId,
                out PublishedSimulationDiagnosticsSnapshot published) ||
            published.Generation == _lastRecordedGeneration)
            return;

        _lastRecordedGeneration = published.Generation;
        IncrementalContactPipelineCsvRecorderRuntime.TryRecord(published.Pipeline);
    }
}
#endif

}
