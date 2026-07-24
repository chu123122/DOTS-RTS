using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private NativeList<SimulationDebuggerPairSample> _simulationDebuggerSelectedPairs;
    private NativeReference<SimulationDebuggerUnitSample> _simulationDebuggerSelectedUnit;
    private NativeReference<byte> _simulationDebuggerSelectedUnitValid;
    private Entity _incrementalDiagnosticsEntity;

    private void CreatePersistentDiagnostics()
    {
        _simulationDebuggerSelectedPairs =
            new NativeList<SimulationDebuggerPairSample>(64, Allocator.Persistent);
        _simulationDebuggerSelectedUnit =
            new NativeReference<SimulationDebuggerUnitSample>(Allocator.Persistent);
        _simulationDebuggerSelectedUnitValid =
            new NativeReference<byte>(Allocator.Persistent);
        _incrementalDiagnosticsEntity = EntityManager.CreateEntity(
            typeof(IncrementalContactPipelineSnapshot));
    }

    private void DisposePersistentDiagnostics()
    {
        if (EntityManager.Exists(_incrementalDiagnosticsEntity))
            EntityManager.DestroyEntity(_incrementalDiagnosticsEntity);
        if (_simulationDebuggerSelectedPairs.IsCreated)
            _simulationDebuggerSelectedPairs.Dispose();
        if (_simulationDebuggerSelectedUnit.IsCreated)
            _simulationDebuggerSelectedUnit.Dispose();
        if (_simulationDebuggerSelectedUnitValid.IsCreated)
            _simulationDebuggerSelectedUnitValid.Dispose();
    }
}
}
