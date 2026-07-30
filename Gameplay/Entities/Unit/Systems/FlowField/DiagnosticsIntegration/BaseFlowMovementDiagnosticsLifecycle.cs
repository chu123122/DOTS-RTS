using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
#if RTS_CONTACT_DIAGNOSTICS
    private Entity _incrementalDiagnosticsEntity;
    private EntityQuery _legacyDiagnosticSelectionQuery;
#else
    private Entity _incrementalDiagnosticsEntity { get => Entity.Null; set { } }
#endif

    private void CreatePersistentDiagnostics()
    {
#if RTS_CONTACT_DIAGNOSTICS
        _legacyDiagnosticSelectionQuery = GetEntityQuery(
            ComponentType.ReadOnly<ContactDiagnosticSelection>());
        _incrementalDiagnosticsEntity = EntityManager.CreateEntity(
            typeof(IncrementalContactPipelineSnapshot),
            typeof(PredictiveDiscContactStatistics),
            typeof(ShadowNeighborCacheStatistics),
            typeof(SelectedBodyContactDiagnostic));
        EntityManager.AddBuffer<ContactIterationDiagnostic>(
            _incrementalDiagnosticsEntity);
        EntityManager.AddBuffer<ContactPairDiagnostic>(
            _incrementalDiagnosticsEntity);
        EntityManager.AddBuffer<ContactHeatSample>(
            _incrementalDiagnosticsEntity);
#endif
    }

    private void DisposePersistentDiagnostics()
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EntityManager.Exists(_incrementalDiagnosticsEntity)) EntityManager.DestroyEntity(_incrementalDiagnosticsEntity);
#endif
    }

    private Entity ResolveDiagnosticSelectedEntity(ulong worldId)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Entity selected = SimulationDebuggerRuntime.SelectedEntityFor(worldId);
        int legacySelectionCount =
            _legacyDiagnosticSelectionQuery.CalculateEntityCount();
        if (legacySelectionCount == 0)
            return selected;

        using NativeArray<ContactDiagnosticSelection> legacySelections =
            _legacyDiagnosticSelectionQuery
                .ToComponentDataArray<ContactDiagnosticSelection>(
                    Allocator.Temp);
        Entity legacySelected = Entity.Null;
        for (int i = 0; i < legacySelections.Length; i++)
        {
            Entity candidate = legacySelections[i].SelectedEntity;
            if (candidate == Entity.Null)
                continue;
            if (legacySelected != Entity.Null && candidate != legacySelected)
                return selected;
            legacySelected = candidate;
        }
        if (legacySelected == Entity.Null)
            return selected;

        selected = legacySelected;
        SimulationDebuggerRuntime.SetSelectedEntityFor(worldId, selected);
        return selected;
#else
        return Entity.Null;
#endif
    }

    private bool TryGetCompletedIncrementalContactSnapshot(
        out IncrementalContactPipelineSnapshot snapshot)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (_incrementalDiagnosticsEntity != Entity.Null &&
            EntityManager.Exists(_incrementalDiagnosticsEntity) &&
            EntityManager.HasComponent<IncrementalContactPipelineSnapshot>(
                _incrementalDiagnosticsEntity))
        {
            snapshot = EntityManager.GetComponentData<IncrementalContactPipelineSnapshot>(
                _incrementalDiagnosticsEntity);
            return snapshot.Statistics.Timestep != 0;
        }
#endif
        snapshot = default;
        return false;
    }
}
}
