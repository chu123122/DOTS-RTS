using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

[WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class RtsCommandSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGlobalTarget>();
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<MoveOrder>();
    }

    protected override void OnUpdate()
    {
        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
        var moveOrder = EntityManager.GetComponentData<MoveOrder>(gridEntity);
        EntityManager.SetComponentEnabled<MoveOrder>(gridEntity, false);

        FlowFieldGrid grid = SystemAPI.GetSingleton<FlowFieldGrid>();
        var selectedUnits = new List<SelectedUnit>();
        foreach (var (selection, transform, entity) in
                 SystemAPI.Query<RefRO<UnitSelected>, RefRO<LocalTransform>>()
                     .WithAll<BasicUnitTag, UnitMoveDestination>()
                     .WithEntityAccess())
        {
            if (!selection.ValueRO.Value)
                continue;

            selectedUnits.Add(new SelectedUnit
            {
                Entity = entity,
                Position = transform.ValueRO.Position,
                FootprintSpan = CalculateFootprintSpan(
                    entity,
                    transform.ValueRO,
                    grid.CellRadius * 2f)
            });
        }

        if (selectedUnits.Count == 0)
            return;

        selectedUnits.Sort((a, b) =>
        {
            int indexComparison = a.Entity.Index.CompareTo(b.Entity.Index);
            return indexComparison != 0
                ? indexComparison
                : a.Entity.Version.CompareTo(b.Entity.Version);
        });

        float maximumSpan = 0f;
        var positions = new List<float3>(selectedUnits.Count);
        foreach (SelectedUnit unit in selectedUnits)
        {
            maximumSpan = math.max(maximumSpan, unit.FootprintSpan);
            positions.Add(unit.Position);
        }

        float slotSpacing = math.max(
            math.max(0.05f, maximumSpan) + 0.05f,
            math.max(0.05f, grid.CellRadius * 0.5f));
        List<float3> slots = MoveDestinationSlotUtility.GenerateWalkableSlots(
            moveOrder.TargetPosition,
            selectedUnits.Count,
            slotSpacing,
            grid);
        if (slots.Count < selectedUnits.Count)
        {
            UnityEngine.Debug.LogError(
                $"固定槽位不足：需要 {selectedUnits.Count}，可用 {slots.Count}。移动订单未应用。");
            return;
        }

        int[] slotAssignments = MoveDestinationSlotUtility.AssignSlotsPreservingFormation(
            positions,
            slots,
            moveOrder.TargetPosition);

        foreach (var (destination, arrivalState) in
                 SystemAPI.Query<RefRW<UnitMoveDestination>, RefRW<FlowArrivalState>>()
                     .WithAll<BasicUnitTag>())
        {
            destination.ValueRW.IsActive = 0;
            arrivalState.ValueRW.IsSettled = true;
        }

        RecalculateFlowFieldTag request =
            EntityManager.GetComponentData<RecalculateFlowFieldTag>(gridEntity);
        request.RequestVersion++;
        float cellSize = math.max(0.0001f, grid.CellRadius * 2f);
        float3 flowFieldTarget = slots[0];
        for (int unitIndex = 0; unitIndex < selectedUnits.Count; unitIndex++)
        {
            SelectedUnit unit = selectedUnits[unitIndex];
            float3 slot = slots[slotAssignments[unitIndex]];
            int directApproachDistance =
                (int)math.ceil(math.distance(slot.xz, flowFieldTarget.xz) / cellSize) + 2;
            EntityManager.SetComponentData(unit.Entity, new UnitMoveDestination
            {
                Position = slot,
                ArrivalRadius = math.max(0.05f, unit.FootprintSpan * 0.2f),
                DirectApproachIntegrationDistance = directApproachDistance,
                OrderVersion = request.RequestVersion,
                IsActive = 1
            });
            EntityManager.SetComponentData(
                unit.Entity,
                new FlowArrivalState { IsSettled = false });
        }

        SystemAPI.SetComponent(gridEntity, new FlowFieldGlobalTarget
        {
            TargetPosition = flowFieldTarget
        });
        EntityManager.SetComponentData(gridEntity, request);
        EntityManager.SetComponentEnabled<RecalculateFlowFieldTag>(gridEntity, true);
    }

    private float CalculateFootprintSpan(
        Entity entity,
        LocalTransform transform,
        float fallbackSpan)
    {
        if (!EntityManager.HasComponent<PhysicsCollider>(entity))
            return math.max(0.05f, fallbackSpan);

        PhysicsCollider physicsCollider = EntityManager.GetComponentData<PhysicsCollider>(entity);
        if (!physicsCollider.IsValid)
            return math.max(0.05f, fallbackSpan);

        float uniformScale = math.max(math.abs(transform.Scale), 0.0001f);
        var colliderTransform = new RigidTransform(transform.Rotation, float3.zero);
        Aabb aabb = physicsCollider.Value.Value.CalculateAabb(colliderTransform, uniformScale);
        float3 size = aabb.Max - aabb.Min;
        return math.max(0.05f, math.max(size.x, size.z));
    }

    private struct SelectedUnit
    {
        public Entity Entity;
        public float3 Position;
        public float FootprintSpan;
    }
}
}
