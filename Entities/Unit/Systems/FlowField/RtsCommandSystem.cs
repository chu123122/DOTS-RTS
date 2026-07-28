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
        RequireForUpdate<FlowFieldRuntimeState>();
        RequireForUpdate<MoveOrder>();
        RequireForUpdate<MoveOrderSelectionElement>();
    }

    protected override void OnUpdate()
    {
        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();

        // 首次 Flow Field 还没发布时，保留 MoveOrder 的 enabled 状态。
        // 启动烘焙完成后的下一帧会继续消费同一条指令。
        if (SystemAPI.GetSingleton<FlowFieldRuntimeState>().ActiveVersion == 0)
            return;

        var moveOrder = EntityManager.GetComponentData<MoveOrder>(gridEntity);

        FlowFieldGrid grid = SystemAPI.GetSingleton<FlowFieldGrid>();
        DynamicBuffer<MoveOrderSelectionElement> recipients =
            EntityManager.GetBuffer<MoveOrderSelectionElement>(gridEntity);
        if (recipients.Length == 0)
        {
            // 空快照不能等后续左键选择，否则旧目标会错误绑到未来选择。
            EntityManager.SetComponentEnabled<MoveOrder>(gridEntity, false);
            return;
        }

        var selectedUnits = new List<SelectedUnit>();
        for (int recipientIndex = 0; recipientIndex < recipients.Length; recipientIndex++)
        {
            Entity entity = recipients[recipientIndex].Entity;
            if (!EntityManager.Exists(entity) ||
                !EntityManager.HasComponent<BasicUnitTag>(entity) ||
                !EntityManager.HasComponent<UnitMoveDestination>(entity) ||
                !EntityManager.HasComponent<LocalTransform>(entity))
            {
                continue;
            }

            LocalTransform transform = EntityManager.GetComponentData<LocalTransform>(entity);
            selectedUnits.Add(new SelectedUnit
            {
                Entity = entity,
                Position = transform.Position,
                FootprintSpan = CalculateFootprintSpan(
                    entity,
                    transform,
                    grid.CellRadius * 2f)
            });
        }

        if (selectedUnits.Count == 0)
        {
            recipients.Clear();
            EntityManager.SetComponentEnabled<MoveOrder>(gridEntity, false);
            return;
        }

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

        // 只有订单快照拿到完整槽位时才消费；不修改实时 UnitSelected。
        EntityManager.SetComponentEnabled<MoveOrder>(gridEntity, false);
        recipients.Clear();

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
