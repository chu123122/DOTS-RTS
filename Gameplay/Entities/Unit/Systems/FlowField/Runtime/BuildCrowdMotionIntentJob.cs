using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>Gameplay Adapter 内部的 NavigationField 读取语义。</summary>
internal static class FlowNavigationView
{
    public static bool TryRead(
        NativeArray<FlowFieldCell> cells,
        FlowGridGeometry geometry,
        int2 cell,
        out FlowFieldCell value)
    {
        if (!cells.IsCreated || !geometry.Contains(cell))
        {
            value = default;
            return false;
        }

        value = cells[geometry.FlatIndex(cell)];
        return true;
    }

    public static bool IsReachable(FlowFieldCell cell) =>
        cell.Cost != 0 && cell.IntegrationValue != ushort.MaxValue;
}

/// <summary>
/// Gameplay Adapter 收集导航事实，只产出不可变 CrowdPhysicsBodyInput。
/// </summary>
[BurstCompile]
public partial struct BuildCrowdMotionIntentJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> NavigationCells;
    public FlowGridGeometry NavigationGrid;
    public uint ActiveRequestVersion;

    public NativeArray<CrowdPhysicsBodyInput> StepInputs;

    public void Execute(
        Entity entity,
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings,
        in UnitContactBody contactBody,
        in CrowdDiscShape shape,
        in UnitMoveDestination destination,
        ref FlowArrivalState arrivalState)
    {
        CrowdBodySnapshot body = new CrowdBodySnapshot
        {
            Entity = entity,
            Position = transform.Position,
            Rotation = transform.Rotation,
            Velocity = velocity.Value,
            MoveSpeed = speed.Value,
            MaxAcceleration = settings.MaxForce,
            InverseMass = math.max(0f, contactBody.InverseMass),
            Radius = shape.Radius > 0f
                ? shape.Radius
                : NavigationGrid.CellRadius,
            ShapeVersion = shape.Version
        };
        CrowdNavigationState navigation = default;
        CrowdMotionIntent intent = default;

        int2 cellPosition = NavigationGrid.WorldToCell(transform.Position);
        if (!FlowNavigationView.TryRead(
                NavigationCells,
                NavigationGrid,
                cellPosition,
                out FlowFieldCell cell))
        {
            body.IsInsideSimulationDomain = 0;
            StoreInput(entityIndex, body, navigation, intent);
            return;
        }

        navigation.Cell = cellPosition;
        navigation.BestDirectionIndex = cell.BestDirectionIndex;
        navigation.IntegrationValue = cell.IntegrationValue;
        navigation.IsReachable = (byte)(FlowNavigationView.IsReachable(cell) ? 1 : 0);
        navigation.IsBlocked = (byte)(cell.Cost == 0 ? 1 : 0);

        bool hasActiveDestination = destination.IsActive != 0;
        if (hasActiveDestination && destination.OrderVersion != ActiveRequestVersion)
        {
            arrivalState.IsSettled = true;
            body.Velocity = float3.zero;
            body.IsInsideSimulationDomain = 1;
            navigation.IsSettled = 1;
            StoreInput(entityIndex, body, navigation, intent);
            return;
        }

        float2 destinationDelta = destination.Position.xz - transform.Position.xz;
        float destinationDistance = math.length(destinationDelta);
        float arrivalEnterRadius = math.max(0.01f, destination.ArrivalRadius);
        float arrivalExitRadius = arrivalEnterRadius +
                                  math.max(0.05f, body.Radius * 0.5f);
        bool isSettled = !hasActiveDestination ||
                         (arrivalState.IsSettled
                             ? destinationDistance <= arrivalExitRadius
                             : destinationDistance <= arrivalEnterRadius);
        arrivalState.IsSettled = isSettled;

        float3 preferredVelocity = float3.zero;
        if (hasActiveDestination && !isSettled && navigation.IsReachable != 0)
        {
            bool useDirectApproach =
                navigation.IntegrationValue <= destination.DirectApproachIntegrationDistance;
            if (useDirectApproach)
            {
                float3 preferredDirection = math.normalizesafe(
                    new float3(destinationDelta.x, 0f, destinationDelta.y));
                float brakingDistance = math.max(
                    NavigationGrid.CellRadius * 2f,
                    math.max(body.Radius * 2f, arrivalExitRadius));
                float speedScale = math.saturate(destinationDistance / brakingDistance);
                preferredVelocity = preferredDirection * speed.Value * speedScale;
            }
            else
            {
                int2 directionOffset = FlowFieldUtils.GetDirectionOffset(
                    navigation.BestDirectionIndex);
                float3 preferredDirection = math.normalizesafe(
                    new float3(directionOffset.x, 0f, directionOffset.y));
                preferredVelocity = preferredDirection * speed.Value;
            }
        }

        body.IsInsideSimulationDomain = 1;
        navigation.IsSettled = (byte)(isSettled ? 1 : 0);
        intent.PreferredVelocity = preferredVelocity;
        intent.SteeringVelocityError = preferredVelocity - body.Velocity;

        StoreInput(entityIndex, body, navigation, intent);
    }

    private void StoreInput(
        int entityIndex,
        CrowdBodySnapshot body,
        CrowdNavigationState navigation,
        CrowdMotionIntent intent)
    {
        StepInputs[entityIndex] = new CrowdPhysicsBodyInput
        {
            StableId = body.Entity,
            Position = body.Position,
            Rotation = body.Rotation,
            Velocity = body.Velocity,
            PreferredVelocity = intent.PreferredVelocity,
            SteeringVelocityError = intent.SteeringVelocityError,
            MoveSpeed = body.MoveSpeed,
            MaxAcceleration = body.MaxAcceleration,
            InverseMass = body.InverseMass,
            Radius = body.Radius,
            ShapeVersion = body.ShapeVersion,
            IsInsideSimulationDomain = body.IsInsideSimulationDomain,
            IsSettled = navigation.IsSettled
        };
    }
}

}
