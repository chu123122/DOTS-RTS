using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Captures body facts and produces navigation-only movement intent. It reads the
/// shared cell storage through FlowNavigationView and does not retain a complete
/// FlowFieldCell in the simulation frame state.
/// </summary>
[BurstCompile]
public partial struct CalculateIndependentFlowForceJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> NavigationCells;
    public FlowGridGeometry NavigationGrid;
    public uint ActiveRequestVersion;

    [ReadOnly] public NativeArray<float2> CollisionFootprints;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        Entity entity,
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings,
        in UnitContactBody contactBody,
        in UnitMoveDestination destination,
        ref FlowArrivalState arrivalState)
    {
        var state = new FlowMovementFrameState
        {
            Entity = entity,
            CurrentPosition = transform.Position,
            CurrentRotation = transform.Rotation,
            CurrentVelocity = velocity.Value,
            MoveSpeed = speed.Value,
            MaxForce = settings.MaxForce,
            InverseMass = math.max(0f, contactBody.InverseMass),
            Radius = math.cmax(CollisionFootprints[entityIndex]) * 0.5f
        };

        int2 cellPos = NavigationGrid.WorldToCell(transform.Position);
        if (!FlowNavigationView.TryRead(
                NavigationCells,
                NavigationGrid,
                cellPos,
                out FlowFieldCell cell))
        {
            state.IsInsideGrid = false;
            States[entityIndex] = state;
            return;
        }

        WriteNavigationCellSemantics(ref state, cellPos, cell);
        bool hasActiveDestination = destination.IsActive != 0;
        if (hasActiveDestination && destination.OrderVersion != ActiveRequestVersion)
        {
            arrivalState.IsSettled = true;
            state.CurrentVelocity = float3.zero;
            state.IsSettled = true;
            state.IsInsideGrid = true;
            state.MotionIntent.PreferredVelocity = float3.zero;
            state.IndependentForce = float3.zero;
            States[entityIndex] = state;
            return;
        }

        float2 destinationDelta = destination.Position.xz - transform.Position.xz;
        float destinationDistance = math.length(destinationDelta);
        float arrivalEnterRadius = math.max(0.01f, destination.ArrivalRadius);
        float arrivalExitRadius = arrivalEnterRadius +
                                  math.max(0.05f, state.Radius * 0.5f);
        bool isSettled = !hasActiveDestination ||
                         (arrivalState.IsSettled
                             ? destinationDistance <= arrivalExitRadius
                             : destinationDistance <= arrivalEnterRadius);
        arrivalState.IsSettled = isSettled;

        float3 desiredVelocity = float3.zero;
        if (hasActiveDestination && !isSettled &&
            state.Navigation.IsReachable != 0)
        {
            bool useDirectApproach =
                state.Navigation.IntegrationValue <=
                destination.DirectApproachIntegrationDistance;
            if (useDirectApproach)
            {
                float3 desiredDirection = math.normalizesafe(
                    new float3(destinationDelta.x, 0f, destinationDelta.y));
                float brakingDistance = math.max(
                    NavigationGrid.CellRadius * 2f,
                    math.max(state.Radius * 2f, arrivalExitRadius));
                float speedScale = math.saturate(destinationDistance / brakingDistance);
                desiredVelocity = desiredDirection * speed.Value * speedScale;
            }
            else
            {
                int2 dirOffset = FlowFieldUtils.GetDirectionOffset(
                    state.Navigation.BestDirectionIndex);
                float3 desiredDirection = math.normalizesafe(
                    new float3(dirOffset.x, 0f, dirOffset.y));
                desiredVelocity = desiredDirection * speed.Value;
            }
        }

        state.IsSettled = isSettled;
        state.IsInsideGrid = true;
        state.MotionIntent.PreferredVelocity = desiredVelocity;
        state.IndependentForce = desiredVelocity - velocity.Value;
        States[entityIndex] = state;
    }

    private static void WriteNavigationCellSemantics(
        ref FlowMovementFrameState state,
        int2 cellPosition,
        FlowFieldCell cell)
    {
        state.Navigation.Cell = cellPosition;
        state.Navigation.BestDirectionIndex = cell.BestDirectionIndex;
        state.Navigation.IntegrationValue = cell.IntegrationValue;
        state.Navigation.IsReachable = (byte)(
            FlowNavigationView.IsReachable(cell) ? 1 : 0);
        state.Navigation.IsBlocked = (byte)(cell.Cost == 0 ? 1 : 0);
    }
}
}
