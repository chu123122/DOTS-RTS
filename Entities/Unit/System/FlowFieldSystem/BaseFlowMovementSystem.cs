using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

public abstract partial class BaseFlowMovementSystem : SystemBase
{
    private EntityQuery _movementQuery;

    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<UnitSpatialMap>();

        _movementQuery = GetEntityQuery(
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<Velocity>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>());
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var spatialMap = SystemAPI.GetSingleton<UnitSpatialMap>();
        if (!gridComponent.Grid.IsCreated) return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0) return;

        var states = new NativeArray<FlowMovementFrameState>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);
        var predictedPositions = new NativeParallelHashMap<Entity, float3>(unitCount, Allocator.TempJob);

        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(isReadOnly: true);
        transformLookup.Update(this);

        var independentForceJob = new CalculateIndependentFlowForceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            States = states
        };
        JobHandle independentForceHandle = independentForceJob.ScheduleParallel(_movementQuery, Dependency);

        var softAvoidanceJob = new CalculateSoftAvoidanceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            SpatialMap = spatialMap.Map,
            TransformLookup = transformLookup,
            SeparationWeight = 4f,
            SeparationRadius = 0.6f,
            States = states
        };
        JobHandle softAvoidanceHandle = softAvoidanceJob.ScheduleParallel(_movementQuery, independentForceHandle);

        var integrateForcesJob = new IntegrateFlowForcesJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            GridOrigin = gridComponent.GridOrigin,
            CellRadius = gridComponent.CellRadius,
            PredictedPositions = predictedPositions.AsParallelWriter(),
            States = states
        };
        JobHandle integrateForcesHandle = integrateForcesJob.ScheduleParallel(_movementQuery, softAvoidanceHandle);

        var constraintJob = new CalculateFlowConstraintsJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            SpatialMap = spatialMap.Map,
            PredictedPositions = predictedPositions,
            SeparationRadius = 0.6f,
            States = states
        };
        JobHandle constraintHandle = constraintJob.ScheduleParallel(_movementQuery, integrateForcesHandle);

        var applyMovementJob = new ApplyFlowMovementJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            States = states
        };
        JobHandle applyMovementHandle = applyMovementJob.ScheduleParallel(_movementQuery, constraintHandle);
        JobHandle stateDisposeHandle = states.Dispose(applyMovementHandle);
        JobHandle predictedPositionDisposeHandle = predictedPositions.Dispose(applyMovementHandle);
        Dependency = JobHandle.CombineDependencies(stateDisposeHandle, predictedPositionDisposeHandle);
    }
}

public struct FlowMovementFrameState
{
    public float3 CurrentPosition;
    public quaternion CurrentRotation;
    public float3 CurrentVelocity;
    public float MoveSpeed;
    public float MaxForce;

    public int2 CellPosition;
    public FlowFieldCell Cell;
    public float FlowWeight;
    public bool IsAtDestination;
    public bool IsInsideGrid;

    public float3 IndependentForce;
    public float3 SoftAvoidanceForce;
    public float3 IntegratedVelocity;
    public float3 PredictedPosition;
    public float3 PositionCorrection;
}

[BurstCompile]
public partial struct CalculateIndependentFlowForceJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings)
    {
        var state = new FlowMovementFrameState
        {
            CurrentPosition = transform.Position,
            CurrentRotation = transform.Rotation,
            CurrentVelocity = velocity.Value,
            MoveSpeed = speed.Value,
            MaxForce = settings.MaxForce
        };

        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x ||
            cellPos.y < 0 || cellPos.y >= GridDimensions.y)
        {
            state.IsInsideGrid = false;
            States[entityIndex] = state;
            return;
        }

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];
        const int arrivalDistance = 2;
        float flowWeight = 1.0f;
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            float linearT = (float)cell.IntegrationValue / arrivalDistance;
            flowWeight = math.sqrt(linearT);
        }

        bool isAtDestination = cell.IntegrationValue == 0;
        float3 moveForce = float3.zero;
        if (!isAtDestination && cell.Cost != 0)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = desiredDir * speed.Value * flowWeight - velocity.Value;
        }

        state.CellPosition = cellPos;
        state.Cell = cell;
        state.FlowWeight = flowWeight;
        state.IsAtDestination = isAtDestination;
        state.IsInsideGrid = true;
        state.IndependentForce = moveForce;
        States[entityIndex] = state;
    }
}

[BurstCompile]
public partial struct CalculateSoftAvoidanceJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;

    public float SeparationWeight;
    public float SeparationRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 separationForce = float3.zero;
        int neighborCount = 0;

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = state.CellPosition + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost == 0)
                {
                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius,
                        state.CurrentPosition.y,
                        checkCell.y * CellRadius * 2 + CellRadius);

                    AccumulateWallSoftForce(state.CurrentPosition, wallPosition, state.MoveSpeed, ref separationForce);
                    continue;
                }

                if (!SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var iterator))
                    continue;

                do
                {
                    if (neighborEntity == entity) continue;
                    if (!TransformLookup.HasComponent(neighborEntity)) continue;

                    float3 neighborPosition = TransformLookup[neighborEntity].Position;
                    AccumulateUnitSoftForce(
                        state.CurrentPosition,
                        neighborPosition,
                        state.MoveSpeed,
                        ref separationForce,
                        ref neighborCount);
                } while (SpatialMap.TryGetNextValue(out neighborEntity, ref iterator));
            }
        }

        if (neighborCount > 0)
        {
            separationForce /= neighborCount;
            float currentWeight = state.IsAtDestination ? SeparationWeight * 1.5f : SeparationWeight;
            separationForce *= currentWeight;
        }

        state.SoftAvoidanceForce = separationForce;
        States[entityIndex] = state;
    }

    private void AccumulateWallSoftForce(
        float3 position,
        float3 wallPosition,
        float moveSpeed,
        ref float3 separationForce)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDirection = diff / dist;
        float repelStrength = (wallCheckRadius - dist) / dist * 10.0f;
        separationForce += pushDirection * repelStrength * moveSpeed;
    }

    private void AccumulateUnitSoftForce(
        float3 position,
        float3 neighborPosition,
        float moveSpeed,
        ref float3 separationForce,
        ref int neighborCount)
    {
        float3 diff = position - neighborPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float separationRadiusSq = SeparationRadius * SeparationRadius;
        if (distSq >= separationRadiusSq || distSq <= 0.00001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDirection = diff / dist;
        float softFactor = 1.0f - dist / SeparationRadius;
        separationForce += pushDirection * softFactor * moveSpeed;
        neighborCount++;
    }
}

[BurstCompile]
public partial struct IntegrateFlowForcesJob : IJobEntity
{
    public float DeltaTime;
    public float3 GridOrigin;
    public float CellRadius;
    public NativeParallelHashMap<Entity, float3>.ParallelWriter PredictedPositions;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid)
        {
            state.IntegratedVelocity = float3.zero;
            state.PredictedPosition = state.CurrentPosition;
            States[entityIndex] = state;
            return;
        }

        float3 totalForce = state.IndependentForce + state.SoftAvoidanceForce;
        if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 cellCenter = GridOrigin + new float3(
                state.CellPosition.x * CellRadius * 2 + CellRadius,
                state.CurrentPosition.y,
                state.CellPosition.y * CellRadius * 2 + CellRadius);
            float3 escapeDirection = math.normalize(state.CurrentPosition - cellCenter);
            if (math.lengthsq(escapeDirection) < 0.001f)
                escapeDirection = new float3(1, 0, 0);
            totalForce += escapeDirection * state.MoveSpeed * 5.0f;
        }

        if (math.length(totalForce) > state.MaxForce)
            totalForce = math.normalize(totalForce) * state.MaxForce;

        float3 integratedVelocity = state.CurrentVelocity + totalForce * DeltaTime;
        if (state.IsAtDestination)
        {
            integratedVelocity *= math.pow(0.8f, DeltaTime * 60f);
        }
        else if (state.FlowWeight < 0.99f)
        {
            integratedVelocity *= math.pow(0.95f, DeltaTime * 60f);
        }

        if (math.length(integratedVelocity) > state.MoveSpeed)
            integratedVelocity = math.normalize(integratedVelocity) * state.MoveSpeed;

        float3 predictedPosition = state.CurrentPosition + integratedVelocity * DeltaTime;
        predictedPosition.y = state.CurrentPosition.y;

        state.IntegratedVelocity = integratedVelocity;
        state.PredictedPosition = predictedPosition;
        States[entityIndex] = state;
        PredictedPositions.TryAdd(entity, predictedPosition);
    }
}

[BurstCompile]
public partial struct CalculateFlowConstraintsJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] public NativeParallelHashMap<Entity, float3> PredictedPositions;

    public float SeparationRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 positionCorrection = float3.zero;

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = state.CellPosition + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost == 0)
                {
                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius,
                        state.PredictedPosition.y,
                        checkCell.y * CellRadius * 2 + CellRadius);

                    AccumulateWallConstraint(state.PredictedPosition, wallPosition, ref positionCorrection);
                    continue;
                }

                if (!SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var iterator))
                    continue;

                do
                {
                    if (neighborEntity == entity) continue;
                    if (!PredictedPositions.TryGetValue(neighborEntity, out float3 neighborPosition)) continue;

                    AccumulateUnitConstraint(state.PredictedPosition, neighborPosition, ref positionCorrection);
                } while (SpatialMap.TryGetNextValue(out neighborEntity, ref iterator));
            }
        }

        state.PositionCorrection = positionCorrection;
        States[entityIndex] = state;
    }

    private void AccumulateWallConstraint(
        float3 position,
        float3 wallPosition,
        ref float3 positionCorrection)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float wallHardRadius = CellRadius + 0.5f;
        if (dist >= wallHardRadius) return;

        float3 pushDirection = diff / dist;
        float penetration = wallHardRadius - dist;
        positionCorrection += pushDirection * (penetration * 0.5f);
    }

    private void AccumulateUnitConstraint(
        float3 position,
        float3 neighborPosition,
        ref float3 positionCorrection)
    {
        float3 diff = position - neighborPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float separationRadiusSq = SeparationRadius * SeparationRadius;
        if (distSq >= separationRadiusSq || distSq <= 0.00001f)
            return;

        float dist = math.sqrt(distSq);
        const float hardRadius = 0.5f;
        if (dist >= hardRadius) return;

        float3 pushDirection = diff / dist;
        float penetration = hardRadius - dist;
        positionCorrection += pushDirection * (penetration * 0.4f);
    }
}

[BurstCompile]
public partial struct ApplyFlowMovementJob : IJobEntity
{
    public float DeltaTime;
    [ReadOnly] public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        ref LocalTransform transform,
        ref Velocity velocity)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid)
        {
            velocity.Value = float3.zero;
            return;
        }

        float3 integratedVelocity = state.IntegratedVelocity;
        float3 positionCorrection = state.PositionCorrection;
        bool isHardColliding = math.lengthsq(positionCorrection) > 0.0001f;

        bool shouldMove = math.lengthsq(integratedVelocity) > 0.005f || isHardColliding;
        if (shouldMove)
        {
            float3 newPosition = state.PredictedPosition;
            if (isHardColliding)
            {
                const float maxCorrectionPerFrame = 0.15f;
                if (math.lengthsq(positionCorrection) > maxCorrectionPerFrame * maxCorrectionPerFrame)
                    positionCorrection = math.normalize(positionCorrection) * maxCorrectionPerFrame;

                newPosition += positionCorrection;
            }

            newPosition.y = state.CurrentPosition.y;
            transform.Position = newPosition;
            integratedVelocity.y = 0;

            if (math.lengthsq(integratedVelocity) > 0.01f)
            {
                quaternion targetRotation = quaternion.LookRotationSafe(math.normalize(integratedVelocity), math.up());
                transform.Rotation = math.slerp(state.CurrentRotation, targetRotation, DeltaTime * 10.0f);
            }
        }
        else if (state.IsAtDestination && !isHardColliding)
        {
            integratedVelocity = float3.zero;
        }

        velocity.Value = integratedVelocity;
    }
}
