using Entities.Unit.System.FlowFieldSystem;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;
using Unity.NetCode;
using 通用; // 你的命名空间

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
[UpdateAfter(typeof(FlowFieldBakeSystem))] // 确保在路径烘焙之后移动
public partial class UnitFlowMovementSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
    }

    protected override void OnUpdate()
    {
        // 1. 获取 Flow Field 数据
        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGrid>();
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();

        // 安全检查：如果 Grid 还没初始化或者被销毁了，就不跑
        if (!gridComponent.Grid.IsCreated) return;

        float deltaTime = SystemAPI.Time.DeltaTime;

        // 2. 调度并行 Job
        // 我们把 Grid 的关键数据传进去
        var moveJob = new MoveAlongFlowFieldJob
        {
            DeltaTime = deltaTime,
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius
        };

        // ScheduleParallel 会自动切分 Entity 数组，性能极高
        Dependency = moveJob.ScheduleParallel(Dependency);
    }
}

[BurstCompile]
public partial struct MoveAlongFlowFieldJob : IJobEntity
{
    public float DeltaTime;
    
    // --- Flow Field 数据 ---
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    // IJobEntity 会自动匹配拥有这些组件的 Entity
    // 注意：我们需要 Velocity 组件。如果你还没加，记得去给单位加上。
    public void Execute(
        ref LocalTransform transform, 
        ref Velocity velocity, 
        in UnitMoveSpeed speed) // 假设你创建了Settings，如果没有，暂时用硬编码数值
    {
        // 1. 算出当前在哪个格子
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position,GridOrigin,CellRadius);
        
        // 2. 边界检查：如果跑出地图了，就停下或者不做流场力
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x || 
            cellPos.y < 0 || cellPos.y >= GridDimensions.y)
        {
            // 简单处理：减速停车
            velocity.Value = math.lerp(velocity.Value, float3.zero, DeltaTime * 5f);
            return;
        }

        // 3. 读取格子的方向
        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];

        float3 desiredDirection = float3.zero;

        // 如果有有效方向 (255 = 无效/终点)
        if (cell.BestDirectionIndex != 0xFF)
        {
            // 将索引 (0-7) 转换回 float3 方向向量
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            desiredDirection = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
        }
        else
        {
            // 到达终点或在障碍物内 -> 期望速度为 0
            // (这里未来可以加一个微调，让它停得更准)
        }

        // 4. 计算力与速度 (简单的 Steering Behavior)
        // 期望速度
        float3 targetVelocity = desiredDirection * speed.Value;
        
        // 转向力 = 期望速度 - 当前速度
        float3 steeringForce = targetVelocity - velocity.Value;
        
        // 限制转向力 (模拟转弯半径/惯性)
        // 假设 MaxForce 是 10.0f (你可以从 Settings 里读)
        float maxForce = 20.0f; 
        float forceMagnitude = math.length(steeringForce);
        if (forceMagnitude > maxForce)
        {
            steeringForce = (steeringForce / forceMagnitude) * maxForce;
        }

        // 5. 应用物理更新
        // v = v + a * t
        velocity.Value += steeringForce * DeltaTime;

        // 限制最大速度
        // v = clamp(v, maxSpeed)
        float currentSpeed = math.length(velocity.Value);
        if (currentSpeed > speed.Value)
        {
            velocity.Value = (velocity.Value / currentSpeed) * speed.Value;
        }

        // x = x + v * t
        transform.Position += velocity.Value * DeltaTime;

        // 6. 更新朝向 (Visual Polish)
        // 只有当速度足够大时才旋转，防止原地抽搐
        if (math.lengthsq(velocity.Value) > 0.001f)
        {
            transform.Rotation = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
        }
    }
}