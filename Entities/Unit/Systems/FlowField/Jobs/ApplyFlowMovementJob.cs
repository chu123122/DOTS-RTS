using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using RTS.Unit.Components;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 把人群求解结果写回 ECS。不接触求解器、校验或导航内部状态。
/// </summary>
[BurstCompile]
public partial struct ApplyFlowMovementJob : IJobEntity
{
    [ReadOnly] public NativeArray<CrowdBodyResult> Results;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        ref LocalTransform transform,
        ref Velocity velocity)
    {
        CrowdBodyResult result = Results[entityIndex];
        transform.Position = result.Position;
        transform.Rotation = result.Rotation;
        velocity.Value = result.Velocity;
    }
}
}
