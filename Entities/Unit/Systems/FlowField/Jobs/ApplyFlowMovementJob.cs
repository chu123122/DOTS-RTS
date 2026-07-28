using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using RTS.Unit.Components;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Writes detached crowd-step results back to ECS. It has no access to solver,
/// certification or navigation internals.
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
