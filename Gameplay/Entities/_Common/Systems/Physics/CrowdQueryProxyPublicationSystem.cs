using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;
using RTS.Unit.Components;

namespace RTS.Gameplay.Physics
{
/// <summary>
/// BuildPhysicsWorld 已读取 ECS Transform 后，把本轮构建所消费的 Crowd
/// step 版本发布给 query proxy。下游查询只消费 ProxyVersion。
/// </summary>
[WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(PhysicsSystemGroup))]
[UpdateAfter(typeof(PhysicsInitializeGroup))]
[UpdateBefore(typeof(PhysicsSimulationGroup))]
public partial class CrowdQueryProxyPublicationSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<PhysicsWorldSingleton>();
        RequireForUpdate<CrowdQueryProxy>();
    }

    protected override void OnUpdate()
    {
        // 显式声明对 BuildPhysicsWorld 产品的读取，使发布 Job 进入同一依赖链。
        SystemAPI.GetSingleton<PhysicsWorldSingleton>();
        Dependency = new PublishCrowdQueryProxyVersionJob()
            .ScheduleParallel(Dependency);
    }
}

[BurstCompile]
internal partial struct PublishCrowdQueryProxyVersionJob : IJobEntity
{
    public void Execute(ref CrowdQueryProxy proxy)
    {
        proxy.ProxyVersion = proxy.CrowdStepVersion;
    }
}
}
