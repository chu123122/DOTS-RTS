using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Transforms;
using _RePlaySystem.Base; // 引用 LocalInstance 所在的命名空间

namespace 通用
{
    [UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
    public partial class InitializeUnitSystem : SystemBase
    {
        public Action<int, float3> OnCreateHealthBar;

        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            // 遍历所有带有 NewBasicUnitTag 的实体
            foreach (var (physicsMass, rtsTeam, localTransform, entity) in
                     SystemAPI.Query<RefRW<PhysicsMass>,
                             RefRO<RtsTeam>,
                             RefRO<LocalTransform>>()
                         .WithAll<NewBasicUnitTag>() // 只查询新单位
                         .WithEntityAccess())
            {
                int unitId = -1;
                bool readyToInitialize = false;

                // --- 检查 1: 联机模式 (GhostInstance) ---
                if (SystemAPI.HasComponent<GhostInstance>(entity))
                {
                    unitId = SystemAPI.GetComponent<GhostInstance>(entity).ghostId;
                    readyToInitialize = true;
                }
                // --- 检查 2: 回放/本地模式 (LocalInstance) ---
                else if (SystemAPI.HasComponent<LocalInstance>(entity))
                {
                    unitId = SystemAPI.GetComponent<LocalInstance>(entity).Id;
                    readyToInitialize = true;
                }
                // --- 检查 3: 组件尚未同步 (Waiting) ---
                else
                {
                    // NetCode 还没把组件挂上去，跳过，下帧再处理
                    // 此时不要 RemoveComponent<NewBasicUnitTag>，保留它
                    continue;
                }

                // --- 执行初始化 ---
                if (readyToInitialize)
                {
                    // 1. 物理锁定 (防止被撞翻)
                    physicsMass.ValueRW.InverseInertia = float3.zero;

                    // 2. 队伍颜色处理
                    var teamColor = rtsTeam.ValueRO.Value switch
                    {
                        TeamType.Blue => new float4(0, 0, 1, 1),
                        TeamType.Red => new float4(1, 0, 0, 1),
                        _ => new float4(1)
                    };
                    
                    // ecb.SetComponent(entity, new URPMaterialPropertyBaseColor(){Value = teamColor});

                    // 3. 触发 UI 回调
                    OnCreateHealthBar?.Invoke(unitId, localTransform.ValueRO.Position);

                    // 4. 移除标记，表示初始化完成
                    ecb.RemoveComponent<NewBasicUnitTag>(entity);
                }
            }

            ecb.Playback(EntityManager);
            ecb.Dispose();
        }
    }
}