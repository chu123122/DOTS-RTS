using _RePlaySystem.Base;
using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using 通用;

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class RTSCommandSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGlobalTarget>();
        RequireForUpdate<NetworkTime>();
    }

    protected override void OnUpdate()
    {
        var networkTime = SystemAPI.GetSingleton<NetworkTime>();
        if (!networkTime.IsFirstTimeFullyPredictingTick) return;

        // 1. 获取当前已经生效的全局目标
        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
        var currentTarget = SystemAPI.GetComponent<FlowFieldGlobalTarget>(gridEntity);

        bool newCommandFound = false;
        float3 newTargetPos = float3.zero;

        // 2. 寻找新输入
        foreach (var input in SystemAPI.Query<RefRW<UnitMoveTargetPosition>>())
        {
            if (input.ValueRO.GetMoveInput)
            {
                // 即使我们这里设为 false，下一帧它可能还会变回 true
                // 但这没关系，下面的逻辑会过滤掉它
                input.ValueRW.GetMoveInput = false; 

                // 检查：新输入的目标点，是否和当前 Grid 的目标点足够接近？
                // 如果距离小于 0.1，说明是同一个指令的后续残留信号，直接跳过
                if (math.distance(input.ValueRO.Value, currentTarget.TargetPosition) < 0.1f)
                {
                    continue; 
                }

                newCommandFound = true;
                newTargetPos = input.ValueRO.Value;
                
                // 这里不需要 break，我们要把所有单位的输入状态都重置一下（虽然是暂时的）
            }
        }

        // 3. 只有当目标点真正改变时，才触发 Bake
        if (newCommandFound)
        {
            // 更新目标
            SystemAPI.SetComponent(gridEntity, new FlowFieldGlobalTarget { TargetPosition = newTargetPos });

            // 触发烘焙
            if (!SystemAPI.HasComponent<RecalculateFlowFieldTag>(gridEntity))
            {
                EntityManager.AddComponent<RecalculateFlowFieldTag>(gridEntity);
                Debug.Log($"[RTSCommand] New Order Received: {newTargetPos}. Baking...");
            }
        }
    }
}