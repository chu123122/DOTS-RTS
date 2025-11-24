using _RePlaySystem.Base;
using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using 通用;

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class RtsCommandSystem : SystemBase
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

        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
        var currentTarget = SystemAPI.GetComponent<FlowFieldGlobalTarget>(gridEntity);

        bool newCommandFound = false;
        float3 newTargetPos = float3.zero;

        foreach (var input in
                 SystemAPI.Query<
                     RefRW<UnitMoveTargetPosition>>())
        {
            if (input.ValueRO.GetMoveInput)
            {
                input.ValueRW.GetMoveInput = false; 

                if (math.distance(input.ValueRO.Value, currentTarget.TargetPosition) < 0.1f)
                    continue; 

                newCommandFound = true;
                newTargetPos = input.ValueRO.Value;
                
            }
        }

        if (newCommandFound)
        {
            SystemAPI.SetComponent(gridEntity, new FlowFieldGlobalTarget { TargetPosition = newTargetPos });

            if (!SystemAPI.HasComponent<RecalculateFlowFieldTag>(gridEntity))
            {
                EntityManager.AddComponent<RecalculateFlowFieldTag>(gridEntity);
                Debug.Log($"[RTSCommand] New Order Received: {newTargetPos}. Baking...");
            }
        }
    }
}