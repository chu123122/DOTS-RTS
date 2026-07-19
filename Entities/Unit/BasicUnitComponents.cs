using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.Serialization;

namespace 通用
{
    public struct BasicUnitTag : IComponentData
    {
    }

    public struct IsNewCreatingTag : IComponentData
    {
    }

    public struct RtsTeam : IComponentData
    {
        [GhostField] public TeamType Value;
    }

    public struct UnitMoveSpeed : IComponentData
    {
        public float Value;
    }

    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct UnitMoveTargetPosition : IInputComponentData
    {
        [GhostField(Quantization = 0)]public float3 Value;
        [GhostField(Quantization = 0)] public bool GetMoveInput;
    }
    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct UnitSelected : IInputComponentData
    {
        [GhostField]public bool Value;
    }
    
    public struct Velocity : IComponentData
    {
        public float3 Value;
    }

    /// <summary>
    /// 记录单位是否已经进入当前流场目标的到达区域。
    /// 该状态跨帧保留，用于为进入/退出到达区域提供滞回，避免边界反复启停。
    /// </summary>
    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct FlowArrivalState : IComponentData
    {
        [GhostField] public bool IsSettled;
    }

    public struct UnitMovementSettings : IComponentData
    {
        public float MaxForce; // 转向力的最大值 (建议 20-50)
        public float RotationSpeed; // 转身速度 (建议 10-20)
    }

   
    
   
  
    public struct IsUserUnitTag:IComponentData,IEnableableComponent
    {}

   
}
