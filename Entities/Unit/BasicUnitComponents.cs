using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

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

    public struct UnitSelected : IComponentData
    {
        public bool Value;
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

    /// <summary>
    /// 当前本地移动订单为单位分配的固定槽位。只在新订单到来时改写。
    /// </summary>
    public struct UnitMoveDestination : IComponentData
    {
        public float3 Position;
        public float ArrivalRadius;
        public int DirectApproachIntegrationDistance;
        public uint OrderVersion;
        public byte IsActive;
    }

    public struct UnitMovementSettings : IComponentData
    {
        public float MaxForce; // 转向力的最大值 (建议 20-50)
        public float RotationSpeed; // 转身速度 (建议 10-20)
    }

    /// <summary>
    /// 自定义单位接触求解质量。Unity Physics 中的单位保持 Kinematic，
    /// XPBD 接触只读取这里的逆质量。
    /// </summary>
    public struct UnitContactBody : IComponentData
    {
        public float InverseMass;
    }

   
    
   
  
    public struct IsUserUnitTag:IComponentData,IEnableableComponent
    {}

   
}
