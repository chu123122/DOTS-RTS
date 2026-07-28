using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using 通用;

namespace RTS.Unit.Components
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
    /// 单位是否已进入当前流场目标的到达区域。
    /// 跨帧保留，给进出到达区加滞回，避免在边界反复启停。
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
        public float MaxForce; // 转向力上限
        public float RotationSpeed; // 转身速度
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
