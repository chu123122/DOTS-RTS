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

    /// <summary>
    /// Crowd Physics 独立形状。初始化/形状同步阶段写入，物理 step 只读。
    /// </summary>
    public struct CrowdDiscShape : IComponentData
    {
        public float Radius;
        public uint Version;
    }

    /// <summary>
    /// Unity Physics 中保留的单位 query proxy；不参与 Crowd locomotion 响应。
    /// CrowdStepVersion 是 ECS Transform 的提交版本，ProxyVersion 是最近一次
    /// BuildPhysicsWorld 已消费并发布的版本。
    /// </summary>
    public struct CrowdQueryProxy : IComponentData
    {
        public uint CrowdStepVersion;
        public uint ProxyVersion;
    }

   
    
   
  
    public struct IsUserUnitTag:IComponentData,IEnableableComponent
    {}

   
}
