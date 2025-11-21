using Unity.Entities;
using Unity.NetCode;

// 正常的网络版
[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
[UpdateAfter(typeof(UnitSpatialPartitionSystem))]
public partial class NetCodeUnitFlowMovementSystem : BaseFlowMovementSystem
{
    protected override void OnCreate()
    {
        base.OnCreate();
        // 【互斥锁】只有在连接了网络，且有游戏流时才运行
        RequireForUpdate<NetworkStreamInGame>();
    }
}