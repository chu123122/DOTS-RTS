using Unity.Entities;
using _RePlaySystem.Base; // 假设这是你的 LocalInstance 所在

// 本地回放版
[UpdateInGroup(typeof(SimulationSystemGroup))] // 运行在标准 Update 中
[UpdateAfter(typeof(UnitSpatialPartitionSystem))] 
public partial class LocalUnitFlowMovementSystem : BaseFlowMovementSystem
{
    protected override void OnCreate()
    {
        base.OnCreate();
        // 【互斥锁】只有在存在本地回放实例时才运行
        RequireForUpdate<LocalInstance>(); 
        
        // 如果你需要屏蔽掉网络版，确保 LocalInstance 和 NetworkStreamInGame 不会同时存在
        // 或者在网络版里加一个 RequireForUpdate<NetworkStreamInGame>()
    }
}