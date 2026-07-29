using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using _RePlaySystem.Base; // 引用 InputCommandType 和 RequestCommandRpcSystem

public class RTSUnitSpawner : MonoBehaviour
{
    [Header("Spawn Settings")]
    public float3 SpawnPosition = new float3(0, 0, 0); // 指定出生点
    public int PlayerId = 0;

    public void OnSpawnButtonClicked()
    {
        // 1. 获取 ECS 世界
        var world = World.DefaultGameObjectInjectionWorld;
        if (world == null) return;

        // 2. 获取 RequestCommandRpcSystem (客户端发信系统)
        var rpcSystem = world.GetExistingSystemManaged<RequestCommandRpcSystem>();
        if (rpcSystem == null)
        {
            Debug.LogError("找不到 RequestCommandRpcSystem，请确保它正在运行！");
            return;
        }

        // 3. 发送指令 (这会自动触发 SendInputCommand 里的录制逻辑)
        // 注意：这里用 InputCommandType.Create
        rpcSystem.SendInputCommand(InputCommandType.Create, SpawnPosition, PlayerId);
        
        Debug.Log("UI: 请求创建单位");
    }
}