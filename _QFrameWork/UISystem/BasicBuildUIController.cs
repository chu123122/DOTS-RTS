using _RePlaySystem.Base;
using DefaultNamespace;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using QFramework;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace Test
{
    public class BasicBuildUIController : MonoBehaviour, ICanGetServiceSystem, ICanSendEvent
    {
        public Button createUnitButton;
        public Button create50UnitButton;

        private int _unitSpawnSequence;

        private void Awake()
        {
            createUnitButton.onClick.AddListener(CreateUnit);
            create50UnitButton.onClick.AddListener(() =>
            {
                for (int i = 0; i < 20; i++)
                {
                    CreateUnit();
                }
            });
        }

        private void CreateUnit()
        {
            // 在客户端命令源头生成一次确定性微小偏移。
            // 实时 RPC 和回放记录共用同一个最终位置，避免两条路径重复或遗漏偏移。
            float spawnOffset = _unitSpawnSequence % 10 * 0.01f;
            _unitSpawnSequence++;
            float3 position = new float3(spawnOffset, 0.5f, 0);

            var clientHelpSystem=this.GetService<ClientHelpSystem>();
            clientHelpSystem.SendSpawnCreateEntityRpc(new CreateBaseUnitRpc(position));
            var world = World.DefaultGameObjectInjectionWorld;
        
            // 2. 获取我们写好的 "发信系统"
            var rpcSystem = world.GetExistingSystemManaged<RequestCommandRpcSystem>();
        
            // 3. 【关键】通过这个入口发送 "Create" 指令
            // 这样它既会发给服务器生成单位，也会自动被录制到 ReplayBuffer 里
            rpcSystem.SendInputCommand(
                InputCommandType.Create, // 告诉它是创建指令
                position            // 告诉它在哪创建
            );
            
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}
