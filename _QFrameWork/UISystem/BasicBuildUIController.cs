using System.Collections;
using System.Collections.Generic;
using _RePlaySystem.Base;
using DefaultNamespace;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using QFramework;
using UI.MapUI;
using UI.MapUI.Events;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace Test
{
    public class BasicBuildUIController : MonoBehaviour, ICanGetServiceSystem, ICanSendEvent
    {
        public Button createUnitButton;

        private void Awake()
        {
            createUnitButton.onClick.AddListener(() =>
            {
                float3 position = new float3(0, 0.5f, 0);
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
            });
        }

        private IEnumerator DelayedEntityQuery(List<int> ghostIds)
        {
            yield return new WaitForSeconds(2f);//TODO: 需要换种可靠的解决方案
            foreach (var id in ghostIds)
            {
                Entity entity = this.GetService<ClientHelpSystem>().GetEntityByIndexInClientWorld(id);
                this.SendEvent(new CreateEntityEvent()
                {
                    IsEnemy = false,
                    Type = MapUIEntityType.Solider,
                    Entity = entity
                });
            }
        }


        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}