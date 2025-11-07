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
                var query = World.DefaultGameObjectInjectionWorld.
                    EntityManager.CreateEntityQuery(typeof(NetWorkDataContainer));
                int ghostId = query.GetSingleton<NetWorkDataContainer>().Id;
                float3 position = new float3(0, 0.5f, 0);
                var clientHelpSystem=this.GetService<ClientHelpSystem>();
                clientHelpSystem.SendSpawnCreateEntityRpc(new CreateBaseUnitRpc(position));
                
                List<int> ghostIds = new List<int>() { ghostId };
                
                RequestCommandRpcSystem commandRpcSystem = this.GetService<RequestCommandRpcSystem>();

                PlayerInputCommand command =
                    commandRpcSystem.CreateInputCommand(InputCommandType.Create, position, ghostIds);
                commandRpcSystem.SendInputCommand(command);
                
                StartCoroutine(DelayedEntityQuery(ghostIds));
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