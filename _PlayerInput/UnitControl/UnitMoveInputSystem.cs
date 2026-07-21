using _RePlaySystem.Base;
using DefaultNamespace;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.UIElements;
using 简单战斗.ServiceLocator;
using 通用;

namespace 客户端
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(GhostInputSystemGroup))]
    public partial class UnitMoveInputSystem : SystemBase, ICanGetServiceSystem,IGetService
    {
        private PlayerAction _playerAction;

        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldGlobalTarget>();
            RequireForUpdate<MoveOrderSelectionElement>();
        }

        protected override void OnUpdate()
        {
            if (Input.GetMouseButtonDown((int)MouseButton.RightMouse))
            {
                OnSelectUnitMovePosition();
            }
        }

        private void OnSelectUnitMovePosition()
        {
            Entity cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
            Camera mainCamera = EntityManager.GetComponentObject<MainCameraComponents>(cameraEntity).Value;
            if (mainCamera == null)
            {
                Debug.LogWarning("无法下达移动命令：Main Camera 不可用。");
                return;
            }

            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            var groundPlane = new Plane(Vector3.up, Vector3.zero);
            if (groundPlane.Raycast(ray, out float enter))
            {
                Entity flowFieldEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
                DynamicBuffer<MoveOrderSelectionElement> recipients =
                    EntityManager.GetBuffer<MoveOrderSelectionElement>(flowFieldEntity);
                recipients.Clear();
                foreach (var (unitSelected, entity) in
                         SystemAPI.Query<RefRO<UnitSelected>>()
                             .WithAll<BasicUnitTag, UnitMoveDestination>()
                             .WithEntityAccess())
                {
                    if (unitSelected.ValueRO.Value)
                        recipients.Add(new MoveOrderSelectionElement { Entity = entity });
                }

                if (recipients.Length == 0)
                {
                    Debug.LogWarning("移动命令未发送：当前没有选中的单位。");
                    return;
                }

                float3 targetPosition = ray.GetPoint(enter);
                EntityManager.SetComponentData(flowFieldEntity, new MoveOrder
                {
                    TargetPosition = targetPosition
                });
                EntityManager.SetComponentEnabled<MoveOrder>(flowFieldEntity, true);
                Debug.Log(
                    $"移动命令已发送：右键快照 {recipients.Length} 个单位，" +
                    $"目标 {targetPosition}。");

                RequestCommandRpcSystem requestCommandRpcSystem =
                    this.GetService<RequestCommandRpcSystem>();
                requestCommandRpcSystem.SendInputCommand(
                    InputCommandType.Move, 
                    targetPosition
                );
            }
            else
            {
                Debug.LogWarning("移动命令未发送：鼠标射线没有与地面平面相交。");
            }
        }
    }
}
