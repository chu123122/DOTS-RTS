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
                bool hasSelectedUnit = false;
                foreach (var unitSelected in
                         SystemAPI.Query<RefRO<UnitSelected>>())
                {
                    if (unitSelected.ValueRO.Value)
                    {
                        hasSelectedUnit = true;
                        break;
                    }
                }

                if (!hasSelectedUnit)
                {
                    Debug.LogWarning("移动命令未发送：当前没有选中的单位。");
                    return;
                }

                Entity flowFieldEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
                float3 targetPosition = ray.GetPoint(enter);
                EntityManager.SetComponentData(flowFieldEntity, new MoveOrder
                {
                    TargetPosition = targetPosition
                });
                EntityManager.SetComponentEnabled<MoveOrder>(flowFieldEntity, true);

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
