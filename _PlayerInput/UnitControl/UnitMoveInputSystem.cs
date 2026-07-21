using _RePlaySystem.Base;
using DefaultNamespace;
using Unity.Entities;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using Unity.NetCode;
using Unity.Physics;
using UnityEngine;
using UnityEngine.UIElements;
using 简单战斗.ServiceLocator;
using 通用;
using RaycastHit = Unity.Physics.RaycastHit;

namespace 客户端
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(GhostInputSystemGroup))]
    public partial class UnitMoveInputSystem : SystemBase, ICanGetServiceSystem,IGetService
    {
        private PlayerAction _playerAction;
        private CollisionFilter _collisionFilter;

        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldGlobalTarget>();
            _collisionFilter = new CollisionFilter
            {
                BelongsTo = ~0u,
                CollidesWith = 1 << 0,
            };
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
            CollisionWorld collisionWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
            Entity cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
            Camera mainCamera = EntityManager.GetComponentObject<MainCameraComponents>(cameraEntity).Value;

            Vector3 mousePosition = Input.mousePosition;
            mousePosition.z = 100f;
            if (mainCamera == null) Debug.LogWarning("camera is null");
            Vector3 worldPosition = mainCamera.ScreenToWorldPoint(mousePosition);

            RaycastInput selectionInput = new RaycastInput
            {
                Start = mainCamera.transform.position,
                End = worldPosition,
                Filter = _collisionFilter,
            };

            if (collisionWorld.CastRay(selectionInput, out RaycastHit closestHit))
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

                if (!hasSelectedUnit) return;

                Entity flowFieldEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
                EntityManager.SetComponentData(flowFieldEntity, new MoveOrder
                {
                    TargetPosition = closestHit.Position
                });
                EntityManager.SetComponentEnabled<MoveOrder>(flowFieldEntity, true);

                RequestCommandRpcSystem requestCommandRpcSystem =
                    this.GetService<RequestCommandRpcSystem>();
                requestCommandRpcSystem.SendInputCommand(
                    InputCommandType.Move, 
                    closestHit.Position
                );
            }
        }
    }
}
