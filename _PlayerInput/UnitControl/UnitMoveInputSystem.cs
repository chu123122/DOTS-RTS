using System.Collections.Generic;
using _RePlaySystem.Base;
using DefaultNamespace;
using Unity.Entities;
using Unity.NetCode;
using Unity.Physics;
using UnityEngine;
using UnityEngine.UIElements;
using 简单战斗.ServiceLocator;
using 通用;
using RaycastHit = Unity.Physics.RaycastHit;

namespace 客户端
{
    [UpdateInGroup(typeof(GhostInputSystemGroup))]
    public partial class UnitMoveInputSystem : SystemBase, ICanGetServiceSystem,IGetService
    {
        private PlayerAction _playerAction;
        private CollisionFilter _collisionFilter;

        protected override void OnCreate()
        {
            RequireForUpdate<NetworkStreamInGame>();
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
                List<int> ghostIds = new List<int>();
                foreach (var (targetPosition, unitSelected, ghostInstance) in
                         SystemAPI.Query<RefRW<UnitMoveTargetPosition>
                             , RefRO<UnitSelected>,
                             RefRO<GhostInstance>>().WithAll<GhostOwnerIsLocal>())
                {
                    if (unitSelected.ValueRO.Value)
                    {
                        targetPosition.ValueRW.Value = closestHit.Position;
                        targetPosition.ValueRW.GetMoveInput = true;
                        ghostIds.Add(ghostInstance.ValueRO.ghostId);
                    }
                }
                if(ghostIds.Count<=0)return;//未实际获取到选中单位
                
                RequestCommandRpcSystem requestCommandRpcSystem =
                    this.GetService<RequestCommandRpcSystem>();
                requestCommandRpcSystem.SendInputCommand(
                    requestCommandRpcSystem.CreateInputCommand(InputCommandType.Move, closestHit.Position, ghostIds));
            }
        }
    }
}