using _RePlaySystem.Base;
using Unity.Entities;
using Unity.NetCode;
using Unity.Physics;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using 客户端;
using 通用;
using Math = System.Math;
using RaycastHit = Unity.Physics.RaycastHit;

[UpdateInGroup(typeof(GhostInputSystemGroup))]
public partial class UnitSelectedInputSystem : SystemBase
{
    private CollisionFilter _collisionFilter;
    private PlayerAction _playerAction;

    private bool TouchShift =>
        Math.Abs(_playerAction.UnitControl.MultiSelectWithShift.ReadValue<float>() - 1) < 0.001f;

    private Vector2 MousePosition =>
        _playerAction.UnitControl.MultiSelectWithMouse.ReadValue<Vector2>();

    protected override void OnCreate()
    {
        _collisionFilter = new CollisionFilter
        {
            BelongsTo = ~0u,
            CollidesWith = 1 << 1,
        };
    }
    protected override void OnStartRunning()
    {
        _playerAction = new PlayerAction();
        _playerAction.UnitControl.Enable();
        _playerAction.UnitControl.SelectUnit.performed += OnSelectUnitHandler;
        _playerAction.UnitControl.MultiSelectWithShift.performed += OnContinueSelectUnitHandler;
    }


    protected override void OnStopRunning()
    {
        _playerAction.UnitControl.SelectUnit.performed -= OnSelectUnitHandler;
        _playerAction.UnitControl.MultiSelectWithShift.performed -= OnContinueSelectUnitHandler;
        _playerAction.Disable();
    }
    private void OnSelectUnit()
    {
        var collisionWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
        var cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
        Camera mainCamera = EntityManager.GetComponentObject<MainCameraComponents>(cameraEntity).Value;
        
        var mousePosition = Input.mousePosition;
        mousePosition.z = 100f;
        var worldPosition = mainCamera.ScreenToWorldPoint(mousePosition);

        var selectionInput = new RaycastInput
        {
            Start = mainCamera.transform.position,
            End = worldPosition,
            Filter = _collisionFilter,
        };

        if (collisionWorld.CastRay(selectionInput, out RaycastHit closestHit))
        {
            if (EntityManager.HasComponent<BasicUnitTag>(closestHit.Entity))
            {
                EntityManager.SetComponentData(closestHit.Entity, new UnitSelected
                {
                    Value = true
                });
            }
        }
    }

    private void OnDeSelectAllUnit()
    {
        foreach (var (unitSelected, entity) in SystemAPI.Query<RefRW<UnitSelected>>()
                     .WithAll<GhostOwnerIsLocal>().WithEntityAccess())
        {
            unitSelected.ValueRW.Value = false;
        }
    }


    private void OnSelectUnitHandler(InputAction.CallbackContext obj)
    {
        if (SceneManager.GetActiveScene() != SceneManager.GetSceneByName("Main")) return;
        // 先取消所有选中
        if (!TouchShift) OnDeSelectAllUnit();
        // 再选中新单位
        OnSelectUnit();
    }

    private void OnContinueSelectUnitHandler(InputAction.CallbackContext obj)
    {
        if (SceneManager.GetActiveScene() != SceneManager.GetSceneByName("Main")) return;
        OnSelectUnit();
    }

  

    protected override void OnUpdate()
    {
    }
}