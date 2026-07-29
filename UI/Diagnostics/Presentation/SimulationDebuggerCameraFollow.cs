using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Test;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// 选中单位后自动跟随 + 时间减缓。
/// 接管 CameraController 的控制权，跟随模式下仍支持边缘滚动和缩放。
/// 中键点击空地取消选中后自动退出跟随并恢复原始时间流速。
/// </summary>
public sealed class SimulationDebuggerCameraFollow : MonoBehaviour
{
    [Header("跟随")]
    [Tooltip("被接管的摄像机控制器。留空则自动从场景查找。")]
    public CameraController CameraController;
    [Min(1f)] public float FollowHeight = 14f;
    [Min(0.01f)] public float FollowSmoothTime = 0.15f;

    [Header("时间减缓")]
    public bool EnableTimeSlow = true;

    [Header("边缘滚动 (跟随模式下)")]
    [Min(0f)] public float EdgeScrollSpeed = 20f;
    [Range(1f, 128f)] public float EdgeScrollBorder = 24f;

    [Header("缩放 (跟随模式下)")]
    [Min(0f)] public float ZoomSensitivity = 5f;
    public float MinHeight = 5f;
    public float MaxHeight = 30f;

    private bool _isFollowing;
    private Vector3 _followOffset;
    private Vector3 _smoothVelocity;
    private float _savedTimeScale = 1f;
    private Camera _controlledCamera;
    private Transform _cameraTransform;

    public Camera FollowCamera => ControlledCamera;

    private Camera ControlledCamera
    {
        get
        {
            if (_controlledCamera == null && CameraController != null)
                _controlledCamera = CameraController.GetComponentInChildren<Camera>(true);
            if (_controlledCamera == null)
                _controlledCamera = Camera.main;
            return _controlledCamera;
        }
    }

    private void OnEnable()
    {
        if (CameraController == null)
            CameraController = FindFirstObjectByType<CameraController>();
        if (CameraController != null)
            _cameraTransform = CameraController.transform;
    }

    private void LateUpdate()
    {
        if (CameraController == null)
        {
            CameraController = FindFirstObjectByType<CameraController>();
            if (CameraController != null)
                _cameraTransform = CameraController.transform;
        }

        Entity selected = SimulationDebuggerRuntime.SelectedEntity;
        bool shouldFollow = selected != Entity.Null;

        if (shouldFollow && !_isFollowing)
        {
            BeginFollow();
        }
        else if (!shouldFollow && _isFollowing)
        {
            EndFollow();
        }

        if (_isFollowing)
            UpdateFollow();
    }

    private void OnDisable()
    {
        if (_isFollowing)
            EndFollow();
    }

    private void OnApplicationQuit()
    {
        if (_isFollowing)
            EndFollow();
    }

    private void BeginFollow()
    {
        if (_cameraTransform == null)
            return;

        Vector3 unitPos = GetSelectedUnitPosition();
        if (float.IsNaN(unitPos.x))
            return;

        _savedTimeScale = Time.timeScale;
        if (EnableTimeSlow)
            Time.timeScale = SimulationDebuggerRuntime.SlowTimeScale;

        if (CameraController != null)
            CameraController.enabled = false;

        _followOffset = Vector3.zero;
        _smoothVelocity = Vector3.zero;

        Vector3 target = CalculateRigTarget(unitPos, FollowHeight);
        _cameraTransform.position = target;
        _isFollowing = true;
    }

    private void EndFollow()
    {
        Time.timeScale = _savedTimeScale;

        if (CameraController != null)
            CameraController.enabled = true;

        _isFollowing = false;
    }

    private void UpdateFollow()
    {
        float deltaTime = Time.unscaledDeltaTime;
        if (deltaTime <= 0f || _cameraTransform == null)
            return;

        Vector3 unitPos = GetSelectedUnitPosition();
        if (float.IsNaN(unitPos.x))
        {
            SimulationDebuggerRuntime.SelectedEntity = Entity.Null;
            return;
        }

        HandleEdgeScroll(deltaTime);
        HandleZoomInput();

        if (EnableTimeSlow)
            Time.timeScale = SimulationDebuggerRuntime.SlowTimeScale;

        float effectiveHeight = FollowHeight + _followOffset.y;
        Vector3 target = CalculateRigTarget(unitPos, effectiveHeight)
                       + new Vector3(_followOffset.x, 0f, _followOffset.z);

        _cameraTransform.position = Vector3.SmoothDamp(
            _cameraTransform.position,
            target,
            ref _smoothVelocity,
            FollowSmoothTime,
            Mathf.Infinity,
            deltaTime);
    }

    private Vector3 CalculateRigTarget(Vector3 unitPos, float height)
    {
        Camera camera = ControlledCamera;
        if (camera == null || _cameraTransform == null)
            return unitPos + Vector3.up * height;

        Vector3 forward = camera.transform.forward;
        float fy = forward.y;
        if (Mathf.Abs(fy) < 0.001f)
            return unitPos + Vector3.up * height;

        // 求子 Camera 应处世界坐标，扣相对 Rig 偏移；旧实现写到 Rig 导致固定偏移。
        float t = height / fy;
        Vector3 desiredCameraPosition = new Vector3(
            unitPos.x + forward.x * t,
            unitPos.y + height,
            unitPos.z + forward.z * t);
        Vector3 cameraOffsetFromRig = camera.transform.position - _cameraTransform.position;
        return desiredCameraPosition - cameraOffsetFromRig;
    }

    private void HandleEdgeScroll(float deltaTime)
    {
        if (!Application.isFocused)
            return;
        if (SimulationDebuggerPanel.IsPointerOverDebugger(Input.mousePosition))
            return;

        Camera camera = ControlledCamera;
        if (camera == null)
            return;

        Vector3 direction = CameraController.CalculateEdgeScrollDirection(
            Input.mousePosition,
            new Vector2(Screen.width, Screen.height),
            EdgeScrollBorder,
            camera.transform.forward,
            camera.transform.right);

        if (direction.sqrMagnitude <= 0.000001f)
            return;

        _followOffset += direction * (EdgeScrollSpeed * deltaTime);
    }

    private void HandleZoomInput()
    {
        if (SimulationDebuggerPanel.IsPointerOverDebugger(Input.mousePosition))
            return;

        float scrollDelta = Input.mouseScrollDelta.y;
        if (Mathf.Approximately(scrollDelta, 0f))
            return;

        _followOffset.y -= scrollDelta * ZoomSensitivity;
        _followOffset.y = Mathf.Clamp(
            _followOffset.y,
            MinHeight - FollowHeight,
            MaxHeight - FollowHeight);
    }

    private static Vector3 GetSelectedUnitPosition()
    {
        Entity selected = SimulationDebuggerRuntime.SelectedEntity;
        if (selected == Entity.Null)
            return new Vector3(float.NaN, 0f, 0f);

        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
            return new Vector3(float.NaN, 0f, 0f);

        EntityManager em = world.EntityManager;
        if (!em.Exists(selected) || !em.HasComponent<LocalTransform>(selected))
            return new Vector3(float.NaN, 0f, 0f);

        float3 pos = em.GetComponentData<LocalTransform>(selected).Position;
        return new Vector3(pos.x, pos.y, pos.z);
    }
}
}
