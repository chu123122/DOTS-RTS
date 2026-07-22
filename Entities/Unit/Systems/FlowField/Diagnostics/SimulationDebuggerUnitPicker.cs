using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using RTS.Unit.Components;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Development-only middle mouse picker for the simulation debugger.
/// A short middle click selects the nearest UnitContactBody on the XZ ground plane;
/// middle-button drags remain available to camera controls.
/// </summary>
public sealed class SimulationDebuggerUnitPicker : MonoBehaviour
{
    [Header("中键选择")]
    public bool Enabled = true;
    [Range(0, 6)] public int MouseButton = 2;
    [Min(0.05f)] public float MaximumClickDuration = 0.35f;
    [Min(0f)] public float MaximumDragPixels = 9f;
    [Min(1f)] public float PixelPickRadius = 24f;
    [Min(1f)] public float MaxPickDepth = 150f;
    public bool ClearSelectionWhenNothingHit;
    public bool LogSelectionChanges;
    public Camera SelectionCamera;

    private bool _middlePressed;
    private Vector2 _pressPosition;
    private float _pressTime;

    private void Update()
    {
        if (!Enabled)
            return;

        if (Input.GetMouseButtonDown(MouseButton))
        {
            Vector2 position = Input.mousePosition;
            if (SimulationDebuggerPanel.IsPointerOverDebugger(position))
            {
                _middlePressed = false;
                return;
            }

            _middlePressed = true;
            _pressPosition = position;
            _pressTime = Time.unscaledTime;
        }

        if (!_middlePressed || !Input.GetMouseButtonUp(MouseButton))
            return;

        _middlePressed = false;
        Vector2 releasePosition = Input.mousePosition;
        if (SimulationDebuggerPanel.IsPointerOverDebugger(releasePosition))
            return;
        if (Time.unscaledTime - _pressTime > MaximumClickDuration)
            return;
        if ((releasePosition - _pressPosition).sqrMagnitude >
            MaximumDragPixels * MaximumDragPixels)
            return;

        if (TryPickUnit(releasePosition, out Entity selected))
        {
            SetSelection(selected);
        }
        else if (ClearSelectionWhenNothingHit)
        {
            SetSelection(Entity.Null);
        }
    }

    private bool TryPickUnit(Vector2 screenPosition, out Entity selected)
    {
        selected = Entity.Null;
        SimulationDebuggerCameraFollow follow = GetComponent<SimulationDebuggerCameraFollow>();
        Camera camera = SelectionCamera != null
            ? SelectionCamera
            : follow != null && follow.FollowCamera != null
                ? follow.FollowCamera
                : Camera.main;
        World world = World.DefaultGameObjectInjectionWorld;
        if (camera == null || world == null || !world.IsCreated)
            return false;

        float bestDistanceSq = PixelPickRadius * PixelPickRadius;
        float bestDepth = float.MaxValue;
        EntityManager entityManager = world.EntityManager;
        EntityQuery query = entityManager.CreateEntityQuery(
            ComponentType.ReadOnly<LocalTransform>(),
            ComponentType.ReadOnly<UnitContactBody>());
        try
        {
            NativeArray<Entity> entities = query.ToEntityArray(Allocator.Temp);
            NativeArray<LocalTransform> transforms =
                query.ToComponentDataArray<LocalTransform>(Allocator.Temp);
            try
            {
                for (int i = 0; i < entities.Length; i++)
                {
                    float3 position = transforms[i].Position;
                    Vector3 screen = camera.WorldToScreenPoint(
                        new Vector3(position.x, position.y, position.z));
                    if (screen.z <= 0f)
                        continue;

                    // WorldToScreenPoint 返回全屏像素坐标，兼容非全屏 viewport。
                    Vector2 projected = new Vector2(screen.x, screen.y);
                    if (!camera.pixelRect.Contains(projected))
                        continue;

                    float distanceSq = (projected - screenPosition).sqrMagnitude;
                    if (distanceSq > bestDistanceSq)
                        continue;
                    if (Mathf.Approximately(distanceSq, bestDistanceSq) &&
                        screen.z >= bestDepth)
                        continue;

                    bestDistanceSq = distanceSq;
                    bestDepth = screen.z;
                    selected = entities[i];
                }
            }
            finally
            {
                transforms.Dispose();
                entities.Dispose();
            }
        }
        finally
        {
            query.Dispose();
        }

        return selected != Entity.Null;
    }

    private void SetSelection(Entity selected)
    {
        SimulationDebuggerRuntime.SelectedEntity = selected;

        // Keep compatibility with the original Stage 3 diagnostic selection path.
        // The runtime bridge remains sufficient when the singleton is not present.
        World world = World.DefaultGameObjectInjectionWorld;
        if (world != null && world.IsCreated)
        {
            EntityManager entityManager = world.EntityManager;
            EntityQuery query = entityManager.CreateEntityQuery(
                ComponentType.ReadWrite<Stage3ContactDiagnosticSelection>());
            try
            {
                if (query.CalculateEntityCount() > 0)
                {
                    NativeArray<Entity> selectionEntities =
                        query.ToEntityArray(Allocator.Temp);
                    try
                    {
                        Entity selectionEntity = selectionEntities[0];
                        Stage3ContactDiagnosticSelection value =
                            entityManager.GetComponentData<Stage3ContactDiagnosticSelection>(
                                selectionEntity);
                        value.SelectedEntity = selected;
                        entityManager.SetComponentData(selectionEntity, value);
                    }
                    finally
                    {
                        selectionEntities.Dispose();
                    }
                }
            }
            finally
            {
                query.Dispose();
            }
        }

        if (LogSelectionChanges)
        {
            Debug.Log(selected == Entity.Null
                ? "[Simulation Debugger] 已清除单位选择。"
                : $"[Simulation Debugger] 已选择 Entity {selected.Index}:{selected.Version}。详细数据将在下一时间步快照中显示。");
        }
    }
}
}
