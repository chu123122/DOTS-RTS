using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.EventSystems;
using 通用; // 你的命名空间

public class RTSSelectionManager : MonoBehaviour
{
    [Header("UI References")]
    public RectTransform selectionBoxRect; // 拖进去那个绿色的 Image
    public Canvas mainCanvas; // 拖进去你的主 Canvas

    [Header("Settings")]
    public float minSelectionSize = 10f; // 防止误触

    private Vector2 startMousePos;
    private bool isDragging;
    private Camera mainCam;
    private EntityManager entityManager;

    void Start()
    {
        mainCam = Camera.main;
        // 获取默认的世界 (World)
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        
        if(selectionBoxRect != null)
            selectionBoxRect.gameObject.SetActive(false);
    }

    void Update()
    {
        // 1. 鼠标按下：开始框选
        if (Input.GetMouseButtonDown(0))
        {
            if (EventSystem.current.IsPointerOverGameObject()) return;
            startMousePos = Input.mousePosition;
            isDragging = true;
            if(selectionBoxRect != null)
                selectionBoxRect.gameObject.SetActive(true);
        }

        // 2. 鼠标按住：更新 UI 框
        if (Input.GetMouseButton(0) && isDragging)
        {
            UpdateSelectionBoxVisual();
        }

        // 3. 鼠标松开：结算选中
        if (Input.GetMouseButtonUp(0) && isDragging)
        {
            isDragging = false;
            if(selectionBoxRect != null)
                selectionBoxRect.gameObject.SetActive(false);

            // 只有拖拽距离足够大才算框选，否则算点击
            if (Vector2.Distance(startMousePos, Input.mousePosition) > minSelectionSize)
            {
                SelectUnitsInScreenRect(startMousePos, Input.mousePosition);
            }
            else
            {
                // 这里可以是"单选"逻辑，或者交给你的 Raycast 系统
            }
        }
    }

    private void UpdateSelectionBoxVisual()
    {
        Vector2 currentMousePos = Input.mousePosition;
        
        float width = currentMousePos.x - startMousePos.x;
        float height = currentMousePos.y - startMousePos.y;

        selectionBoxRect.sizeDelta = new Vector2(Mathf.Abs(width), Mathf.Abs(height));
        
        // 处理反向拖拽的情况 (锚点位置)
        float x = width > 0 ? startMousePos.x : currentMousePos.x;
        float y = height > 0 ? startMousePos.y : currentMousePos.y;
        
        // 如果 Canvas 是 Screen Space Overlay，直接赋值
        // 如果是 Camera Space，可能需要转换，这里假设是 Overlay
        selectionBoxRect.position = new Vector2(x, y);
    }

    private void SelectUnitsInScreenRect(Vector2 start, Vector2 end)
    {
        // 1. 构建屏幕空间的 Rect
        Vector2 min = Vector2.Min(start, end);
        Vector2 max = Vector2.Max(start, end);
        Rect selectionRect = Rect.MinMaxRect(min.x, min.y, max.x, max.y);

        // 2. 查询所有带有 LocalTransform 和 UnitSelected 的 Entity
        // 注意：这是在主线程直接操作 EntityManager，对于几百个单位完全没问题
        // 如果你有 10000 个单位，这里需要改成 Job
        
        EntityQuery query = entityManager.CreateEntityQuery(
            typeof(LocalTransform), 
            typeof(UnitSelected),
            typeof(BasicUnitTag) // 确保只选单位，不选子弹等
        );

        NativeArray<Entity> entities = query.ToEntityArray(Allocator.Temp);
        NativeArray<LocalTransform> transforms = query.ToComponentDataArray<LocalTransform>(Allocator.Temp);
        NativeArray<UnitSelected> selectedStates = query.ToComponentDataArray<UnitSelected>(Allocator.Temp);

        // 是否按住了 Shift 键 (加选)
        bool isAdditive = Input.GetKey(KeyCode.LeftShift);

        for (int i = 0; i < entities.Length; i++)
        {
            // 核心逻辑：把单位的世界坐标转为屏幕坐标
            Vector3 screenPos = mainCam.WorldToScreenPoint(transforms[i].Position);

            // 检查是否在框内
            bool isInside = selectionRect.Contains(screenPos);

            UnitSelected currentState = selectedStates[i];
            
            if (isInside)
            {
                currentState.Value = true;
            }
            else if (!isAdditive) // 如果没按 Shift 且不在框内，就取消选中
            {
                currentState.Value = false;
            }

            // 写回 ECS
            entityManager.SetComponentData(entities[i], currentState);
        }

        entities.Dispose();
        transforms.Dispose();
        selectedStates.Dispose();
        
        Debug.Log("框选完成，状态已更新 ECS");
    }
}