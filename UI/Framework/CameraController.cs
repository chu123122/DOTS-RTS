using RTS.Unit.FlowField.Diagnostics;
using UnityEngine;
using UnityEngine.EventSystems;

namespace Test
{
    /// <summary>
    /// RTS 摄像机控制：屏幕边缘平移，并以屏幕中心落点为锚点缩放。
    /// </summary>
    public class CameraController : MonoBehaviour
    {
        [Header("Edge Scroll")]
        [Min(0f)] public float edgeScrollSpeed = 25f;
        [Range(1f, 128f)] public float edgeScrollBorder = 24f;
        [Min(1f)] public float zoomedOutSpeedMultiplier = 1.8f;
        [Min(0.001f)] public float positionSmoothTime = 0.08f;
        public bool blockEdgeScrollOverUi = true;

        [Header("Center Anchored Zoom")]
        [Min(0f)] public float zoomSensitivity = 5f;
        public float minCameraHeight = 10f;
        public float maxCameraHeight = 60f;
        public float groundHeight;

        [SerializeField] private Camera controlledCamera;

        private Vector3 _targetPosition;
        private Vector3 _smoothVelocity;

        private void Awake()
        {
            if (controlledCamera == null)
                controlledCamera = GetComponentInChildren<Camera>(true);
            if (controlledCamera == null)
                controlledCamera = Camera.main;

            if (controlledCamera == null)
            {
                Debug.LogError("CameraController 找不到可控制的 Camera。", this);
                enabled = false;
                return;
            }

            _targetPosition = transform.position;
        }

        private void Update()
        {
            float deltaTime = Time.unscaledDeltaTime;
            if (deltaTime <= 0f)
                return;

            HandleEdgeScroll(deltaTime);
            HandleZoomInput();
            transform.position = Vector3.SmoothDamp(
                transform.position,
                _targetPosition,
                ref _smoothVelocity,
                positionSmoothTime,
                Mathf.Infinity,
                deltaTime);
        }

        private void OnDisable()
        {
            _smoothVelocity = Vector3.zero;
        }

        private void OnValidate()
        {
            edgeScrollSpeed = Mathf.Max(0f, edgeScrollSpeed);
            edgeScrollBorder = Mathf.Max(1f, edgeScrollBorder);
            zoomedOutSpeedMultiplier = Mathf.Max(1f, zoomedOutSpeedMultiplier);
            positionSmoothTime = Mathf.Max(0.001f, positionSmoothTime);
            zoomSensitivity = Mathf.Max(0f, zoomSensitivity);
            if (maxCameraHeight < minCameraHeight)
                maxCameraHeight = minCameraHeight;
        }

        private void HandleEdgeScroll(float deltaTime)
        {
            if (!Application.isFocused)
                return;
            if (SimulationDebuggerPanel.IsPointerOverDebugger(Input.mousePosition))
                return;
            if (blockEdgeScrollOverUi &&
                EventSystem.current != null &&
                EventSystem.current.IsPointerOverGameObject())
                return;

            Vector3 direction = CalculateEdgeScrollDirection(
                Input.mousePosition,
                new Vector2(Screen.width, Screen.height),
                edgeScrollBorder,
                controlledCamera.transform.forward,
                controlledCamera.transform.right);
            if (direction.sqrMagnitude <= 0.000001f)
                return;

            float heightRatio = Mathf.InverseLerp(
                minCameraHeight,
                maxCameraHeight,
                GetTargetCameraPosition().y);
            float speedScale = Mathf.Lerp(1f, zoomedOutSpeedMultiplier, heightRatio);
            _targetPosition += direction * (edgeScrollSpeed * speedScale * deltaTime);
        }

        private void HandleZoomInput()
        {
            if (SimulationDebuggerPanel.IsPointerOverDebugger(Input.mousePosition))
                return;
            float scrollDelta = Input.mouseScrollDelta.y;
            if (Mathf.Approximately(scrollDelta, 0f))
                return;

            Vector3 pendingParentDelta = _targetPosition - transform.position;
            Vector3 targetCameraPosition = controlledCamera.transform.position + pendingParentDelta;
            Ray centerRay = controlledCamera.ViewportPointToRay(new Vector3(0.5f, 0.5f));
            if (TryCalculateCenterPreservingZoomDelta(
                    targetCameraPosition,
                    centerRay.direction,
                    groundHeight,
                    scrollDelta,
                    zoomSensitivity,
                    minCameraHeight,
                    maxCameraHeight,
                    out Vector3 cameraDelta))
            {
                _targetPosition += cameraDelta;
            }
        }

        private Vector3 GetTargetCameraPosition()
        {
            return controlledCamera.transform.position + (_targetPosition - transform.position);
        }

        public static Vector3 CalculateEdgeScrollDirection(
            Vector2 mousePosition,
            Vector2 screenSize,
            float border,
            Vector3 cameraForward,
            Vector3 cameraRight)
        {
            if (screenSize.x <= 0f || screenSize.y <= 0f ||
                mousePosition.x < 0f || mousePosition.y < 0f ||
                mousePosition.x > screenSize.x || mousePosition.y > screenSize.y)
                return Vector3.zero;

            Vector2 input = Vector2.zero;
            border = Mathf.Max(1f, border);
            if (mousePosition.x <= border)
                input.x -= 1f;
            else if (mousePosition.x >= screenSize.x - border)
                input.x += 1f;
            if (mousePosition.y <= border)
                input.y -= 1f;
            else if (mousePosition.y >= screenSize.y - border)
                input.y += 1f;
            if (input.sqrMagnitude <= 0f)
                return Vector3.zero;

            cameraForward.y = 0f;
            cameraRight.y = 0f;
            cameraForward.Normalize();
            cameraRight.Normalize();
            return (cameraRight * input.x + cameraForward * input.y).normalized;
        }

        public static bool TryCalculateCenterPreservingZoomDelta(
            Vector3 cameraPosition,
            Vector3 centerRayDirection,
            float groundHeight,
            float scrollDelta,
            float zoomSensitivity,
            float minCameraHeight,
            float maxCameraHeight,
            out Vector3 cameraDelta)
        {
            cameraDelta = Vector3.zero;
            if (Mathf.Abs(centerRayDirection.y) <= 0.000001f)
                return false;

            centerRayDirection.Normalize();
            float currentDistance =
                (groundHeight - cameraPosition.y) / centerRayDirection.y;
            if (currentDistance <= 0f)
                return false;

            float desiredHeight = Mathf.Clamp(
                cameraPosition.y - scrollDelta * Mathf.Max(0f, zoomSensitivity),
                minCameraHeight,
                Mathf.Max(minCameraHeight, maxCameraHeight));
            float desiredDistance =
                (groundHeight - desiredHeight) / centerRayDirection.y;
            if (desiredDistance <= 0f)
                return false;

            Vector3 centerAnchor = cameraPosition + centerRayDirection * currentDistance;
            Vector3 desiredCameraPosition =
                centerAnchor - centerRayDirection * desiredDistance;
            cameraDelta = desiredCameraPosition - cameraPosition;
            return true;
        }
    }
}
