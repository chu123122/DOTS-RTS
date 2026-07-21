using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;

namespace Test
{
    /// <summary>
    /// 相机控制
    /// </summary>
    public class CameraController : MonoBehaviour
    {
        [Range(0,2)]
        public float movementSensitivity = 1f;

        [Min(0f)]
        public float zoomSensitivity = 5f;

        public float minCameraHeight = 10f;
        public float maxCameraHeight = 60f;
        
        private Camera _camera;
        private float _cameraHeightOffset;
        private Vector3 _startPosition;
        private Vector3 _currentPosition;
        private Vector3 _newPosition;

        private void Awake()
        {
            _camera = Camera.main;
            _cameraHeightOffset = _camera.transform.position.y - transform.position.y;
            _newPosition = transform.position;
        }

        private void Update()
        {
            HandleMouseInput();
            HandleZoomInput();
            transform.position = Vector3.Lerp(transform.position, _newPosition, Time.deltaTime * movementSensitivity);
        }

        /// <summary>
        /// 通过移动父节点调整实际摄像机的世界高度。
        /// </summary>
        private void HandleZoomInput()
        {
            float scrollDelta = Input.mouseScrollDelta.y;
            if (Mathf.Approximately(scrollDelta, 0f))
            {
                return;
            }

            float targetCameraHeight = Mathf.Clamp(
                _newPosition.y + _cameraHeightOffset - scrollDelta * zoomSensitivity,
                minCameraHeight,
                maxCameraHeight);
            _newPosition.y = targetCameraHeight - _cameraHeightOffset;
        }

        /// <summary>
        /// 处理鼠标的输入（旧版输入系统）
        /// </summary>
        private void HandleMouseInput()
        {
            if (Input.GetMouseButtonDown((int)MouseButton.Right)&&!EventSystem.current.IsPointerOverGameObject())
            {
                Plane plane = new Plane(Vector3.up, Vector3.zero);
                Ray ray = _camera.ScreenPointToRay(Input.mousePosition);

                if (plane.Raycast(ray, out float entry))
                {
                    _startPosition=ray.GetPoint(entry);
                }
            }

            if (Input.GetMouseButton((int)MouseButton.Right)&&!EventSystem.current.IsPointerOverGameObject())
            {
                Plane plane = new Plane(Vector3.up, Vector3.zero);
                Ray ray =_camera.ScreenPointToRay(Input.mousePosition);

                if (plane.Raycast(ray, out var entry))
                {
                    _currentPosition = ray.GetPoint(entry);

                    Vector3 draggedPosition = transform.position + _startPosition - _currentPosition;
                    draggedPosition.y = _newPosition.y;
                    _newPosition = draggedPosition;
                }
            }

            if (Input.GetMouseButtonUp((int)MouseButton.Right))
            {
                _startPosition = Vector2.zero;
                _currentPosition = Vector2.zero;
            }
        }
    }
}
