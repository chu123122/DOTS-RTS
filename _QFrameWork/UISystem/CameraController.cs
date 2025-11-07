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
        
        private Camera _camera;
        private Vector3 _startPosition;
        private Vector3 _currentPosition;
        private Vector3 _newPosition;

        private void Awake()
        {
            _camera = Camera.main;
            _newPosition = transform.position;
        }

        private void Update()
        {
            HandleMouseInput();
            transform.position = Vector3.Lerp(transform.position, _newPosition, Time.deltaTime * movementSensitivity);
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
 
                    _newPosition = transform.position + _startPosition - _currentPosition;
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