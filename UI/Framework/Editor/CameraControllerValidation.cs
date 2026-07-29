using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Test.Editor
{
    public static class CameraControllerValidation
    {
        private static string ValidationRequestPath => Path.GetFullPath(Path.Combine(
            Application.dataPath,
            "../Temp/RunCameraControllerValidation"));

        [InitializeOnLoadMethod]
        private static void RunRequestedValidationAfterReload()
        {
            if (!File.Exists(ValidationRequestPath))
                return;

            File.Delete(ValidationRequestPath);
            EditorApplication.delayCall += Run;
        }

        [MenuItem("RTS/Validation/Camera Controller")]
        public static void Run()
        {
            ValidateEdgeScrollDirections();
            ValidateCenterAnchoredZoom();
            Debug.Log("CAMERA_CONTROLLER_VALIDATION_OK\n" +
                      "edge scroll: center=idle, edges=camera-relative\n" +
                      "zoom: center ground anchor preserved, height clamped");
        }

        private static void ValidateEdgeScrollDirections()
        {
            Vector2 screen = new Vector2(1920f, 1080f);
            Vector3 forward = Vector3.forward;
            Vector3 right = Vector3.right;

            Require(CameraController.CalculateEdgeScrollDirection(
                        new Vector2(960f, 540f), screen, 24f, forward, right) == Vector3.zero,
                "Screen center unexpectedly generated camera movement.");
            Require(Vector3.Distance(
                        CameraController.CalculateEdgeScrollDirection(
                            new Vector2(0f, 540f), screen, 24f, forward, right),
                        Vector3.left) <= 0.0001f,
                "Left edge did not move camera left.");
            Require(Vector3.Distance(
                        CameraController.CalculateEdgeScrollDirection(
                            new Vector2(1919f, 1079f), screen, 24f, forward, right),
                        new Vector3(1f, 0f, 1f).normalized) <= 0.0001f,
                "Top-right corner did not combine camera-relative axes.");
        }

        private static void ValidateCenterAnchoredZoom()
        {
            Vector3 cameraPosition = new Vector3(0f, 40f, -40f);
            Vector3 rayDirection = new Vector3(0f, -1f, 1f).normalized;
            Require(CameraController.TryCalculateCenterPreservingZoomDelta(
                    cameraPosition,
                    rayDirection,
                    0f,
                    1f,
                    5f,
                    10f,
                    60f,
                    out Vector3 zoomDelta),
                "Valid downward center ray did not produce a zoom delta.");

            Vector3 nextPosition = cameraPosition + zoomDelta;
            Vector3 beforeAnchor = GroundIntersection(cameraPosition, rayDirection, 0f);
            Vector3 afterAnchor = GroundIntersection(nextPosition, rayDirection, 0f);
            Require(Mathf.Abs(nextPosition.y - 35f) <= 0.0001f,
                "Zoom did not apply the configured height step.");
            Require(Vector3.Distance(beforeAnchor, afterAnchor) <= 0.0001f,
                "Zoom changed the screen-center ground anchor.");

            Require(CameraController.TryCalculateCenterPreservingZoomDelta(
                    cameraPosition,
                    rayDirection,
                    0f,
                    100f,
                    5f,
                    10f,
                    60f,
                    out Vector3 clampedDelta) &&
                    Mathf.Abs((cameraPosition + clampedDelta).y - 10f) <= 0.0001f,
                "Zoom did not clamp to the minimum camera height.");
        }

        private static Vector3 GroundIntersection(
            Vector3 origin,
            Vector3 direction,
            float groundHeight)
        {
            float distance = (groundHeight - origin.y) / direction.y;
            return origin + direction * distance;
        }

        private static void Require(bool condition, string message)
        {
            if (!condition)
                throw new InvalidOperationException(message);
        }
    }
}
