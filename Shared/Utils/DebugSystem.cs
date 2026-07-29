using System.Collections.Generic;
using UnityEngine;

namespace Utils
{
    public static class DebugSystem
    {
        private const string CurrentDebugName = "乐酌";

        public static void Log(string message, string debugName, Color color = new Color())
        {
            if (!CurrentDebugName.Equals(debugName)) return;
            if (color != new Color()) message = $"<color={color.ToColorString()}>" + message + "</color>";
            Debug.Log(message);
        }

        public static void LogError(string message, string debugName, Color color = new Color())
        {
            if (!CurrentDebugName.Equals(debugName)) return;
            if (color != new Color()) message = $"<color={color.ToColorString()}>" + message + "</color>";
            Debug.LogError(message);
        }

        public static void LogWarning(string message, string debugName, Color color = new Color())
        {
            if (!CurrentDebugName.Equals(debugName)) return;
            if (color != new Color()) message = $"<color={color.ToColorString()}>" + message + "</color>";
            Debug.LogWarning(message);
        }
    }

    public static class DebugDrawing
    {
        /// <summary>
        /// 在 XY 平面上绘制线框圆
        /// </summary>
        /// <param name="center">圆心</param>
        /// <param name="radius">半径</param>
        /// <param name="color">颜色</param>
        /// <param name="segments">分段数，分段越多圆越平滑</param>
        public static void DrawWireCircleXY(Vector3 center, float radius, Color color, int segments = 32)
        {
            float angleStep = 360f / segments;
            Vector3 prevPoint = center + new Vector3(radius, 0f, 0f);
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * angleStep * Mathf.Deg2Rad;
                Vector3 nextPoint = center + new Vector3(Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius, 0f);
                Debug.DrawLine(prevPoint, nextPoint, color);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// 在 XZ 平面上绘制线框圆
        /// </summary>
        public static void DrawWireCircleXZ(Vector3 center, float radius, Color color, int segments = 32)
        {
            float angleStep = 360f / segments;
            Vector3 prevPoint = center + new Vector3(radius, 0f, 0f);
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * angleStep * Mathf.Deg2Rad;
                Vector3 nextPoint = center + new Vector3(Mathf.Cos(angle) * radius, 0f, Mathf.Sin(angle) * radius);
                Debug.DrawLine(prevPoint, nextPoint, color);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// 在 YZ 平面上绘制线框圆
        /// </summary>
        public static void DrawWireCircleYZ(Vector3 center, float radius, Color color, int segments = 32)
        {
            float angleStep = 360f / segments;
            Vector3 prevPoint = center + new Vector3(0f, radius, 0f);
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * angleStep * Mathf.Deg2Rad;
                // 在 YZ 平面，X 坐标保持不变
                Vector3 nextPoint = center + new Vector3(0f, Mathf.Cos(angle) * radius, Mathf.Sin(angle) * radius);
                Debug.DrawLine(prevPoint, nextPoint, color);
                prevPoint = nextPoint;
            }
        }

        /// <summary>
        /// 绘制一个线框球，方法是在三个正交平面（XY、XZ、YZ）上分别绘制一个线框圆
        /// </summary>
        /// <param name="center">球心</param>
        /// <param name="radius">球的半径</param>
        /// <param name="color">颜色</param>
        /// <param name="segments">每个圆的分段数</param>
        public static void DrawWireSphere(Vector3 center, float radius, Color color, int segments = 32)
        {
            // 在 XY 平面绘制
            DrawWireCircleXY(center, radius, color, segments);
            // 在 XZ 平面绘制
            DrawWireCircleXZ(center, radius, color, segments);
            // 在 YZ 平面绘制
            DrawWireCircleYZ(center, radius, color, segments);
        }
    }


    public static class ColorExtensions
    {
        private static readonly Dictionary<Color, string> ColorMap = new Dictionary<Color, string>
        {
            { Color.red, "red" },
            { Color.green, "green" },
            { Color.blue, "blue" },
            { Color.white, "white" },
            { Color.black, "black" },
            { Color.yellow, "yellow" },
        };

        public static string ToColorString(this Color color)
        {
            return ColorMap.GetValueOrDefault(color, "Unknown Color,请在ColorExtensions里添加对应映射");
        }
    }
}