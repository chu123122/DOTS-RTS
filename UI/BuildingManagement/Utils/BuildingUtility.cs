using QFramework.BuildingManagement.Utils;
using UnityEngine;
using UnityEngine.AI;

namespace QFramework.Systems.BuildingSystem.Utility
{
    public class BuildingUtility : IBuildingUtility
    {
        // 透明度，用于建筑物的透明效果
        private const float ColorA = 0.5f;

        // 存储着色器属性 ID，用于修改材质的透明度和深度写入设置
        private static readonly int Surface = Shader.PropertyToID("_Surface");
        private static readonly int ZWrite = Shader.PropertyToID("_ZWrite");


        public void SetOpaque(Material material)
        {
            material.SetFloat(Surface, 0); // 不透明
            material.SetFloat(ZWrite, 1); // 启用深度写入
            material.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Geometry;
            material.DisableKeyword("_SURFACE_TYPE_TRANSPARENT");

            // 设置材质的透明度为 1（完全不透明）
            Color color = material.color;
            color.a = 1f;
            material.color = color;
        }


        public void SetTransparent(Material material)
        {
            Debug.Log("设置成透明");
            material.SetFloat(Surface, 1); // 透明
            material.SetFloat(ZWrite, 0); // 禁用深度写入
            material.renderQueue = 3000; // 透明渲染队列
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");

            // 设置透明度为指定的透明度值
            Color color = material.color;
            color.a = ColorA;
            material.color = color;
        }

        // 设置建筑物的碰撞体和导航网格障碍物是否启用
        public void SetDeflate(GameObject previewGameObject, bool isDeflate)
        {
            previewGameObject.GetComponent<Collider>().enabled = isDeflate;
            previewGameObject.GetComponent<NavMeshObstacle>().enabled = isDeflate;
        }
    }
}