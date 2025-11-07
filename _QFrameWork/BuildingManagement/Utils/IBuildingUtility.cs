using UnityEngine;

namespace QFramework.BuildingManagement.Utils
{
    public interface IBuildingUtility : IUtility
    {
        // 设置建筑物的碰撞体和导航网格障碍物是否启用
        void SetDeflate(GameObject previewGameObject, bool isDeflate);

        // 设置建筑物材质为不透明状态
        void SetOpaque(Material material);

        // 设置建筑物材质为透明状态
        void SetTransparent(Material material);
    }
}