using QFramework.BuildingManagement.Utils;
using UnityEngine;
using UnityEngine.SceneManagement;
using Utils;

namespace QFramework.BuildingManagement.Commands
{
    public class SetBeCreatedBuildingCommand:AbstractCommand
    {
        private readonly GameObject _createBuilding;
        private IBuildingUtility _buildingUtility; 
        public SetBeCreatedBuildingCommand(GameObject createBuilding)
        {
            _createBuilding = createBuilding;
        }

        protected override void OnExecute()
        {
            DebugSystem.Log("设置预览建筑为透明", "Building");
            _buildingUtility = this.GetUtility<IBuildingUtility>();
            // 设置预览建筑为透明状态
            _buildingUtility.SetTransparent(_createBuilding.GetComponent<Renderer>().material);
            // 设置建筑物的碰撞体和导航网格障碍物状态（初始为不激活）
            // _buildingUtility.SetDeflate(_createBuilding, false);
        }
    }
}