using QFramework.BuildingManagement.Utils;
using UnityEngine;

namespace QFramework.BuildingManagement.Commands
{
    public class SetHasCreatedBuildingCommand:AbstractCommand
    {
        private readonly GameObject _createBuilding;
        private IBuildingUtility _buildingUtility; 
        public SetHasCreatedBuildingCommand(GameObject createBuilding)
        {
            _createBuilding = createBuilding;
        }
        protected override void OnExecute() {
            _buildingUtility = this.GetUtility<IBuildingUtility>();
            
            // 当玩家点击鼠标后，设置建筑物为不透明
            _buildingUtility.SetOpaque(_createBuilding.GetComponent<Renderer>().material);
            // 启用建筑物的碰撞体和导航网格障碍物
            _buildingUtility.SetDeflate(_createBuilding, true);
        }
    }
}