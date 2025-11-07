using System.Collections.Generic;
using DefaultNamespace;
using QFramework.BuildingManagement.Utils;
using Test.BuildingSystem;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;
using Utils;
using 简单战斗.ServiceLocator;

namespace QFramework.BuildingManagement.Services
{
    /// <summary>
    /// 目前负责纯粹的数据交互，不负责表现层面上的逻辑
    /// </summary>
    public class GridManager : ServiceObject<GridManager>, IGridManager
    {
        public Transform gridTransform;
        public List<GameObject> buildingsList = new List<GameObject>();
        private readonly IGridData _gridData = new GridData();
        private Grid _grid;

        protected override void Awake()
        {
            base.Awake();
            _grid = gridTransform.GetChild(0).GetComponent<Grid>();
        }

        // 将世界坐标（Vector3）转化为Grid上Cell坐标（Vector3Int）
        public Vector3Int WorldToCell(Vector3 position)
        {
            return _grid.WorldToCell(position);
        }

        // 将世界坐标对齐到最近的网格单元
        public Vector3 SnapToGrid(Vector3 worldPosition)
        {
            Vector3Int vector3Int = _grid.WorldToCell(worldPosition);
            Vector3 vector3 = _grid.GetCellCenterWorld(vector3Int);
            return vector3;
        }

        /// <summary>
        /// 往GridData里面添加物体
        /// </summary>
        /// <param name="building"></param>
        /// <param name="gridPosition"></param>
        /// <returns></returns>
        public void PutObjectAt(BuildingBase building, Vector3Int gridPosition)
        {
            buildingsList.Add(building.gameObject);

            int placedObjectIndex = buildingsList.Count - 1;
            _gridData.AddObjectAt(gridPosition, building.buildSize, building.id, placedObjectIndex);
        }

        /// <summary>
        /// 从GridData里面删除物体
        /// </summary>
        /// <param name="gridPosition"></param>
        public void RemoveObject(Vector3Int gridPosition)
        {
            int placedObjectIndex = _gridData.GetRepresentationIndex(gridPosition);
            buildingsList.RemoveAt(placedObjectIndex);
            _gridData.RemoveObjectAt(gridPosition);
        }

        /// <summary>
        /// 判断是否可以放置物体在该区域
        /// </summary>
        /// <param name="gridPosition"></param>
        /// <param name="building"></param>
        /// <returns></returns>
        public bool CheckHaveBuildingAt(Vector3Int gridPosition, BuildingBase building)
        {
            Vector2Int objectSize = building.buildSize;
            return _gridData.CanPlaceObjectAt(gridPosition, objectSize);
        }

        public bool CheckHaveBuildingAt(Vector3 position, BuildingBase building)
        {
            var gridPosition = _grid.WorldToCell(position);
            Vector2Int objectSize = building.buildSize;
            return _gridData.CanPlaceObjectAt(gridPosition, objectSize);
        }
    }
}