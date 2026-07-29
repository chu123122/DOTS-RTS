using Test.BuildingSystem;
using UnityEngine;
using 简单战斗.ServiceLocator;

namespace QFramework.BuildingManagement.Utils
{
    public interface IGridManager:IServiceBase
    {
        Vector3Int WorldToCell(Vector3 position);
        Vector3 SnapToGrid(Vector3 worldPosition);

        /// <summary>
        /// 往GridData里面添加物体
        /// </summary>
        /// <param name="building"></param>
        /// <param name="gridPosition"></param>
        /// <returns></returns>
        void PutObjectAt(BuildingBase building, Vector3Int gridPosition);

        /// <summary>
        /// 从GridData里面删除物体
        /// </summary>
        /// <param name="gridPosition"></param>
        void RemoveObject(Vector3Int gridPosition);

        /// <summary>
        /// 判断是否可以放置物体在该区域
        /// </summary>
        /// <param name="gridPosition"></param>
        /// <param name="building"></param>
        /// <returns></returns>
        bool CheckHaveBuildingAt(Vector3Int gridPosition, BuildingBase building);

        bool CheckHaveBuildingAt(Vector3 position, BuildingBase building);
    }
}