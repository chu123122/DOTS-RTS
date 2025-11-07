using System;
using System.Collections.Generic;
using UnityEngine;

namespace Test.BuildingSystem
{
    public class GridData:IGridData
    {
        // 用于存储已经放置的物体，键是网格位置，值是对应的放置数据
        private readonly Dictionary<Vector3Int, PlacementData> _placedObjects = new();

        // 在指定网格位置添加一个物体，物体的大小和 ID 以及放置物体的索引都将记录
        public void AddObjectAt(Vector3Int gridPosition, Vector2Int objectSize, int id, int placedObjectIndex)
        {
            List<Vector3Int> positionToOccuply = CalculatePositions(gridPosition, objectSize);
            PlacementData data = new PlacementData(positionToOccuply, id, placedObjectIndex);
        
            foreach (var pos in positionToOccuply)
            {
                if (_placedObjects.ContainsKey(pos))
                {
                    throw new Exception("Dictionary already contains this cell position");
                }
                _placedObjects[pos] = data;
            }
        }

        // 计算物体在给定网格位置和大小下，占用的所有网格位置
        private List<Vector3Int> CalculatePositions(Vector3Int gridPosition, Vector2Int objectSize)
        {
            List<Vector3Int> returnVal1 = new();
            for (int x = 0; x < objectSize.x; x++)
            {
                for (int y = 0; y < objectSize.y; y++)
                {
                    returnVal1.Add(gridPosition + new Vector3Int(x, 0, y));
                }
            }
            return returnVal1;
        }

        // 检查物体是否可以放置在给定的网格位置，判断是否有其他物体占用该区域
        public bool CanPlaceObjectAt(Vector3Int gridPosition, Vector2Int objectSize)
        {
            List<Vector3Int> positionToOccupy = CalculatePositions(gridPosition, objectSize);
        
            foreach (var pos in positionToOccupy)
            {
                if (_placedObjects.ContainsKey(pos))
                {
                    return false;
                }
            }
            return true;
        }

        // 移除在指定网格位置的物体及其占用的所有位置
        public void RemoveObjectAt(Vector3Int gridPosition)
        {
            foreach (var pos in _placedObjects[gridPosition].OccupiedPositions)
            {
                _placedObjects.Remove(pos);
            }
        }

        // 获取指定网格位置的物体的表示索引，如果该位置没有物体，返回 -1
        public int GetRepresentationIndex(Vector3Int gridPosition)
        {
            if (_placedObjects.ContainsKey(gridPosition) == false)
                return -1; 
            return _placedObjects[gridPosition].PlacedObjectIndex;
        }
    }


// 记录一个物体在网格上放置的信息，包含物体占用的位置列表、ID 和放置索引
    public class PlacementData
    {
        // 物体占用的所有网格位置
        public readonly List<Vector3Int> OccupiedPositions;
    
        // 物体的 ID
        public int ID { get; private set; }
    
        // 物体的放置索引，用于区分不同的放置实例
        public int PlacedObjectIndex { get; private set; }

        // 构造函数，初始化占用位置、ID 和放置索引
        public PlacementData(List<Vector3Int> occupiedPositions, int iD, int placedObjectIndex)
        {
            this.OccupiedPositions = occupiedPositions;
            ID = iD;
            PlacedObjectIndex = placedObjectIndex;
        }
    }
}