using UnityEngine;
using UnityEngine.Serialization;

namespace Test.BuildingSystem
{
    [System.Serializable]
    public class BuildingData
    {
        public int id; // 建筑物的唯一标识符（ID）

        public string prefabPath;      // 预制体路径
        
        public string buildName; // 建筑物的名称，标识该建筑物

        public string description; // 建筑物的描述，对该建筑物的功能和特点进行概括

        public float maxHp; // 建筑物的生命值，表示建筑的耐久度

        public Vector2Int buildSize; // 建筑物的尺寸，使用 Vector2Int 来表示建筑物在地面上的占用格子大小（宽，高）
        
        
        
        [System.NonSerialized]
        private GameObject _prefabReference;

        public GameObject GetPrefab()
        {
            if (_prefabReference == null)
            {
                _prefabReference = Resources.Load<GameObject>(prefabPath);
            }
            return _prefabReference;
        }
    }
    [System.Serializable]
    public class BuildingDataWrapper
    {
        public BuildingData[] buildings;
    }
    
    
}