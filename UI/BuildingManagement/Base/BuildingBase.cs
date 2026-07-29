using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Serialization;

namespace Test.BuildingSystem
{
    // BuildingBase 是所有建筑物类的基类，提供建筑物的基本信息和更新逻辑。
    // 该类是抽象的，不能直接实例化。它定义了建筑物的名称、生命值、大小和 ID 等基本属性。
    public abstract class BuildingBase : MonoBehaviour
    {
        public int id; // 建筑物的唯一标识符（ID）

        public string buildName; // 建筑物的名称，标识该建筑物

        public string description; // 建筑物的描述，对该建筑物的功能和特点进行概括

        public float maxHp; // 建筑物的生命值，表示建筑的耐久度

        public Vector2Int buildSize; // 建筑物的尺寸，使用 Vector2Int 来表示建筑物在地面上的占用格子大小（宽，高）

        public int buildingCost; //建筑物的消耗

        
        // 构造函数用于初始化建筑物的名称、生命值、尺寸和 ID
        // 默认情况下，ID 设置为 -1，如果提供了其他值则使用指定的 ID
        protected BuildingBase(string buildName,string description, float maxHp, Vector2Int buildSize, int id = -1)
        {
            this.buildName = buildName;
            this.description = description;
            this.maxHp = maxHp;
            this.buildSize = buildSize;
            this.id = id;
        }

        // 抽象方法，逻辑更新函数，所有继承此类的建筑物必须实现此方法来处理每段时间后执行的默认事件
        // deltaTime 是该建筑物的事件执行冷却事件
        protected abstract IEnumerator LogicUpdate(float deltaTime);
        
    }
}