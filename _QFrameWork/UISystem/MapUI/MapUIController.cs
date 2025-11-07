using System;
using DefaultNamespace;
using QFramework;
using UI.MapUI.Events;
using Unity.Entities;
using UnityEngine;
using UnityEngine.UI;

namespace UI.MapUI
{
    public class MapUIController : MonoBehaviour, IController
    {
        public GameObject mapUIParent; // 小地图范围以及小地图实体生成的父物体
        public GameObject mapUIEntityPrefab; // 小地图实体预制体

        private MapUIModel _mapUIModel; // 小地图UI相关的数据

        private void Start()
        {
            _mapUIModel = this.GetModel<MapUIModel>(); // 获取所需的数据
            
            _mapUIModel.SetMapUIData(mapUIParent);

            // todo 将创造和销毁与实际对接绑定
            // 创造事件绑定
            this.RegisterEvent<CreateEntityEvent>(e =>
            {
                CreateMapUIEntity(e.IsEnemy, e.Type, e.Entity);
            }).UnRegisterWhenDisabled(this);
        }
        
        /// <summary>
        /// 创造对应的UI
        /// </summary>
        /// <param name="isEnemy"> 是否是敌人 </param>
        /// <param name="type"> UI类型 </param>
        /// <param name="entity"> 地图上的实体 </param>
        private void CreateMapUIEntity(bool isEnemy, MapUIEntityType type, Entity entity)
        {
            // 创造UI实体并初始化
            GameObject currentMapUIEntity = Instantiate(mapUIEntityPrefab, mapUIParent.transform);
            currentMapUIEntity.GetComponent<MapUIEntityController>().InitMapUIEntity(entity,
                _mapUIModel.GetMapCenter(), _mapUIModel.GetMapRatio(), isEnemy, type);
            
            // 传入数据中并生成字典
            _mapUIModel.ImportMapUIDic(entity.ToString(), currentMapUIEntity);
        }
        /// <summary>
        /// 销毁对应地图UI
        /// </summary>
        /// <param name="entityString"> 对应的实体编号 </param>
        private void DestroyMapUIEntity(string entityString)
        {
            GameObject currentMapUI = _mapUIModel.DestroyMapUIDic(entityString);
            
            Destroy(currentMapUI);
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}
