using System;
using QFramework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.UI;
using Utils;

namespace UI.MapUI
{
    public class MapUIEntityController : MonoBehaviour
    {
        private Entity _entityInMap; // 地图上对应的实体
        private float3 _centerPos; // 地图的中心点
        private float2 _ratio; // 小地图：地图的比率

        private EntityManager _entityManager; // 实体控制器
        private LocalTransform _entityTransform; // 实体的位置组件
        [SerializeField]
        private RectTransform mapUIEntityTransform; // UI位置组件

        private bool _isInit; // 初始化是否完成

        private void Awake()
        {
            _isInit = false;
        }
        
        private void FixedUpdate()
        {
            if (_isInit)
            {
                SetPosition();
            }
        }

        /// <summary>
        /// 初始化小地图UI实体
        /// </summary>
        /// <param name="entity"> 对应地图上的实体 </param>
        /// <param name="centerPos"> 实际地图中心点 </param>
        /// <param name="ratio"> 小地图：实际地图 比率 </param>
        /// <param name="isEnemy"> 是否是敌人 </param>
        /// <param name="type"> UI实体类型 </param>
        public void InitMapUIEntity(Entity entity, float3 centerPos, float2 ratio, bool isEnemy, MapUIEntityType type)
        {
            _entityInMap = entity;
            _centerPos = centerPos;
            _ratio = ratio;

            _entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
            if (_entityManager.HasComponent<LocalTransform>(_entityInMap))
            {
                _entityTransform = _entityManager.GetComponentData<LocalTransform>(_entityInMap);
            }

            SetPosition();
            
            // 处理颜色
            if (isEnemy)
                this.GetComponent<Image>().color = MapUISetting.EnemyColor;
            else
                this.GetComponent<Image>().color = MapUISetting.PlayerColor;

            // 处理大小
            switch (type)
            {
                case MapUIEntityType.Building:
                    this.GetComponent<RectTransform>().sizeDelta = MapUISetting.BuildingSize;
                    break;
                
                case MapUIEntityType.Hero:
                    this.GetComponent<RectTransform>().sizeDelta = MapUISetting.HeroSize;
                    break;
                
                case MapUIEntityType.Solider:
                    this.GetComponent<RectTransform>().sizeDelta = MapUISetting.SoliderSize;
                    break;
                
                default:
                    break;
            }

            _isInit = true;
        }

        private void SetPosition()
        {
            _entityTransform = _entityManager.GetComponentData<LocalTransform>(_entityInMap);
            
            float2 currentPos = new float2((_entityTransform.Position.x - _centerPos.x) * _ratio.x,
                (_entityTransform.Position.z - _centerPos.z) * _ratio.y);
            
            mapUIEntityTransform.anchoredPosition = currentPos;
            // Debug.Log(currentPos);
            //Debug.Log(currentPos);
            DebugSystem.Log($"{currentPos}","");
        }
    }    
}
    
