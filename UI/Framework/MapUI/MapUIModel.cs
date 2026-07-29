using System.Collections.Generic;
using QFramework;
using Unity.Mathematics;
using UnityEngine;

namespace UI.MapUI
{
    public class MapUIModel : AbstractModel
    {
        private Dictionary<string, GameObject> _mapUIEntitiesDic; // 存储小地图上显示的UI的字典

        private float3 _mapCenterPos; // 地图的中点坐标
        private float3 _mapLenAndWid; // 地图的长宽高
        private float2 _mapUILenAndWid; // 小地图的长宽

        /// <summary>
        /// 传入地图数据
        /// </summary>
        /// <param name="map"> 实际地图 </param>
        public void SetMapData(GameObject map)
        {
            _mapCenterPos = map.GetComponent<TerrainCollider>().bounds.center;
            _mapLenAndWid = map.GetComponent<TerrainCollider>().bounds.size;
            
            //Debug.Log(_mapCenterPos + " " + _mapLenAndWid);
        }
        /// <summary>
        /// 仅测试
        /// </summary>
        /// <param name="center"></param>
        /// <param name="lenAndWid"></param>
        public void SetMapData(float3 center, float3 lenAndWid)
        {
            _mapCenterPos = center;
            _mapLenAndWid = lenAndWid;
        }
        /// <summary>
        /// 传入小地图数据
        /// </summary>
        /// <param name="mapUI"> 小地图 </param>
        public void SetMapUIData(GameObject mapUI)
        {
            _mapUILenAndWid = mapUI.GetComponent<RectTransform>().sizeDelta;
            // Debug.Log(_mapUILenAndWid);
        }

        /// <summary>
        /// 获取当前地图的长宽高
        /// </summary>
        /// <returns></returns>
        public float3 GetMapCenter()
        {
            return _mapCenterPos;
        }
        /// <summary>
        /// 获取地图比率
        /// </summary>
        /// <returns> 小地图：游戏地图 </returns>
        public float2 GetMapRatio()
        {
            return new float2(_mapUILenAndWid.x / _mapLenAndWid.x, _mapUILenAndWid.y / _mapLenAndWid.z);
        }
        
        /// <summary>
        /// 导入新UI
        /// </summary>
        /// <param name="key"> 对应的实体编号 </param>
        /// <param name="mapUIEntity"> 对应的UI </param>
        /// <returns> 是否添加成功 </returns>
        public bool ImportMapUIDic(string key, GameObject mapUIEntity)
        {
            if (!_mapUIEntitiesDic.ContainsKey(key))
                return false;
            
            _mapUIEntitiesDic.Add(key, mapUIEntity);
            
            return true;
        }
        /// <summary>
        /// 销毁UI
        /// </summary>
        /// <param name="key"> 对应的实体编号 </param>
        /// <returns> 需要销毁的UI </returns>
        public GameObject DestroyMapUIDic(string key)
        {
            _mapUIEntitiesDic.TryGetValue(key, out GameObject currentMapUI);
            _mapUIEntitiesDic.Remove(key);
            
            return currentMapUI;
        }
        
        protected override void OnInit()
        {
            _mapUIEntitiesDic = new Dictionary<string, GameObject>();
        }
    }
}

