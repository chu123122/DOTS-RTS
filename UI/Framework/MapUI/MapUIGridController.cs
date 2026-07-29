using System;
using DefaultNamespace;
using QFramework;
using Unity.Mathematics;
using UnityEngine;

namespace UI.MapUI
{
    public class MapUIGridController : MonoBehaviour, IController
    {
        public GameObject currentMap; // 当前的地图

        [Header("暂时测试")] 
        public float3 currentCenter;
        public float3 currentLenAndWid;

        private void Start()
        {
            // this.GetModel<MapUIModel>().SetMapData(currentMap);
            this.GetModel<MapUIModel>().SetMapData(currentCenter, currentLenAndWid);
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }    
}

