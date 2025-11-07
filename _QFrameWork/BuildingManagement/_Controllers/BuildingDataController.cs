using System.Collections.Generic;
using DefaultNamespace;
using QFramework.BuildingManagement.Commands;
using Test.BuildingSystem;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

// 生成对于的按钮绑定按钮事件
namespace QFramework.BuildingManagement._Controllers
{
    public class BuildingDataController : MonoBehaviour, IController
    {
        public List<BuildingData> buildingList;
        public GameObject buttonPrefab;
        public Transform buildingUIContainer;
        
        private LayerMask _groundLayerMask;
        private const string JsonFilePath = "Configs/Buildings";
        private void Awake()
        {
            _groundLayerMask = LayerMask.GetMask("Ground");
            LoadBuildingData();
            CreateBuildingButtons();
        }

        private void LoadBuildingData()
        {
            string json = Resources.Load<TextAsset>(JsonFilePath).text; // 获取 JSON 文件的文本内容
            BuildingData[] buildings = JsonUtility.FromJson<BuildingDataWrapper>(json).buildings;
            buildingList.AddRange(buildings);
        }

        private void CreateBuildingButtons()
        {
            foreach (var building in buildingList)
            {
                if(building.prefabPath.Equals("1"))return;
                // 创建 UI 按钮
                GameObject button = Instantiate(buttonPrefab, buildingUIContainer);
                button.name = building.buildName;
                button.GetComponentInChildren<TMP_Text>().text = building.buildName;

                // 绑定点击事件，创建对应的建筑物
                button.GetComponent<Button>().onClick.AddListener(() =>
                {
                    Debug.Log($"Building {building.buildName} selected!");
                    GameObject prefab = Resources.Load<GameObject>(building.prefabPath);
                    this.SendCommand(new CreateBuildingCommand(prefab, _groundLayerMask));
                });
            }
        }


        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}