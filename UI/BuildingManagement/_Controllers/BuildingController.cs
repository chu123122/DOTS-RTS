using DefaultNamespace;
using UnityEngine;
using UnityEngine.UI;
using 简单战斗.ServiceLocator;

// 建造面板的显示和关闭
namespace QFramework.BuildingManagement._Controllers
{
    public class BuildingController : MonoBehaviour, IController, IGetService
    {
        // UI 按钮，玩家点击该按钮后进入建筑模式
        public Button buildingButton;
        
        public GameObject buildingCanvasObject;
        // 只允许建筑物放置在该 Layer 层的物体上
        public LayerMask groundMask;
        // private int tagBuild = 0;

        // 在初始化时绑定按钮的点击事件，进入建筑模式   
        
        private void Awake()
        {
            SubscribeToEvents();
        }

        private void SubscribeToEvents()
        {
            buildingButton.onClick.AddListener(() =>
            {
                Debug.Log("Enter Building State!"); // TODO: 进入时停状态
                OpenBuildingUI();
            });
        }

        //反激活buildingCanvasObject的激活状态
        private void OpenBuildingUI()
        {
            buildingCanvasObject.SetActive(!buildingCanvasObject.activeSelf);
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}
