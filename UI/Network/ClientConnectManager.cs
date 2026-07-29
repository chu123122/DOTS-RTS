using Unity.Entities;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace 客户端
{
    /// <summary>
    /// 本地模式入口。保留 NetCode 包供现有组件和回放代码编译，
    /// 但运行时不再销毁 Local World，也不创建 Server/Client World。
    /// </summary>
    public class ClientConnectManager : MonoBehaviour
    {
        [SerializeField] private Button connectButton;

        private void Awake()
        {
            connectButton.onClick.AddListener(OnPlayerConnect);
        }

        private void OnPlayerConnect()
        {
            World localWorld = World.DefaultGameObjectInjectionWorld;
            if (localWorld == null || !localWorld.IsCreated)
            {
                Debug.LogError("Local World 不可用，无法进入本地游戏。");
                return;
            }

            SceneManager.LoadScene(1);
            SceneManager.LoadSceneAsync("SubScene 1", LoadSceneMode.Additive);
        }
    }
}
