    #if UNITY_EDITOR
    using Unity.Entities;
    using UnityEngine.SceneManagement;


    public partial class LoadConnectionSceneSystem:SystemBase
    {
        protected override void OnCreate()
        {
            this.Enabled = false;
            if(SceneManager.GetActiveScene()==SceneManager.GetSceneByName("ConnectionScene"))return;
            SceneManager.LoadScene("ConnectionScene");
        }

        protected override void OnUpdate()
        {
        }
    }
    #endif
