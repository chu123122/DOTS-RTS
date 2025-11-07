using DefaultNamespace;
using Entities._Common;
using Test;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using 通用;
using 通用.HealBar;

namespace a
{
    public class HealthBarController:MonoBehaviour,IServiceSystemLocator
    {
        [SerializeField]private GameObject healthBarPrefab;

        private void OnEnable()
        {
            var initializeUnitSystem = World.DefaultGameObjectInjectionWorld.GetExistingSystemManaged<InitializeUnitSystem>();
            initializeUnitSystem.OnCreateHealthBar += CreateHealthBar;
        }

        private void OnDisable()
        {
            if(World.DefaultGameObjectInjectionWorld==null)return;
            var initializeUnitSystem = World.DefaultGameObjectInjectionWorld.GetExistingSystemManaged<InitializeUnitSystem>();
            initializeUnitSystem.OnCreateHealthBar -= CreateHealthBar;
        }

        private void CreateHealthBar(int id,float3 position)
        {
            GameObject healthBar = Instantiate(healthBarPrefab,position,Quaternion.identity,transform);
            healthBar.GetComponent<HealthBar>().FollowEntity=
                this.GetService<ClientHelpSystem>().GetEntityByIndexInClientWorld(id);
        }
        private void Update()
        {
  
        }
    }
}