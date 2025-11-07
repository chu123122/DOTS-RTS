using System.Collections;
using UnityEngine;
using UnityEngine.Serialization;

namespace Test.BuildingSystem
{
    public class HomeBuilding : BuildingBase
    {
        public HomeBuilding(string buildName, string description, float buildHp, Vector2Int buildSize, int id = -1) :
            base(buildName, description, buildHp, buildSize, id)
        {
        }

        private void Start()
        {
            StartCoroutine(LogicUpdate(5f));
        }

        protected override IEnumerator LogicUpdate(float deltaTime)
        {
            yield return new WaitForSeconds(deltaTime);
        }
        
    }
}