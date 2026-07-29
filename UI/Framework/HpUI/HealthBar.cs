using Entities._Common;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using 通用;

namespace Test
{
    public class HealthBar : MonoBehaviour
    {
        public float yOffset;
        public Entity FollowEntity;
        private Camera _camera;
        private Slider _slider;

        private void Awake()
        {
            _camera = Camera.main;
            _slider = this.GetComponent<Slider>();
        }

        private void Update()
        {
            var entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
            LocalTransform localTransform =
                entityManager.GetComponentData<LocalTransform>(FollowEntity);
            SetPosition(localTransform);

            HealthPointData hpData = entityManager.GetComponentData<HealthPointData>(FollowEntity);
            SetHealth(hpData);
        }

        private void SetPosition( LocalTransform localTransform)
        {
            Vector3 screenPos = _camera.WorldToScreenPoint(localTransform.Position);
            screenPos.y += yOffset;
            transform.position = screenPos;
        }

        private void SetHealth(HealthPointData hpData)
        {
            _slider.value = (float)hpData.CurrentHp / hpData.MaximumHp;
        }
    }
}