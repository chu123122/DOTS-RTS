using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using _RePlaySystem.Base;
using 通用;

namespace Entities._Common.SpawnEntityRpc
{
    public class CreateBaseUnitRpc:ICreateEntityRpc
    {
        private readonly float3 _position;
        public CreateBaseUnitRpc(float3 position)
        {
            _position=position;
        }

        public void CreateEntityRpc()
        {
            World world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated)
                throw new InvalidOperationException("Local World 不可用，无法生成单位。");

            EntityManager entityManager = world.EntityManager;
            using var prefabQuery = entityManager.CreateEntityQuery(typeof(RtsLocalPrefabs));
            if (prefabQuery.IsEmptyIgnoreFilter)
                throw new InvalidOperationException("本地单位 Prefab 尚未完成加载。");

            Entity prefab = prefabQuery.GetSingleton<RtsLocalPrefabs>().Entity;
            Entity unit = entityManager.Instantiate(prefab);
            LocalTransform transform = LocalTransform.FromPosition(_position);
            transform.Scale = 0.5f;
            entityManager.SetComponentData(unit, transform);

            int localId = AllocateLocalId(entityManager);
            if (entityManager.HasComponent<LocalInstance>(unit))
            {
                entityManager.SetComponentData(unit, new LocalInstance { Id = localId });
            }
            else
            {
                entityManager.AddComponentData(unit, new LocalInstance { Id = localId });
            }

            if (entityManager.HasComponent<RtsTeam>(unit))
                entityManager.SetComponentData(unit, new RtsTeam { Value = TeamType.Blue });
        }

        private static int AllocateLocalId(EntityManager entityManager)
        {
            using var sequenceQuery =
                entityManager.CreateEntityQuery(typeof(LocalInstanceIdSequence));
            Entity sequenceEntity;
            if (sequenceQuery.IsEmptyIgnoreFilter)
            {
                sequenceEntity = entityManager.CreateEntity();
                entityManager.AddComponentData(
                    sequenceEntity,
                    new LocalInstanceIdSequence { NextId = 1 });
            }
            else
            {
                sequenceEntity = sequenceQuery.GetSingletonEntity();
            }

            LocalInstanceIdSequence sequence =
                entityManager.GetComponentData<LocalInstanceIdSequence>(sequenceEntity);
            int id = sequence.NextId;
            sequence.NextId++;
            entityManager.SetComponentData(sequenceEntity, sequence);
            return id;
        }
    }
}
