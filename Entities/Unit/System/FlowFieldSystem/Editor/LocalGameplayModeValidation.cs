using System;
using System.IO;
using Entities.Unit.System.FlowFieldSystem;
using Entities._Common.SpawnEntityRpc;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEditor;
using UnityEngine;
using _RePlaySystem.Base;
using 通用;

public static class LocalGameplayModeValidation
{
    private static string ValidationRequestPath => Path.GetFullPath(Path.Combine(
        Application.dataPath,
        "../Temp/RunLocalGameplayModeValidation"));

    [InitializeOnLoadMethod]
    private static void RunRequestedValidationAfterReload()
    {
        if (!File.Exists(ValidationRequestPath))
            return;

        File.Delete(ValidationRequestPath);
        EditorApplication.delayCall += Run;
    }

    [MenuItem("RTS/Validation/Local Gameplay Mode")]
    public static void Run()
    {
        ValidateMoveOrderWithoutNetworkWorld();
        ValidateLocalUnitSpawnAndIds();
        Debug.Log("LOCAL_GAMEPLAY_VALIDATION_OK\nmove order: local consumption=1\nlocal spawn ids: 1,2");
    }

    private static void ValidateMoveOrderWithoutNetworkWorld()
    {
        using var world = new World("Local Move Order Validation", WorldFlags.Game);
        EntityManager entityManager = world.EntityManager;
        Entity managerEntity = entityManager.CreateEntity(
            typeof(FlowFieldGlobalTarget),
            typeof(MoveOrder),
            typeof(RecalculateFlowFieldTag));
        entityManager.SetComponentData(
            managerEntity,
            new FlowFieldGlobalTarget { TargetPosition = float3.zero });
        entityManager.SetComponentData(
            managerEntity,
            new MoveOrder { TargetPosition = new float3(3f, 0f, 4f) });
        entityManager.SetComponentData(
            managerEntity,
            new RecalculateFlowFieldTag { RequestVersion = 7 });
        entityManager.SetComponentEnabled<MoveOrder>(managerEntity, true);
        entityManager.SetComponentEnabled<RecalculateFlowFieldTag>(managerEntity, false);

        RtsCommandSystem system = world.CreateSystemManaged<RtsCommandSystem>();
        system.Update();

        Require(
            math.distance(
                entityManager.GetComponentData<FlowFieldGlobalTarget>(managerEntity)
                    .TargetPosition,
                new float3(3f, 0f, 4f)) <= 0.0001f,
            "Local move order did not update the Flow Field target.");
        Require(
            entityManager.GetComponentData<RecalculateFlowFieldTag>(managerEntity)
                .RequestVersion == 8 &&
            entityManager.IsComponentEnabled<RecalculateFlowFieldTag>(managerEntity),
            "Local move order did not request a new Flow Field bake.");
        Require(!entityManager.IsComponentEnabled<MoveOrder>(managerEntity),
            "Local move order was not consumed exactly once.");
    }

    private static void ValidateLocalUnitSpawnAndIds()
    {
        World previousWorld = World.DefaultGameObjectInjectionWorld;
        using var world = new World("Local Spawn Validation", WorldFlags.Game);
        try
        {
            World.DefaultGameObjectInjectionWorld = world;
            EntityManager entityManager = world.EntityManager;
            Entity prefab = entityManager.CreateEntity(
                typeof(Prefab),
                typeof(LocalTransform),
                typeof(RtsTeam));
            Entity prefabContainer = entityManager.CreateEntity();
            entityManager.AddComponentData(
                prefabContainer,
                new RtsLocalPrefabs { Entity = prefab });

            new CreateBaseUnitRpc(new float3(1f, 0.5f, 2f)).CreateEntityRpc();
            new CreateBaseUnitRpc(new float3(2f, 0.5f, 3f)).CreateEntityRpc();

            using EntityQuery query = entityManager.CreateEntityQuery(
                ComponentType.ReadOnly<LocalInstance>(),
                ComponentType.ReadOnly<LocalTransform>());
            using NativeArray<Entity> units = query.ToEntityArray(Allocator.Temp);
            Require(units.Length == 2, "Local spawn did not create exactly two units.");

            int firstId = entityManager.GetComponentData<LocalInstance>(units[0]).Id;
            int secondId = entityManager.GetComponentData<LocalInstance>(units[1]).Id;
            Require(math.min(firstId, secondId) == 1 && math.max(firstId, secondId) == 2,
                "Local unit IDs were not unique and sequential.");
        }
        finally
        {
            World.DefaultGameObjectInjectionWorld = previousWorld;
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}
