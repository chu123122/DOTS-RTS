using System;
using System.IO;
using Entities._Common.SpawnEntityRpc;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEditor;
using UnityEngine;
using _RePlaySystem.Base;
using 通用;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Editor
{

/// <summary>
/// 本地 World、固定槽位与诊断可选路径的编辑器回归入口。
/// </summary>
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
        ValidateFixedDestinationSlotMath();
        ValidatePerSlotArrivalAndSteering();
        ValidateMovementWithoutDiagnosticComponents();
        Debug.Log(
            "LOCAL_GAMEPLAY_VALIDATION_OK\n" +
            "move order: local consumption=1\n" +
            "local spawn ids: 1,2\n" +
            "fixed slots: unique, walkable, assigned=1\n" +
            "per-slot movement: direct steer=1, settled=1\n" +
            "diagnostics optional: movement=1");
    }

    private static void ValidateMoveOrderWithoutNetworkWorld()
    {
        var gridCells = new NativeArray<FlowFieldCell>(100, Allocator.Temp);
        try
        {
            for (int i = 0; i < gridCells.Length; i++)
                gridCells[i] = new FlowFieldCell { Cost = 1 };

            using var world = new World("Local Move Order Validation", WorldFlags.Game);
            EntityManager entityManager = world.EntityManager;
            Entity managerEntity = entityManager.CreateEntity(
                typeof(FlowFieldGlobalTarget),
                typeof(FlowFieldGrid),
                typeof(MoveOrder),
                typeof(FlowFieldRuntimeState),
                typeof(RecalculateFlowFieldTag));
            entityManager.SetComponentData(
                managerEntity,
                new FlowFieldGlobalTarget { TargetPosition = float3.zero });
            entityManager.SetComponentData(managerEntity, new FlowFieldGrid
            {
                Grid = gridCells,
                GridOrigin = float3.zero,
                GridDimensions = new int2(10, 10),
                CellRadius = 0.5f
            });
            entityManager.SetComponentData(
                managerEntity,
                new MoveOrder { TargetPosition = new float3(3f, 0f, 4f) });
            entityManager.SetComponentData(
                managerEntity,
                new RecalculateFlowFieldTag { RequestVersion = 7 });
            entityManager.SetComponentData(
                managerEntity,
                new FlowFieldRuntimeState { ActiveVersion = 0 });
            entityManager.SetComponentEnabled<MoveOrder>(managerEntity, true);
            entityManager.SetComponentEnabled<RecalculateFlowFieldTag>(managerEntity, false);

            Entity unit = entityManager.CreateEntity(
                typeof(BasicUnitTag),
                typeof(UnitSelected),
                typeof(UnitMoveDestination),
                typeof(FlowArrivalState),
                typeof(LocalTransform));
            entityManager.SetComponentData(unit, new UnitSelected { Value = true });
            entityManager.SetComponentData(
                unit,
                LocalTransform.FromPosition(new float3(1f, 0f, 1f)));

            RtsCommandSystem system = world.CreateSystemManaged<RtsCommandSystem>();
            system.Update();

            Require(entityManager.IsComponentEnabled<MoveOrder>(managerEntity),
                "Move order was consumed before the initial Flow Field became ready.");
            Require(
                entityManager.GetComponentData<RecalculateFlowFieldTag>(managerEntity)
                    .RequestVersion == 7,
                "Deferred move order mutated the Flow Field request.");

            entityManager.SetComponentData(
                managerEntity,
                new FlowFieldRuntimeState
                {
                    ActiveVersion = 1,
                    ActiveRequestVersion = 7
                });
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
            UnitMoveDestination destination =
                entityManager.GetComponentData<UnitMoveDestination>(unit);
            Require(destination.IsActive != 0 && destination.OrderVersion == 8,
                "Selected local unit did not receive a fixed destination.");
        }
        finally
        {
            gridCells.Dispose();
        }
    }

    private static void ValidateFixedDestinationSlotMath()
    {
        var gridCells = new NativeArray<FlowFieldCell>(25, Allocator.Temp);
        try
        {
            for (int i = 0; i < gridCells.Length; i++)
                gridCells[i] = new FlowFieldCell { Cost = 1 };
            gridCells[FlowFieldUtils.GetFlatIndex(new int2(2, 2), new int2(5, 5))] =
                new FlowFieldCell { Cost = 0 };
            var grid = new FlowFieldGrid
            {
                Grid = gridCells,
                GridOrigin = new float3(-2.5f, 0f, -2.5f),
                GridDimensions = new int2(5, 5),
                CellRadius = 0.5f
            };

            var slots = MoveDestinationSlotUtility.GenerateWalkableSlots(
                float3.zero,
                4,
                0.75f,
                grid);
            Require(slots.Count == 4, "Slot generator did not produce the requested count.");
            for (int i = 0; i < slots.Count; i++)
            {
                int2 cell = (int2)math.floor((slots[i].xz - grid.GridOrigin.xz) / 1f);
                Require(grid.Grid[FlowFieldUtils.GetFlatIndex(cell, grid.GridDimensions)].Cost != 0,
                    "Slot generator placed a destination inside a blocked cell.");
                for (int j = i + 1; j < slots.Count; j++)
                    Require(math.distancesq(slots[i], slots[j]) > 0.000001f,
                        "Slot generator produced duplicate destinations.");
            }

            float3[] unitPositions =
            {
                new float3(-1f, 0f, 0f),
                new float3(1f, 0f, 0f),
                new float3(0f, 0f, -1f),
                new float3(0f, 0f, 1f)
            };
            int[] assignments = MoveDestinationSlotUtility.AssignSlotsPreservingFormation(
                unitPositions,
                slots,
                float3.zero);
            var used = new bool[slots.Count];
            foreach (int slotIndex in assignments)
            {
                Require(slotIndex >= 0 && slotIndex < slots.Count && !used[slotIndex],
                    "Slot assignment did not produce a unique valid destination.");
                used[slotIndex] = true;
            }
        }
        finally
        {
            gridCells.Dispose();
        }
    }

    private static void ValidatePerSlotArrivalAndSteering()
    {
        var gridCells = new NativeArray<FlowFieldCell>(9, Allocator.Temp);
        var footprints = new NativeArray<float2>(1, Allocator.Temp);
        var states = new NativeArray<FlowMovementFrameState>(1, Allocator.Temp);
        try
        {
            for (int i = 0; i < gridCells.Length; i++)
            {
                gridCells[i] = new FlowFieldCell
                {
                    Cost = 1,
                    IntegrationValue = 1,
                    BestDirectionIndex = 0
                };
            }

            footprints[0] = new float2(1f, 1f);
            var job = new CalculateIndependentFlowForceJob
            {
                Grid = gridCells,
                GridOrigin = float3.zero,
                GridDimensions = new int2(3, 3),
                CellRadius = 0.5f,
                ActiveRequestVersion = 1,
                CollisionFootprints = footprints,
                States = states
            };
            var velocity = new Velocity { Value = float3.zero };
            var speed = new UnitMoveSpeed { Value = 2f };
            var settings = new UnitMovementSettings { MaxForce = 20f };
            var contactBody = new UnitContactBody { InverseMass = 1f };
            var destination = new UnitMoveDestination
            {
                Position = new float3(1.8f, 0f, 1f),
                ArrivalRadius = 0.2f,
                DirectApproachIntegrationDistance = 2,
                OrderVersion = 2,
                IsActive = 1
            };
            var arrival = new FlowArrivalState { IsSettled = false };

            job.Execute(
                Entity.Null,
                0,
                LocalTransform.FromPosition(new float3(1f, 0f, 1f)),
                new Velocity { Value = new float3(-1f, 0f, 0f) },
                speed,
                settings,
                contactBody,
                destination,
                ref arrival);
            Require(arrival.IsSettled &&
                    math.lengthsq(states[0].CurrentVelocity) <= 0.000001f &&
                    math.lengthsq(states[0].IndependentForce) <= 0.000001f,
                "Unit did not stop while waiting for its matching Flow Field request.");

            destination.OrderVersion = 1;
            job.Execute(
                Entity.Null,
                0,
                LocalTransform.FromPosition(new float3(1f, 0f, 1f)),
                velocity,
                speed,
                settings,
                contactBody,
                destination,
                ref arrival);
            Require(!arrival.IsSettled && states[0].IndependentForce.x > 0f,
                "Unit did not steer directly toward its assigned slot.");

            job.Execute(
                Entity.Null,
                0,
                LocalTransform.FromPosition(new float3(1.75f, 0f, 1f)),
                velocity,
                speed,
                settings,
                contactBody,
                destination,
                ref arrival);
            Require(arrival.IsSettled &&
                    math.lengthsq(states[0].IndependentForce) <= 0.000001f,
                "Unit did not settle independently at its assigned slot.");
        }
        finally
        {
            states.Dispose();
            footprints.Dispose();
            gridCells.Dispose();
        }
    }

    private static void ValidateMovementWithoutDiagnosticComponents()
    {
        var gridCells = new NativeArray<FlowFieldCell>(16, Allocator.TempJob);
        try
        {
            for (int i = 0; i < gridCells.Length; i++)
            {
                gridCells[i] = new FlowFieldCell
                {
                    Cost = 1,
                    IntegrationValue = 3,
                    BestDirectionIndex = 2
                };
            }

            using var world = new World("Diagnostic Optional Movement Validation", WorldFlags.Game);
            world.SetTime(new Unity.Core.TimeData(1d, 0.1f));
            EntityManager entityManager = world.EntityManager;
            Entity manager = entityManager.CreateEntity(
                typeof(FlowFieldGrid),
                typeof(FlowFieldRuntimeState),
                typeof(FlowFieldSettings),
                typeof(UnitContactSolverSettings));
            entityManager.SetComponentData(manager, new FlowFieldGrid
            {
                Grid = gridCells,
                GridOrigin = float3.zero,
                GridDimensions = new int2(4, 4),
                CellRadius = 0.5f
            });
            entityManager.SetComponentData(
                manager,
                new FlowFieldRuntimeState
                {
                    ActiveVersion = 1,
                    ActiveRequestVersion = 1
                });
            entityManager.SetComponentData(manager, new FlowFieldSettings
            {
                GridOrigin = float3.zero,
                GridDimensions = new int2(4, 4),
                CellRadius = 0.5f,
                SoftAvoidanceResponseRate = 1f,
                SoftAvoidanceShell = 0.1f,
                SettledSoftAvoidanceMultiplier = 1f,
                RvoTimeHorizon = 1f
            });
            entityManager.SetComponentData(manager, new UnitContactSolverSettings
            {
                SubstepCount = 1,
                IterationCount = 1,
                Compliance = 0f,
                PredictiveSkin = 0f,
                EnableDiagnostics = false,
                EnableFatAabbCache = false
            });

            Entity unit = entityManager.CreateEntity(
                typeof(LocalInstance),
                typeof(LocalTransform),
                typeof(Velocity),
                typeof(FlowArrivalState),
                typeof(UnitMoveSpeed),
                typeof(UnitMovementSettings),
                typeof(UnitContactBody),
                typeof(UnitMoveDestination));
            entityManager.SetComponentData(unit, new LocalInstance { Id = 1 });
            entityManager.SetComponentData(
                unit,
                LocalTransform.FromPosition(new float3(1f, 0f, 1f)));
            entityManager.SetComponentData(unit, new UnitMoveSpeed { Value = 2f });
            entityManager.SetComponentData(
                unit,
                new UnitMovementSettings { MaxForce = 20f, RotationSpeed = 10f });
            entityManager.SetComponentData(
                unit,
                new UnitContactBody { InverseMass = 1f });
            entityManager.SetComponentData(unit, new UnitMoveDestination
            {
                Position = new float3(3f, 0f, 1f),
                ArrivalRadius = 0.1f,
                DirectApproachIntegrationDistance = 0,
                OrderVersion = 1,
                IsActive = 1
            });

            LocalUnitFlowMovementSystem system =
                world.CreateSystemManaged<LocalUnitFlowMovementSystem>();
            system.Update();
            entityManager.CompleteAllTrackedJobs();

            Require(
                entityManager.GetComponentData<LocalTransform>(unit).Position.x > 1f,
                "Movement system did not run without diagnostic singleton components.");
        }
        finally
        {
            gridCells.Dispose();
        }
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
}
