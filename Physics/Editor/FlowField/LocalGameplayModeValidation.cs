using System;
using System.IO;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using Entities.Unit.System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Profiling;
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
using RTS.Gameplay.Physics;
using TMG.NFE_Tutorial;

namespace RTS.Unit.FlowField.Editor
{

/// <summary>
/// 本地 World、固定槽位与诊断可选路径的编辑器回归入口。
/// 标记文件只在程序集重载后消费，避免验证运行在半完成的编译状态。
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
        ValidateQueryProxyVersionContract();
        ValidateVersionedAttackAndTrackQueries();
        ValidateVersionedTriggerDamage();
        ValidateMovementWithoutDiagnosticComponents();
        ValidateCacheAndSolverOutputEquivalence();
        Debug.Log(
            "LOCAL_GAMEPLAY_VALIDATION_OK\n" +
            "move order: right-click snapshot consumed once, live selection unchanged=1\n" +
            "local spawn ids: 1,2\n" +
            "fixed slots: unique, walkable, assigned=1\n" +
            "per-slot movement: direct steer=1, settled=1\n" +
            "query proxy: crowd commit=9, physics publish=9\n" +
            "attack/track query: version match=1, stale source/target rejected=1\n" +
            "trigger damage: matching version accepted=1, stale rejected=1\n" +
            "diagnostics optional: movement=1, profiler recorder bound=1\n" +
            "cache/solver outputs: off=timestep=cross-frame, GS~=Jacobi, " +
            "Jacobi multi-contact incident=1\n"
        );
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
            entityManager.AddBuffer<MoveOrderSelectionElement>(managerEntity);
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
            entityManager.GetBuffer<MoveOrderSelectionElement>(managerEntity)
                .Add(new MoveOrderSelectionElement { Entity = unit });

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

            entityManager.SetComponentData(unit, new UnitSelected { Value = false });
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
                "Right-click selection snapshot did not receive a fixed destination.");
            Require(!entityManager.GetComponentData<UnitSelected>(unit).Value,
                "Applying a move order unexpectedly changed the live selection.");

            entityManager.GetBuffer<MoveOrderSelectionElement>(managerEntity)
                .Add(new MoveOrderSelectionElement { Entity = unit });
            entityManager.SetComponentData(unit, new UnitSelected { Value = true });
            entityManager.SetComponentData(
                managerEntity,
                new MoveOrder { TargetPosition = new float3(5f, 0f, 5f) });
            entityManager.SetComponentEnabled<MoveOrder>(managerEntity, true);
            system.Update();

            Require(entityManager.GetComponentData<UnitSelected>(unit).Value,
                "Applying a move order unexpectedly cleared the unit selection.");
            Require(!entityManager.IsComponentEnabled<MoveOrder>(managerEntity),
                "Second right-click snapshot was not consumed in one update.");
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
        var stepInputs =
            new NativeArray<CrowdPhysicsBodyInput>(1, Allocator.Temp);
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
            var job = new BuildCrowdMotionIntentJob
            {
                NavigationCells = gridCells,
                NavigationGrid = new FlowGridGeometry(
                    float3.zero, new int2(3, 3), 0.5f),
                ActiveRequestVersion = 1,
                StepInputs = stepInputs
            };
            var velocity = new Velocity { Value = float3.zero };
            var speed = new UnitMoveSpeed { Value = 2f };
            var settings = new UnitMovementSettings { MaxForce = 20f };
            var contactBody = new UnitContactBody { InverseMass = 1f };
            var shape = new CrowdDiscShape { Radius = 0.5f, Version = 1 };
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
                shape,
                destination,
                ref arrival);
            Require(arrival.IsSettled &&
                    math.lengthsq(stepInputs[0].Velocity) <= 0.000001f &&
                    math.lengthsq(stepInputs[0].SteeringVelocityError) <= 0.000001f,
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
                shape,
                destination,
                ref arrival);
            Require(!arrival.IsSettled &&
                    stepInputs[0].SteeringVelocityError.x > 0f,
                "Unit did not steer directly toward its assigned slot.");

            job.Execute(
                Entity.Null,
                0,
                LocalTransform.FromPosition(new float3(1.75f, 0f, 1f)),
                velocity,
                speed,
                settings,
                contactBody,
                shape,
                destination,
                ref arrival);
            Require(arrival.IsSettled &&
                    math.lengthsq(stepInputs[0].SteeringVelocityError) <= 0.000001f,
                "Unit did not settle independently at its assigned slot.");
        }
        finally
        {
            stepInputs.Dispose();
            gridCells.Dispose();
        }
    }

    private static void ValidateMovementWithoutDiagnosticComponents()
    {
        var physicsWorld = new PhysicsWorld(0, 0, 0);
        try
        {
            using var world = new World("Diagnostic Optional Movement Validation", WorldFlags.Game);
            world.SetTime(new Unity.Core.TimeData(1d, 0.1f));
            EntityManager entityManager = world.EntityManager;
            Entity manager = entityManager.CreateEntity(
                typeof(FlowFieldGlobalTarget),
                typeof(FlowFieldRuntimeState),
                typeof(FlowFieldCostState),
                typeof(FlowFieldSettings),
                typeof(UnitContactSolverSettings),
                typeof(RecalculateFlowFieldTag));
            Entity physicsWorldEntity =
                entityManager.CreateEntity(typeof(PhysicsWorldSingleton));
            entityManager.SetComponentData(
                physicsWorldEntity,
                new PhysicsWorldSingleton { PhysicsWorld = physicsWorld });
            entityManager.SetComponentData(
                manager,
                new FlowFieldGlobalTarget
                {
                    TargetPosition = new float3(3f, 0f, 1f)
                });
            entityManager.SetComponentData(
                manager,
                new FlowFieldCostState
                {
                    IsDirty = true,
                    CostVersion = 0
                });
            entityManager.SetComponentData(manager, new FlowFieldSettings
            {
                GridOrigin = float3.zero,
                GridDimensions = new int2(4, 4),
                CellRadius = 0.5f
            });
            entityManager.SetComponentData(manager, new UnitContactSolverSettings
            {
                SubstepCount = 1,
                IterationCount = 1,
                ContactPositionSolver = ContactPositionSolverMode.GaussSeidel,
                Compliance = 0f,
                PredictiveSkin = 0f,
                SoftAvoidanceResponseRate = 1f,
                SoftAvoidanceShell = 0.1f,
                SettledSoftAvoidanceMultiplier = 1f,
                RvoTimeHorizon = 1f,
                EnableDiagnostics = false,
                EnablePersistentContactCache = false
            });
            entityManager.SetComponentData(
                manager,
                new RecalculateFlowFieldTag { RequestVersion = 1 });
            entityManager.SetComponentEnabled<RecalculateFlowFieldTag>(
                manager,
                true);

            FlowFieldBakeSystem bakeSystem =
                world.CreateSystemManaged<FlowFieldBakeSystem>();
            bakeSystem.Update();
            entityManager.SetComponentData(
                physicsWorldEntity,
                new PhysicsWorldSingleton { PhysicsWorld = physicsWorld });
            bakeSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            bakeSystem.Update();

            Require(
                entityManager.GetComponentData<FlowFieldRuntimeState>(manager)
                    .ActiveVersion == 1,
                "Flow Field environment snapshot did not publish.");

            Entity unit = entityManager.CreateEntity(
                typeof(LocalInstance),
                typeof(LocalTransform),
                typeof(Velocity),
                typeof(FlowArrivalState),
                typeof(UnitMoveSpeed),
                typeof(UnitMovementSettings),
                typeof(UnitContactBody),
                typeof(CrowdDiscShape),
                typeof(CrowdQueryProxy),
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
            entityManager.SetComponentData(
                unit,
                new CrowdDiscShape { Radius = 0.5f, Version = 1 });
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
            using ProfilerRecorder simulationUpdateRecorder =
                ProfilerRecorder.StartNew(
                    ProfilerCategory.Scripts,
                    "RTS.Simulation.Update",
                    8);
            system.Update();
            entityManager.CompleteAllTrackedJobs();

            Require(
                entityManager.GetComponentData<LocalTransform>(unit).Position.x > 1f,
                "Movement system did not run without diagnostic singleton components.");
            Require(
                simulationUpdateRecorder.Valid,
                "Profiler recorder could not bind RTS.Simulation.Update.");

#if RTS_CONTACT_DIAGNOSTICS
            Entity legacySelectionA =
                entityManager.CreateEntity(typeof(ContactDiagnosticSelection));
            Entity legacySelectionB =
                entityManager.CreateEntity(typeof(ContactDiagnosticSelection));
            entityManager.SetComponentData(
                legacySelectionA,
                new ContactDiagnosticSelection { SelectedEntity = unit });
            entityManager.SetComponentData(
                legacySelectionB,
                new ContactDiagnosticSelection { SelectedEntity = unit });

            world.SetTime(new Unity.Core.TimeData(1.1d, 0.1f));
            system.Update();
            entityManager.CompleteAllTrackedJobs();
            Require(
                SimulationDebuggerRuntime.SelectedEntityFor(
                    SimulationDebuggerWorldIdentity.FromSequenceNumber(
                        world.Unmanaged.SequenceNumber)) == unit,
                "Duplicate compatible legacy selections were not bridged to the World runtime.");
#endif
        }
        finally
        {
            physicsWorld.Dispose();
        }
    }

    private readonly struct MovementScenarioResult
    {
        public readonly float3 FirstPosition;
        public readonly float3 SecondPosition;
        public readonly float3 ThirdPosition;
        public readonly float3 FourthPosition;
        public readonly float3 FirstVelocity;
        public readonly float3 SecondVelocity;
        public readonly float3 ThirdVelocity;
        public readonly float3 FourthVelocity;

        public MovementScenarioResult(
            float3 firstPosition,
            float3 secondPosition,
            float3 thirdPosition,
            float3 fourthPosition,
            float3 firstVelocity,
            float3 secondVelocity,
            float3 thirdVelocity,
            float3 fourthVelocity)
        {
            FirstPosition = firstPosition;
            SecondPosition = secondPosition;
            ThirdPosition = thirdPosition;
            FourthPosition = fourthPosition;
            FirstVelocity = firstVelocity;
            SecondVelocity = secondVelocity;
            ThirdVelocity = thirdVelocity;
            FourthVelocity = fourthVelocity;
        }
    }

    private static void ValidateCacheAndSolverOutputEquivalence()
    {
        MovementScenarioResult cacheOff = RunMovementScenario(
            "Cache OFF GS",
            enableTimestepCache: false,
            enablePersistentCache: false,
            ContactPositionSolverMode.GaussSeidel);
        MovementScenarioResult timestepCache = RunMovementScenario(
            "Timestep Cache GS",
            enableTimestepCache: true,
            enablePersistentCache: false,
            ContactPositionSolverMode.GaussSeidel);
        MovementScenarioResult crossFrameCache = RunMovementScenario(
            "Cross Frame Cache GS",
            enableTimestepCache: true,
            enablePersistentCache: true,
            ContactPositionSolverMode.GaussSeidel);
        MovementScenarioResult jacobi = RunMovementScenario(
            "Cache OFF Jacobi",
            enableTimestepCache: false,
            enablePersistentCache: false,
            ContactPositionSolverMode.Jacobi);
        MovementScenarioResult multiContactJacobi = RunMovementScenario(
            "Cross Frame Cache Jacobi Multi Contact",
            enableTimestepCache: true,
            enablePersistentCache: true,
            ContactPositionSolverMode.Jacobi,
            multiContact: true);

        RequireScenarioClose(
            cacheOff,
            timestepCache,
            0.0005f,
            "Timestep cache output diverged from Cache OFF.");
        RequireScenarioClose(
            cacheOff,
            crossFrameCache,
            0.0005f,
            "Cross-frame cache output diverged from Cache OFF.");
        RequireScenarioClose(
            cacheOff,
            jacobi,
            0.02f,
            "Jacobi output diverged from the shared GS pipeline.");
        Require(
            math.distance(
                cacheOff.FirstPosition,
                cacheOff.SecondPosition) >= 0.9f,
            "The equivalence scenario did not preserve the hard-contact separation.");
        Require(
            math.distance(
                multiContactJacobi.SecondPosition,
                multiContactJacobi.ThirdPosition) >= 0.9f &&
            math.distance(
                multiContactJacobi.ThirdPosition,
                multiContactJacobi.FourthPosition) >= 0.9f,
            "The Jacobi multi-contact incident chain did not preserve separation.");
    }

    private static MovementScenarioResult RunMovementScenario(
        string name,
        bool enableTimestepCache,
        bool enablePersistentCache,
        ContactPositionSolverMode solverMode,
        bool multiContact = false)
    {
        var physicsWorld = new PhysicsWorld(0, 0, 0);
        try
        {
            using var world = new World(name, WorldFlags.Game);
            EntityManager entityManager = world.EntityManager;
            Entity manager = entityManager.CreateEntity(
                typeof(FlowFieldGlobalTarget),
                typeof(FlowFieldRuntimeState),
                typeof(FlowFieldCostState),
                typeof(FlowFieldSettings),
                typeof(UnitContactSolverSettings),
                typeof(RecalculateFlowFieldTag));
            Entity physicsWorldEntity =
                entityManager.CreateEntity(typeof(PhysicsWorldSingleton));
            entityManager.SetComponentData(
                physicsWorldEntity,
                new PhysicsWorldSingleton { PhysicsWorld = physicsWorld });
            entityManager.SetComponentData(
                manager,
                new FlowFieldGlobalTarget
                {
                    TargetPosition = new float3(6f, 0f, 2f)
                });
            entityManager.SetComponentData(
                manager,
                new FlowFieldCostState { IsDirty = true });
            entityManager.SetComponentData(manager, new FlowFieldSettings
            {
                GridOrigin = float3.zero,
                GridDimensions = new int2(8, 6),
                CellRadius = 0.5f
            });
            entityManager.SetComponentData(manager, new UnitContactSolverSettings
            {
                SubstepCount = 2,
                IterationCount = 4,
                ContactPositionSolver = solverMode,
                Compliance = 0f,
                PredictiveSkin = 0.05f,
                SoftAvoidanceResponseRate = 0f,
                SoftAvoidanceShell = 0f,
                SettledSoftAvoidanceMultiplier = 1f,
                RvoTimeHorizon = 1f,
                EnablePredictivePairGeneration = true,
                EnablePredictiveContacts = true,
                EnableDiagnostics = false,
                EnableTimestepContactSetCache = enableTimestepCache,
                EnablePersistentContactCache = enablePersistentCache,
                PersistentGuardEnvelopeMargin = 0.25f,
                TimestepContactMargin = 0.05f
            });
            entityManager.SetComponentData(
                manager,
                new RecalculateFlowFieldTag { RequestVersion = 1 });
            entityManager.SetComponentEnabled<RecalculateFlowFieldTag>(
                manager,
                true);

            FlowFieldBakeSystem bakeSystem =
                world.CreateSystemManaged<FlowFieldBakeSystem>();
            bakeSystem.Update();
            entityManager.SetComponentData(
                physicsWorldEntity,
                new PhysicsWorldSingleton { PhysicsWorld = physicsWorld });
            bakeSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            bakeSystem.Update();
            Require(
                entityManager.GetComponentData<FlowFieldRuntimeState>(manager)
                    .ActiveVersion == 1,
                $"{name}: Flow Field environment snapshot did not publish.");

            Entity first = CreateScenarioUnit(
                entityManager,
                1,
                new float3(2f, 0f, 2f));
            Entity second = CreateScenarioUnit(
                entityManager,
                2,
                new float3(2.7f, 0f, 2f));
            Entity third = Entity.Null;
            Entity fourth = Entity.Null;
            if (multiContact)
            {
                third = CreateScenarioUnit(
                    entityManager,
                    3,
                    new float3(3.4f, 0f, 2f));
                fourth = CreateScenarioUnit(
                    entityManager,
                    4,
                    new float3(4.1f, 0f, 2f));
            }
            LocalUnitFlowMovementSystem system =
                world.CreateSystemManaged<LocalUnitFlowMovementSystem>();

            for (int step = 0; step < 4; step++)
            {
                world.SetTime(new Unity.Core.TimeData(
                    1d + step * 0.05d,
                    0.05f));
                system.Update();
                entityManager.CompleteAllTrackedJobs();
            }

            return new MovementScenarioResult(
                entityManager.GetComponentData<LocalTransform>(first).Position,
                entityManager.GetComponentData<LocalTransform>(second).Position,
                multiContact
                    ? entityManager.GetComponentData<LocalTransform>(third)
                        .Position
                    : float3.zero,
                multiContact
                    ? entityManager.GetComponentData<LocalTransform>(fourth)
                        .Position
                    : float3.zero,
                entityManager.GetComponentData<Velocity>(first).Value,
                entityManager.GetComponentData<Velocity>(second).Value,
                multiContact
                    ? entityManager.GetComponentData<Velocity>(third).Value
                    : float3.zero,
                multiContact
                    ? entityManager.GetComponentData<Velocity>(fourth).Value
                    : float3.zero);
        }
        finally
        {
            physicsWorld.Dispose();
        }
    }

    private static Entity CreateScenarioUnit(
        EntityManager entityManager,
        int id,
        float3 position)
    {
        Entity unit = entityManager.CreateEntity(
            typeof(LocalInstance),
            typeof(LocalTransform),
            typeof(Velocity),
            typeof(FlowArrivalState),
            typeof(UnitMoveSpeed),
            typeof(UnitMovementSettings),
            typeof(UnitContactBody),
            typeof(CrowdDiscShape),
            typeof(CrowdQueryProxy),
            typeof(UnitMoveDestination));
        entityManager.SetComponentData(unit, new LocalInstance { Id = id });
        entityManager.SetComponentData(
            unit,
            LocalTransform.FromPosition(position));
        entityManager.SetComponentData(
            unit,
            new UnitMoveSpeed { Value = 2f });
        entityManager.SetComponentData(
            unit,
            new UnitMovementSettings
            {
                MaxForce = 20f,
                RotationSpeed = 10f
            });
        entityManager.SetComponentData(
            unit,
            new UnitContactBody { InverseMass = 1f });
        entityManager.SetComponentData(
            unit,
            new CrowdDiscShape { Radius = 0.5f, Version = 1 });
        entityManager.SetComponentData(unit, new UnitMoveDestination
        {
            Position = new float3(6f, 0f, 2f),
            ArrivalRadius = 0.1f,
            DirectApproachIntegrationDistance = 16,
            OrderVersion = 1,
            IsActive = 1
        });
        return unit;
    }

    private static void RequireScenarioClose(
        MovementScenarioResult expected,
        MovementScenarioResult actual,
        float tolerance,
        string message)
    {
        bool positionsMatch =
            math.distance(expected.FirstPosition, actual.FirstPosition) <= tolerance &&
            math.distance(expected.SecondPosition, actual.SecondPosition) <= tolerance &&
            math.distance(expected.ThirdPosition, actual.ThirdPosition) <= tolerance &&
            math.distance(expected.FourthPosition, actual.FourthPosition) <= tolerance;
        bool velocitiesMatch =
            math.distance(expected.FirstVelocity, actual.FirstVelocity) <= tolerance &&
            math.distance(expected.SecondVelocity, actual.SecondVelocity) <= tolerance &&
            math.distance(expected.ThirdVelocity, actual.ThirdVelocity) <= tolerance &&
            math.distance(expected.FourthVelocity, actual.FourthVelocity) <= tolerance;
        Require(positionsMatch && velocitiesMatch, message);
    }

    private static void ValidateQueryProxyVersionContract()
    {
        var results = new NativeArray<CrowdBodyResult>(1, Allocator.TempJob);
        try
        {
            results[0] = new CrowdBodyResult
            {
                Position = new float3(2f, 0f, 3f),
                Rotation = quaternion.identity,
                Velocity = new float3(1f, 0f, 0f)
            };
            var apply = new ApplyFlowMovementJob
            {
                Results = results.AsReadOnly(),
                CrowdStepVersion = 9
            };
            LocalTransform transform = LocalTransform.Identity;
            Velocity velocity = default;
            CrowdQueryProxy proxy = new CrowdQueryProxy
            {
                CrowdStepVersion = 3,
                ProxyVersion = 3
            };
            apply.Execute(0, ref transform, ref velocity, ref proxy);
            Require(
                proxy.CrowdStepVersion == 9 && proxy.ProxyVersion == 3,
                "Crowd commit did not advance only the ECS Transform version.");

            using var world =
                new World("Crowd Query Proxy Publication Validation", WorldFlags.Game);
            EntityManager entityManager = world.EntityManager;
            Entity physicsWorldEntity =
                entityManager.CreateEntity(typeof(PhysicsWorldSingleton));
            entityManager.SetComponentData(
                physicsWorldEntity,
                new PhysicsWorldSingleton());
            Entity unit = entityManager.CreateEntity(typeof(CrowdQueryProxy));
            entityManager.SetComponentData(unit, proxy);

            CrowdQueryProxyPublicationSystem publishSystem =
                world.CreateSystemManaged<CrowdQueryProxyPublicationSystem>();
            publishSystem.Update();
            entityManager.CompleteAllTrackedJobs();

            Require(
                entityManager.GetComponentData<CrowdQueryProxy>(unit)
                    .ProxyVersion == 9,
                "Post-BuildPhysicsWorld publication did not advance ProxyVersion.");
        }
        finally
        {
            results.Dispose();
        }
    }

    private static void ValidateVersionedAttackAndTrackQueries()
    {
        using var world =
            new World("Versioned Attack Track Query Validation", WorldFlags.Game);
        EntityManager entityManager = world.EntityManager;
        Entity source = entityManager.CreateEntity(
            typeof(LocalTransform),
            typeof(CrowdQueryProxy),
            typeof(IsUserUnitTag),
            typeof(AttackDistance),
            typeof(AttackEntity),
            typeof(TrackDistance),
            typeof(TrackEntity));
        Entity target = entityManager.CreateEntity(
            typeof(LocalTransform),
            typeof(CrowdQueryProxy));
        entityManager.SetComponentData(
            source,
            LocalTransform.FromPosition(float3.zero));
        entityManager.SetComponentData(
            target,
            LocalTransform.FromPosition(new float3(1.5f, 0f, 0f)));
        entityManager.SetComponentData(
            source,
            new CrowdQueryProxy
            {
                CrowdStepVersion = 7,
                ProxyVersion = 7
            });
        entityManager.SetComponentData(
            target,
            new CrowdQueryProxy
            {
                CrowdStepVersion = 7,
                ProxyVersion = 7
            });
        entityManager.SetComponentData(source, new AttackDistance { Distance = 3f });
        entityManager.SetComponentData(source, new TrackDistance { Distance = 3f });

        CollisionFilter unitBodyFilter = new CollisionFilter
        {
            BelongsTo = CrowdQueryCollisionFilters.Unit,
            CollidesWith =
                CrowdQueryCollisionFilters.Ground |
                CrowdQueryCollisionFilters.Obstacle,
            GroupIndex = 0
        };
        Require(
            !CollisionFilter.IsCollisionEnabled(
                unitBodyFilter,
                unitBodyFilter),
            "Unity Physics Unit-Unit response filter is still enabled.");
        Require(
            CollisionFilter.IsCollisionEnabled(
                CrowdQueryCollisionFilters.UnitOverlap,
                unitBodyFilter),
            "Unit overlap query cannot see query proxies.");

        using BlobAssetReference<Unity.Physics.Collider> collider =
            Unity.Physics.SphereCollider.Create(
                new SphereGeometry
                {
                    Center = float3.zero,
                    Radius = 0.5f
                },
                unitBodyFilter);
        var physicsWorld = new PhysicsWorld(2, 0, 0);
        try
        {
            NativeArray<RigidBody> staticBodies = physicsWorld.StaticBodies;
            staticBodies[0] = new RigidBody
            {
                Entity = source,
                Collider = collider,
                WorldFromBody = new RigidTransform(
                    quaternion.identity,
                    float3.zero),
                Scale = 1f
            };
            staticBodies[1] = new RigidBody
            {
                Entity = target,
                Collider = collider,
                WorldFromBody = new RigidTransform(
                    quaternion.identity,
                    new float3(1.5f, 0f, 0f)),
                Scale = 1f
            };
            physicsWorld.UpdateIndexMaps();
            physicsWorld.CollisionWorld.BuildBroadphase(
                ref physicsWorld,
                0f,
                float3.zero,
                buildStaticTree: true);
            entityManager.CreateSingleton(new PhysicsWorldSingleton
            {
                PhysicsWorld = physicsWorld
            });

            SystemHandle attackSystem =
                world.GetOrCreateSystem<UnitAttackTriggerSystem>();
            SystemHandle trackSystem =
                world.GetOrCreateSystem<TrackTriggerSystem>();
            attackSystem.Update(world.Unmanaged);
            trackSystem.Update(world.Unmanaged);

            AttackEntity attack =
                entityManager.GetComponentData<AttackEntity>(source);
            TrackEntity track =
                entityManager.GetComponentData<TrackEntity>(source);
            Require(
                attack.Entity == target &&
                attack.QueryProxyVersion == 7 &&
                track.Entity == target &&
                track.QueryProxyVersion == 7,
                "Attack/Track query did not publish the matching proxy version.");

            CrowdQueryProxy staleTarget =
                entityManager.GetComponentData<CrowdQueryProxy>(target);
            staleTarget.ProxyVersion = 6;
            entityManager.SetComponentData(target, staleTarget);
            attackSystem.Update(world.Unmanaged);
            trackSystem.Update(world.Unmanaged);

            attack = entityManager.GetComponentData<AttackEntity>(source);
            track = entityManager.GetComponentData<TrackEntity>(source);
            Require(
                attack.Entity == Entity.Null &&
                attack.QueryProxyVersion == 7 &&
                track.Entity == Entity.Null &&
                track.QueryProxyVersion == 7,
                "Attack/Track query consumed a stale target proxy.");

            staleTarget.ProxyVersion = 7;
            entityManager.SetComponentData(target, staleTarget);
            CrowdQueryProxy staleSource =
                entityManager.GetComponentData<CrowdQueryProxy>(source);
            staleSource.CrowdStepVersion = 8;
            entityManager.SetComponentData(source, staleSource);
            attackSystem.Update(world.Unmanaged);
            trackSystem.Update(world.Unmanaged);

            attack = entityManager.GetComponentData<AttackEntity>(source);
            track = entityManager.GetComponentData<TrackEntity>(source);
            Require(
                attack.Entity == Entity.Null &&
                attack.QueryProxyVersion == 0 &&
                track.Entity == Entity.Null &&
                track.QueryProxyVersion == 0,
                "Attack/Track query mixed a newer ECS source with an older PhysicsWorld.");
        }
        finally
        {
            physicsWorld.Dispose();
        }
    }

    private static void ValidateVersionedTriggerDamage()
    {
        DamageBufferElement damage =
            DamageOnTriggerJob.CreateDamageElement(11, 7);
        Require(
            damage.Value == 11 &&
            CalculateFrameDamageSystem.IsDamageVersionCurrent(
                damage,
                hasQueryProxy: true,
                currentProxyVersion: 7) &&
            !CalculateFrameDamageSystem.IsDamageVersionCurrent(
                damage,
                hasQueryProxy: true,
                currentProxyVersion: 8),
            "Trigger damage consumer did not reject a stale query proxy version.");
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
