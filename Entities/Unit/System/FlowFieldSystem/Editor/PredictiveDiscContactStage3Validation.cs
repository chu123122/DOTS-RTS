using System;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

public static class PredictiveDiscContactStage3Validation
{
    // 该验证同时覆盖 Predictive 功能开关和逐 iteration 诊断输出。
    private const float PositionTolerance = 0.0001f;
    private static string ValidationRequestPath => Path.GetFullPath(Path.Combine(
        Application.dataPath,
        "../Temp/RunPredictiveDiscContactStage3Validation"));

    [InitializeOnLoadMethod]
    private static void RunRequestedValidationAfterReload()
    {
        if (!File.Exists(ValidationRequestPath))
            return;

        File.Delete(ValidationRequestPath);
        EditorApplication.delayCall += Run;
    }

    [MenuItem("RTS/Validation/Predictive Disc Contacts Stage 3 %#F12")]
    public static void Run()
    {
        ValidateSeparatedStationaryDiscs();
        ValidateRegularOverlap();
        ScenarioResult crossing = ValidateHighSpeedSideExchange();
        ScenarioResult predictiveDisabled = ValidatePredictiveToggle();
        ScenarioResult tangent = ValidateTangentialNearMiss();
        ScenarioResult chain = ValidatePrebuiltChainContact();
        ScenarioResult softAvoidance = ValidateSoftAvoidancePerSubstep();
        (ScenarioResult wallOneIteration, ScenarioResult wallEightIterations) =
            ValidateWallAndUnitConstraintsIterateTogether();
        ScenarioResult shadow = ValidateShadowNeighborCacheProbe();
        (ScenarioResult oneIteration, ScenarioResult eightIterations) =
            ValidateIterationResidualReduction();

        Debug.Log(
            "STAGE3_VALIDATION_OK\n" +
            $"crossing: predictive={crossing.Statistics.PredictivePairCount}, " +
            $"activated={crossing.Statistics.PredictiveActivatedCount}\n" +
            $"predictive disabled: potential={predictiveDisabled.Statistics.PotentialPredictivePairCount}, " +
            $"active predictive={predictiveDisabled.Statistics.PredictivePairCount}\n" +
            $"tangent: contacts={tangent.Statistics.ContactPairCount}, " +
            $"unactivated={tangent.Statistics.UnactivatedPairCount}\n" +
            $"chain: active={chain.Statistics.ActiveConstraintCount}\n" +
            $"soft avoidance: evaluations={softAvoidance.Statistics.SoftAvoidanceEvaluationCount}, " +
            $"time={softAvoidance.Statistics.SoftAvoidanceNanoseconds}ns\n" +
            $"wall->unit: B.x {wallOneIteration.Positions[1].x:F6} -> " +
            $"{wallEightIterations.Positions[1].x:F6}\n" +
            $"shadow prev hit/miss={shadow.ShadowStatistics.PreviousFramePairHitCount}/" +
            $"{shadow.ShadowStatistics.PreviousFramePairMissCount}, " +
            $"current wall escapes={shadow.ShadowStatistics.CurrentFrameWallDrivenEscapeBodyCount}\n" +
            $"iterations 1->8: maxPenetration " +
            $"{oneIteration.Statistics.MaxPenetration:F6} -> " +
            $"{eightIterations.Statistics.MaxPenetration:F6}\n" +
            $"timing(ns): pair={eightIterations.Statistics.PairGenerationNanoseconds}, " +
            $"avgIteration={eightIterations.Statistics.AverageIterationNanoseconds}, " +
            $"solver={eightIterations.Statistics.SolverNanoseconds}");
    }

    private static void ValidateSeparatedStationaryDiscs()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(0, 0, 0), float3.zero, 0.25f),
            CreateBody(new float3(2, 0, 0), float3.zero, 0.25f)
        };

        ScenarioResult result = RunScenario(bodies, iterationCount: 4, skin: 0.05f);
        Require(math.distance(result.Positions[0], bodies[0].CurrentPosition) <= PositionTolerance,
            "Separated disc A moved unexpectedly.");
        Require(math.distance(result.Positions[1], bodies[1].CurrentPosition) <= PositionTolerance,
            "Separated disc B moved unexpectedly.");
        Require(result.Statistics.ActiveConstraintCount == 0,
            "Separated stationary discs activated a contact.");
    }

    private static void ValidateRegularOverlap()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(0, 0, 0), float3.zero, 0.25f),
            CreateBody(new float3(0.2f, 0, 0), float3.zero, 0.25f)
        };

        ScenarioResult result = RunScenario(bodies, iterationCount: 4, skin: 0.05f);
        float finalDistance = math.distance(result.Positions[0], result.Positions[1]);
        float center = (result.Positions[0].x + result.Positions[1].x) * 0.5f;
        Require(finalDistance >= 0.5f - PositionTolerance,
            "Regular overlap was not separated to the radius sum.");
        Require(math.abs(center - 0.1f) <= PositionTolerance,
            "Equal-mass regular contact was not symmetric.");
        Require(result.Statistics.ActiveConstraintCount == 1 &&
                result.Statistics.ActualGeneratedPairCount == 1 &&
                result.Statistics.PredictiveGeneratedPairCount == 0 &&
                result.Statistics.PredictivePairCount == 0,
            "Regular overlap was not classified as one active radial contact.");
        Require(result.Statistics.MaxVelocityChange > 0f &&
                result.Statistics.MaxContactPositionCorrection > 0f,
            "Contact correction and velocity-change diagnostics were not recorded.");
    }

    private static ScenarioResult ValidateHighSpeedSideExchange()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(-1, 0, 0), new float3(2, 0, 0), 0.25f),
            CreateBody(new float3(1, 0, 0), new float3(-2, 0, 0), 0.25f)
        };

        // 普通终点距离检测看到的仍是 2，无法发现中途穿越。
        float endpointDistanceWithoutPredictive = math.distance(
            bodies[0].CurrentPosition + bodies[0].CurrentVelocity,
            bodies[1].CurrentPosition + bodies[1].CurrentVelocity);
        Require(endpointDistanceWithoutPredictive > 0.5f,
            "High-speed validation setup does not exchange and re-separate.");

        ScenarioResult result = RunScenario(bodies, iterationCount: 4, skin: 0.05f);
        float3 initialNormal = math.normalize(
            bodies[0].CurrentPosition - bodies[1].CurrentPosition);
        float projectedSeparation = math.dot(
            result.Positions[0] - result.Positions[1],
            initialNormal);
        Require(projectedSeparation >= 0.5f - PositionTolerance,
            "Predictive separation constraint allowed the discs to exchange sides.");
        Require(result.Statistics.PredictivePairCount == 1 &&
                result.Statistics.PredictiveActivatedCount == 1 &&
                result.Statistics.PredictiveGeneratedPairCount == 1,
            "High-speed crossing was not generated and activated as Predictive.");
        return result;
    }

    private static ScenarioResult ValidateTangentialNearMiss()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(0, 0, 0), float3.zero, 0.25f),
            CreateBody(new float3(-1, 0, 0.6f), new float3(2, 0, 0), 0.25f)
        };

        ScenarioResult result = RunScenario(bodies, iterationCount: 4, skin: 0.15f);
        float3 expectedEnd = new float3(1, 0, 0.6f);
        Require(math.distance(result.Positions[1], expectedEnd) <= PositionTolerance,
            "Tangential skin-only Pair produced an unnecessary correction.");
        Require(result.Statistics.PredictiveGeneratedPairCount == 1 &&
                result.Statistics.PredictivePairCount == 0,
            "Tangential near miss was not retained as a non-blocking generated Pair.");
        Require(result.Statistics.ContactPairCount == 1 &&
                result.Statistics.UnactivatedPairCount == 1,
            "Tangential skin candidate was not reported as unactivated.");
        return result;
    }

    private static ScenarioResult ValidatePredictiveToggle()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(-1, 0, 0), new float3(2, 0, 0), 0.25f),
            CreateBody(new float3(1, 0, 0), new float3(-2, 0, 0), 0.25f)
        };

        ScenarioResult result = RunScenario(
            bodies,
            iterationCount: 4,
            skin: 0.05f,
            enablePredictiveContacts: false);
        Require(result.Positions[0].x > result.Positions[1].x,
            "Disabling Predictive Contact did not restore the endpoint-distance miss baseline.");
        Require(result.Statistics.PredictiveGeneratedPairCount == 1 &&
                result.Statistics.PotentialPredictivePairCount == 1 &&
                result.Statistics.PredictivePairCount == 0,
            "Predictive toggle did not preserve Pair discovery while disabling side protection.");
        return result;
    }

    private static ScenarioResult ValidateSoftAvoidancePerSubstep()
    {
        FlowMovementFrameState bodyA = CreateBody(new float3(-0.4f, 0, 0), float3.zero, 0.1f);
        FlowMovementFrameState bodyB = CreateBody(new float3(0.4f, 0, 0), float3.zero, 0.1f);
        bodyA.MoveSpeed = 1f;
        bodyB.MoveSpeed = 1f;
        bodyA.MaxForce = 10f;
        bodyB.MaxForce = 10f;
        FlowMovementFrameState[] bodies = { bodyA, bodyB };

        ScenarioResult result = RunScenario(
            bodies,
            iterationCount: 1,
            skin: 0f,
            substepCount: 4,
            softAvoidanceWeight: 1f,
            softAvoidanceRadius: 1f);
        Require(result.Statistics.SoftAvoidanceEvaluationCount == 4,
            "Soft avoidance was not recomputed once per substep.");
        Require(math.distance(result.Positions[0], result.Positions[1]) > 0.8f,
            "Per-substep soft avoidance did not separate nearby units.");
        return result;
    }

    private static ScenarioResult ValidatePrebuiltChainContact()
    {
        // Pair(0,1) 先检查且尚未重叠；Pair(1,2) 随后把 Body1 推向 Body0。
        // 下一轮 iteration 必须能激活已提前生成的 Pair(0,1)。
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(1.85f, 0, 0), float3.zero, 0.5f),
            CreateBody(new float3(0.8f, 0, 0), float3.zero, 0.5f),
            CreateBody(new float3(0, 0, 0), float3.zero, 0.5f)
        };

        ScenarioResult result = RunScenario(bodies, iterationCount: 4, skin: 0.1f);
        Require(result.Statistics.ContactPairCount >= 2,
            "Chain scenario did not prebuild both neighbor constraints.");
        Require(result.Statistics.ActiveConstraintCount >= 2,
            "Prebuilt B-C Pair did not activate after A-B correction.");
        return result;
    }

    private static (ScenarioResult, ScenarioResult) ValidateIterationResidualReduction()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(0, 0, 0), float3.zero, 0.5f),
            CreateBody(new float3(0.7f, 0, 0), float3.zero, 0.5f),
            CreateBody(new float3(1.4f, 0, 0), float3.zero, 0.5f),
            CreateBody(new float3(2.1f, 0, 0), float3.zero, 0.5f)
        };

        ScenarioResult oneIteration = RunScenario(bodies, iterationCount: 1, skin: 0.05f);
        ScenarioResult eightIterations = RunScenario(bodies, iterationCount: 8, skin: 0.05f);
        Require(eightIterations.Statistics.MaxPenetration <
                oneIteration.Statistics.MaxPenetration,
            "Increasing iterations did not reduce maximum dense-contact penetration.");
        Require(eightIterations.Statistics.AveragePenetration <=
                oneIteration.Statistics.AveragePenetration,
            "Increasing iterations increased average dense-contact penetration.");
        Require(eightIterations.IterationDiagnostics.Length == 8,
            "Stage 3 diagnostics did not record one residual sample per iteration.");
        return (oneIteration, eightIterations);
    }

    private static (ScenarioResult, ScenarioResult)
        ValidateWallAndUnitConstraintsIterateTogether()
    {
        // A 起初未和 B 重叠，但位于墙壁硬半径内。Pair(A,B) 由 skin 提前生成，
        // 墙壁先推 A，随后同一轮单位约束应激活并把 B 向右推开。
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(1.4f, 0, 0.5f), float3.zero, 0.5f),
            CreateBody(new float3(2.42f, 0, 0.5f), float3.zero, 0.5f)
        };

        ScenarioResult oneIteration = RunScenario(
            bodies,
            iterationCount: 1,
            skin: 0.1f,
            includeWall: true);
        ScenarioResult eightIterations = RunScenario(
            bodies,
            iterationCount: 8,
            skin: 0.1f,
            includeWall: true);

        Require(oneIteration.Statistics.TotalWallPositionCorrection > 0f &&
                oneIteration.Statistics.ActiveConstraintCount >= 1,
            "Wall correction did not activate the prebuilt unit Pair in the same iteration.");
        Require(oneIteration.Positions[1].x > bodies[1].CurrentPosition.x,
            "Wall-to-unit correction was not propagated to the neighboring body.");
        Require(eightIterations.Positions[1].x > oneIteration.Positions[1].x,
            "Additional unified iterations did not continue resolving the wall-unit chain.");
        Require(eightIterations.IterationDiagnostics[0].TotalWallPositionCorrection > 0f,
            "Per-iteration diagnostics did not record the wall projection.");
        return (oneIteration, eightIterations);
    }

    private static ScenarioResult ValidateShadowNeighborCacheProbe()
    {
        FlowMovementFrameState[] denseBodies =
        {
            CreateBody(new float3(0f, 0, 0f), float3.zero, 0.5f),
            CreateBody(new float3(0.9f, 0, 0f), float3.zero, 0.5f),
            CreateBody(new float3(1.8f, 0, 0f), float3.zero, 0.5f),
            CreateBody(new float3(2.7f, 0, 0f), float3.zero, 0.5f)
        };
        var previousProxies = new NativeList<ShadowFatBodyProxy>(Allocator.TempJob);
        var previousPairs = new NativeList<ShadowEntityPair>(Allocator.TempJob);

        ScenarioResult secondFrame;
        try
        {
            RunScenarioWithCache(
                denseBodies,
                iterationCount: 4,
                skin: 0.05f,
                enablePredictiveContacts: true,
                includeWall: false,
                substepCount: 2,
                enableShadowTest: true,
                shadowMargin: 0.25f,
                previousProxies: previousProxies,
                previousPairs: previousPairs);
            secondFrame = RunScenarioWithCache(
                denseBodies,
                iterationCount: 4,
                skin: 0.05f,
                enablePredictiveContacts: true,
                includeWall: false,
                substepCount: 2,
                enableShadowTest: true,
                shadowMargin: 0.25f,
                previousProxies: previousProxies,
                previousPairs: previousPairs);
        }
        finally
        {
            previousPairs.Dispose();
            previousProxies.Dispose();
        }

        Require(secondFrame.ShadowStatistics.PreviousFrameCacheAvailable != 0 &&
                secondFrame.ShadowStatistics.PreviousFrameCheckCount == 1,
            "Shadow Test did not reuse the previous frame Fat AABB neighbor list.");
        Require(secondFrame.ShadowStatistics.PreviousFramePairMissCount == 0 &&
                secondFrame.ShadowStatistics.CurrentFramePairMissCount == 0,
            "Stable dense contacts were not covered by the shadow neighbor lists.");
        Require(secondFrame.ShadowStatistics.CurrentFrameCheckCount == 1,
            "Shadow Test did not compare the first-substep cache with the later substep.");

        FlowMovementFrameState[] wallBodies =
        {
            CreateBody(new float3(1.4f, 0, 0.5f), float3.zero, 0.5f),
            CreateBody(new float3(2.42f, 0, 0.5f), float3.zero, 0.5f)
        };
        ScenarioResult shadowOff = RunScenario(
            wallBodies,
            iterationCount: 8,
            skin: 0.1f,
            includeWall: true,
            enableShadowTest: false);
        ScenarioResult shadowOn = RunScenario(
            wallBodies,
            iterationCount: 8,
            skin: 0.1f,
            includeWall: true,
            enableShadowTest: true,
            shadowMargin: 0.001f);
        for (int i = 0; i < shadowOff.Positions.Length; i++)
        {
            Require(math.distance(shadowOff.Positions[i], shadowOn.Positions[i]) <=
                    PositionTolerance,
                "Enabling Shadow Test changed an authoritative solver position.");
        }
        Require(shadowOn.ShadowStatistics.CurrentFrameWallDrivenEscapeBodyCount > 0,
            "Small shadow margin did not expose the wall-driven Fat AABB escape.");
        return new ScenarioResult
        {
            Positions = shadowOn.Positions,
            Statistics = shadowOn.Statistics,
            ShadowStatistics = new ShadowNeighborCacheStatistics
            {
                PreviousFramePairHitCount =
                    secondFrame.ShadowStatistics.PreviousFramePairHitCount,
                PreviousFramePairMissCount =
                    secondFrame.ShadowStatistics.PreviousFramePairMissCount,
                CurrentFrameWallDrivenEscapeBodyCount =
                    shadowOn.ShadowStatistics.CurrentFrameWallDrivenEscapeBodyCount
            }
        };
    }

    private static FlowMovementFrameState CreateBody(
        float3 position,
        float3 velocity,
        float radius,
        float inverseMass = 1f)
    {
        return new FlowMovementFrameState
        {
            CurrentPosition = position,
            CurrentVelocity = velocity,
            IntegratedVelocity = velocity,
            MoveSpeed = 100f,
            MaxForce = 100f,
            InverseMass = inverseMass,
            Radius = radius,
            IsInsideGrid = true,
            Cell = new FlowFieldCell { Cost = 1 }
        };
    }

    private static ScenarioResult RunScenario(
        FlowMovementFrameState[] sourceBodies,
        int iterationCount,
        float skin,
        bool enablePredictiveContacts = true,
        bool includeWall = false,
        int substepCount = 1,
        bool enableShadowTest = false,
        float shadowMargin = 0.25f,
        float softAvoidanceWeight = 0f,
        float softAvoidanceRadius = 0f,
        float settledSoftAvoidanceMultiplier = 1.5f)
    {
        var previousProxies = new NativeList<ShadowFatBodyProxy>(Allocator.TempJob);
        var previousPairs = new NativeList<ShadowEntityPair>(Allocator.TempJob);
        try
        {
            return RunScenarioWithCache(
                sourceBodies,
                iterationCount,
                skin,
                enablePredictiveContacts,
                includeWall,
                substepCount,
                enableShadowTest,
                shadowMargin,
                previousProxies,
                previousPairs,
                softAvoidanceWeight,
                softAvoidanceRadius,
                settledSoftAvoidanceMultiplier);
        }
        finally
        {
            previousPairs.Dispose();
            previousProxies.Dispose();
        }
    }

    private static ScenarioResult RunScenarioWithCache(
        FlowMovementFrameState[] sourceBodies,
        int iterationCount,
        float skin,
        bool enablePredictiveContacts,
        bool includeWall,
        int substepCount,
        bool enableShadowTest,
        float shadowMargin,
        NativeList<ShadowFatBodyProxy> previousProxies,
        NativeList<ShadowEntityPair> previousPairs,
        float softAvoidanceWeight = 0f,
        float softAvoidanceRadius = 0f,
        float settledSoftAvoidanceMultiplier = 1.5f)
    {
        int2 gridDimensions = includeWall ? new int2(5, 3) : new int2(40, 40);
        float3 gridOrigin = includeWall ? float3.zero : new float3(-10, 0, -10);
        var preparedBodies = (FlowMovementFrameState[])sourceBodies.Clone();
        for (int i = 0; i < preparedBodies.Length; i++)
        {
            FlowMovementFrameState body = preparedBodies[i];
            body.Entity = new Entity { Index = i + 1, Version = 1 };
            preparedBodies[i] = body;
        }
        var states = new NativeArray<FlowMovementFrameState>(preparedBodies, Allocator.TempJob);
        var grid = new NativeArray<FlowFieldCell>(
            gridDimensions.x * gridDimensions.y,
            Allocator.TempJob);
        var entries = new NativeList<SweptDiscCellEntry>(16, Allocator.TempJob);
        var pairs = new NativeList<UnitCollisionPair>(16, Allocator.TempJob);
        var shadowEntries = new NativeList<SweptDiscCellEntry>(32, Allocator.TempJob);
        var shadowBodyPairs = new NativeList<UnitCollisionPair>(32, Allocator.TempJob);
        var shadowCurrentProxies = new NativeList<ShadowFatBodyProxy>(16, Allocator.TempJob);
        var shadowCurrentPairs = new NativeList<ShadowEntityPair>(32, Allocator.TempJob);
        var statistics =
            new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob);
        var shadowStatistics =
            new NativeReference<ShadowNeighborCacheStatistics>(Allocator.TempJob);
        var iterationDiagnostics =
            new NativeList<Stage3ContactIterationDiagnostic>(16, Allocator.TempJob);
        var pairDiagnostics =
            new NativeList<Stage3ContactPairDiagnostic>(16, Allocator.TempJob);
        var selectedBodyDiagnostic =
            new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.TempJob);

        try
        {
            for (int i = 0; i < grid.Length; i++)
                grid[i] = new FlowFieldCell { Cost = 1 };
            if (includeWall)
                grid[FlowFieldUtils.GetFlatIndex(int2.zero, gridDimensions)] =
                    new FlowFieldCell { Cost = 0 };

            var solver = new SolveXpbdUnitContactsJob
            {
                DeltaTime = 1f,
                SubstepCount = substepCount,
                IterationCount = iterationCount,
                Compliance = 0f,
                PredictiveSkin = skin,
                SoftAvoidanceWeight = softAvoidanceWeight,
                SoftAvoidanceRadius = softAvoidanceRadius,
                SettledSoftAvoidanceMultiplier = settledSoftAvoidanceMultiplier,
                EnablePredictiveContacts = enablePredictiveContacts,
                EnableDiagnostics = true,
                EnableShadowNeighborCacheTest = enableShadowTest,
                ShadowCacheMargin = shadowMargin,
                DiagnosticSelectedEntity = Entity.Null,
                GridOrigin = gridOrigin,
                GridDimensions = gridDimensions,
                CellRadius = 0.5f,
                Grid = grid,
                SweptCellEntries = entries,
                Pairs = pairs,
                ShadowCellEntries = shadowEntries,
                ShadowBodyPairs = shadowBodyPairs,
                ShadowCurrentProxies = shadowCurrentProxies,
                ShadowCurrentPairs = shadowCurrentPairs,
                ShadowPreviousProxies = previousProxies,
                ShadowPreviousPairs = previousPairs,
                States = states,
                Statistics = statistics,
                ShadowStatistics = shadowStatistics,
                IterationDiagnostics = iterationDiagnostics,
                PairDiagnostics = pairDiagnostics,
                SelectedBodyDiagnostic = selectedBodyDiagnostic
            };
            solver.Run();

            var positions = new float3[states.Length];
            for (int i = 0; i < states.Length; i++)
                positions[i] = states[i].PredictedPosition;

            return new ScenarioResult
            {
                Positions = positions,
                Statistics = statistics.Value,
                ShadowStatistics = shadowStatistics.Value,
                IterationDiagnostics = iterationDiagnostics.AsArray().ToArray()
            };
        }
        finally
        {
            selectedBodyDiagnostic.Dispose();
            pairDiagnostics.Dispose();
            iterationDiagnostics.Dispose();
            shadowStatistics.Dispose();
            statistics.Dispose();
            shadowCurrentPairs.Dispose();
            shadowCurrentProxies.Dispose();
            shadowBodyPairs.Dispose();
            shadowEntries.Dispose();
            pairs.Dispose();
            entries.Dispose();
            grid.Dispose();
            states.Dispose();
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }

    private struct ScenarioResult
    {
        public float3[] Positions;
        public PredictiveDiscContactStatistics Statistics;
        public ShadowNeighborCacheStatistics ShadowStatistics;
        public Stage3ContactIterationDiagnostic[] IterationDiagnostics;
    }
}
