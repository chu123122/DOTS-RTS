using System;
using System.IO;
using Unity.Collections;
using Unity.Entities;
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
        ValidateWallCorrectionDiagnostic();
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
                result.Statistics.PredictiveActivatedCount == 1,
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
        Require(result.Statistics.PredictivePairCount == 0,
            "Tangential near miss was incorrectly frozen to the initial normal.");
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
        Require(result.Statistics.PotentialPredictivePairCount == 1 &&
                result.Statistics.PredictivePairCount == 0,
            "Predictive toggle did not preserve the potential-Pair diagnostic boundary.");
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

    private static void ValidateWallCorrectionDiagnostic()
    {
        var states = new NativeArray<FlowMovementFrameState>(1, Allocator.Temp);
        var statistics =
            new NativeReference<PredictiveDiscContactStatistics>(Allocator.Temp);
        var selectedBody =
            new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.Temp);

        try
        {
            states[0] = new FlowMovementFrameState
            {
                IsInsideGrid = true,
                PositionCorrection = new float3(0.3f, 0, 0.4f)
            };
            var finalizeJob = new FinalizeStage3ContactDiagnosticsJob
            {
                EnableDiagnostics = true,
                DiagnosticSelectedEntity = Entity.Null,
                States = states,
                Statistics = statistics,
                SelectedBodyDiagnostic = selectedBody
            };
            finalizeJob.Execute();

            Require(math.abs(statistics.Value.TotalWallPositionCorrection - 0.5f) <=
                    PositionTolerance &&
                    math.abs(statistics.Value.MaxWallPositionCorrection - 0.5f) <=
                    PositionTolerance,
                "Wall correction diagnostic was not accumulated from the post-contact wall pass.");
        }
        finally
        {
            selectedBody.Dispose();
            statistics.Dispose();
            states.Dispose();
        }
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
        bool enablePredictiveContacts = true)
    {
        var states = new NativeArray<FlowMovementFrameState>(sourceBodies, Allocator.Temp);
        var entries = new NativeList<SweptDiscCellEntry>(16, Allocator.Temp);
        var pairs = new NativeList<UnitCollisionPair>(16, Allocator.Temp);
        var statistics =
            new NativeReference<PredictiveDiscContactStatistics>(Allocator.Temp);
        var iterationDiagnostics =
            new NativeList<Stage3ContactIterationDiagnostic>(16, Allocator.Temp);
        var pairDiagnostics =
            new NativeList<Stage3ContactPairDiagnostic>(16, Allocator.Temp);
        var selectedBodyDiagnostic =
            new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.Temp);

        try
        {
            var solver = new SolveXpbdUnitContactsJob
            {
                DeltaTime = 1f,
                SubstepCount = 1,
                IterationCount = iterationCount,
                Compliance = 0f,
                PredictiveSkin = skin,
                EnablePredictiveContacts = enablePredictiveContacts,
                EnableDiagnostics = true,
                DiagnosticSelectedEntity = Entity.Null,
                GridOrigin = new float3(-10, 0, -10),
                GridDimensions = new int2(40, 40),
                CellRadius = 0.5f,
                SweptCellEntries = entries,
                Pairs = pairs,
                States = states,
                Statistics = statistics,
                IterationDiagnostics = iterationDiagnostics,
                PairDiagnostics = pairDiagnostics,
                SelectedBodyDiagnostic = selectedBodyDiagnostic
            };
            solver.Execute();

            var positions = new float3[states.Length];
            for (int i = 0; i < states.Length; i++)
                positions[i] = states[i].PredictedPosition;

            return new ScenarioResult
            {
                Positions = positions,
                Statistics = statistics.Value,
                IterationDiagnostics = iterationDiagnostics.AsArray().ToArray()
            };
        }
        finally
        {
            selectedBodyDiagnostic.Dispose();
            pairDiagnostics.Dispose();
            iterationDiagnostics.Dispose();
            statistics.Dispose();
            pairs.Dispose();
            entries.Dispose();
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
        public Stage3ContactIterationDiagnostic[] IterationDiagnostics;
    }
}
