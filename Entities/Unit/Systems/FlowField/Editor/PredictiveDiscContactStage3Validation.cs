using System;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Editor
{

/// <summary>
/// Predictive contact、Fat AABB 缓存与逐 iteration 诊断的回归入口。
/// </summary>
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
        ScenarioResult generationDisabled = ValidatePredictiveGenerationToggle();
        ScenarioResult tangent = ValidateTangentialNearMiss();
        ScenarioResult chain = ValidatePrebuiltChainContact();
        ScenarioResult softAvoidance = ValidateSoftAvoidancePerSubstep();
        ScenarioResult rvoAvoidance = ValidateRvoVelocitySolver();
        ScenarioResult timestepContactSet = ValidateTimestepContactSetReuse();
        ScenarioResult substepContactSet = ValidateSubstepContactSetRegeneration();
        ValidateFatAabbDoesNotSweepFromWorldOrigin();
        ValidateDiagnosticReadbackIsolation();
        (ScenarioResult wallOneIteration, ScenarioResult wallEightIterations) =
            ValidateWallAndUnitConstraintsIterateTogether();
        ScenarioResult fatCache = ValidateFatAabbCache();
        ValidateAutomaticFatAabbCaptureSequence();
        (ScenarioResult oneIteration, ScenarioResult eightIterations) =
            ValidateIterationResidualReduction();

        Debug.Log(
            "STAGE3_VALIDATION_OK\n" +
            $"crossing: predictive={crossing.Statistics.PredictivePairCount}, " +
            $"activated={crossing.Statistics.PredictiveActivatedCount}\n" +
            $"predictive disabled: potential={predictiveDisabled.Statistics.PotentialPredictivePairCount}, " +
            $"active predictive={predictiveDisabled.Statistics.PredictivePairCount}\n" +
            $"generation disabled: contacts={generationDisabled.Statistics.ContactPairCount}, " +
            $"predicted={generationDisabled.Statistics.PredictiveGeneratedPairCount}\n" +
            $"tangent: contacts={tangent.Statistics.ContactPairCount}, " +
            $"unactivated={tangent.Statistics.UnactivatedPairCount}\n" +
            $"chain: active={chain.Statistics.ActiveConstraintCount}\n" +
            $"soft avoidance: evaluations={softAvoidance.Statistics.SoftAvoidanceEvaluationCount}, " +
            $"time={softAvoidance.Statistics.SoftAvoidanceNanoseconds}ns\n" +
            $"RVO avoidance: activated={rvoAvoidance.Statistics.SoftAvoidanceActivatedPairCount}, " +
            $"fat uses={rvoAvoidance.Statistics.SoftAvoidanceFatAabbUseCount}\n" +
            $"timestep contacts: builds={timestepContactSet.Statistics.TimestepContactSetBuildCount}, " +
            $"classification={timestepContactSet.Statistics.TimestepContactSetClassificationPassCount}, " +
            $"substep uses={timestepContactSet.Statistics.TimestepContactSetSubstepUseCount}\n" +
            $"wall->unit: B.x {wallOneIteration.Positions[1].x:F6} -> " +
            $"{wallEightIterations.Positions[1].x:F6}\n" +
            $"fat cache: reuse={fatCache.ShadowStatistics.CacheReuseCount}, " +
            $"fallback={fatCache.ShadowStatistics.FullBroadPhaseFallbackCount}, " +
            $"post-solve invalidation={fatCache.ShadowStatistics.PostSolveInvalidationCount}, " +
            $"mapping={fatCache.ShadowStatistics.CachePairMappingBuildCount}/" +
            $"{fatCache.ShadowStatistics.CachePairMappingReuseCount}, " +
            $"corrected checks={fatCache.ShadowStatistics.CorrectedBodyValidationCount}\n" +
            $"auto capture: configs=1x8/2x4/4x2, " +
            $"runs={Stage3ContactDiagnosticAutoCapture.DefaultRoundCount * Stage3ContactDiagnosticAutoCapture.RunsPerRound}, restored=1\n" +
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
            $"Regular overlap was not separated to the radius sum: distance={finalDistance:F6}, " +
            $"pairs={result.Statistics.ContactPairCount}, " +
            $"active={result.Statistics.ActiveConstraintCount}, " +
            $"uniqueActive={result.Statistics.TimestepContactSetUniqueActivatedPairCount}, " +
            $"correction={result.Statistics.TotalContactPositionCorrection:F6}, " +
            $"builds={result.Statistics.TimestepContactSetBuildCount}, " +
            $"fallbacks={result.Statistics.TimestepContactSetFullRebuildCount}.");
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

    private static ScenarioResult ValidateTimestepContactSetReuse()
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
            substepCount: 4);
        Require(result.Statistics.TimestepContactSetBuildCount == 1,
            "Timestep ContactSet was rebuilt without an escape.");
        Require(result.Statistics.TimestepContactSetClassificationPassCount == 1,
            "FilterAndClassifyPairs still ran once per substep instead of once per timestep.");
        Require(result.Statistics.TimestepContactSetSubstepUseCount == 4,
            "All substeps did not consume the same Timestep ContactSet.");
        Require(result.Statistics.TimestepContactSetUniquePairCount == 1 &&
                result.Statistics.TimestepContactSetUniqueActivatedPairCount == 1,
            "Timestep ContactSet did not retain one crossing contact across substeps.");
        Require(result.HeatSamples != null && result.HeatSamples.Length == 2 &&
                result.HeatSamples[0].ContactPairDegree == 1 &&
                result.HeatSamples[1].ContactPairDegree == 1 &&
                result.HeatSamples[0].ActivePairDegree == 1 &&
                result.HeatSamples[1].ActivePairDegree == 1,
            "Contact heat samples did not reflect the reused timestep pair.");
        return result;
    }

    private static ScenarioResult ValidateSubstepContactSetRegeneration()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(0, 0, 0), float3.zero, 0.25f),
            CreateBody(new float3(0.2f, 0, 0), float3.zero, 0.25f)
        };

        ScenarioResult result = RunScenario(
            bodies,
            iterationCount: 4,
            skin: 0.05f,
            substepCount: 4,
            enableTimestepContactSetCache: false);
        Require(result.Statistics.TimestepContactSetBuildCount == 4,
            "Per-substep mode did not rebuild the Contact Set once per substep.");
        Require(result.Statistics.TimestepContactSetClassificationPassCount == 4,
            "Per-substep mode did not classify contacts once per substep.");
        Require(result.Statistics.TimestepContactSetSubstepUseCount == 4,
            "Per-substep Contact Sets were not consumed by all substeps.");
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

    private static void ValidateAutomaticFatAabbCaptureSequence()
    {
        var original = new UnitContactSolverSettings
        {
            SubstepCount = 5,
            IterationCount = 3,
            PredictiveSkin = 0.2f,
            EnablePredictivePairGeneration = false,
            EnablePredictiveContacts = false,
            EnableDiagnostics = false,
            DiagnosticCaptureDuration = 4f,
            DiagnosticCaptureInterval = 0.5f,
            EnableFatAabbCache = true,
            FatAabbCacheMargin = 0.7f
        };
        UnitContactSolverSettings settings = original;
        var automaticCapture = new Stage3ContactDiagnosticAutoCapture();
        automaticCapture.Start(ref settings, 100d);

        Require(automaticCapture.TotalRuns == 9,
            "Automatic capture did not create three OFF -> ON -> OFF rounds.");
        Require(settings.SubstepCount == 1 && settings.IterationCount == 8,
            "Automatic capture did not apply the first 1x8 configuration.");
        Require(!automaticCapture.Tick(
                ref settings, 102.99d, false, out _, out _),
            "Automatic capture started before the initial warmup completed.");

        double startTime = 103d;
        string firstLabel = string.Empty;
        string middleLabel = string.Empty;
        string finalLabel = string.Empty;
        bool completed = false;
        for (int runIndex = 0; runIndex < automaticCapture.TotalRuns; runIndex++)
        {
            Require(automaticCapture.Tick(
                    ref settings, startTime, false, out string runLabel, out _),
                $"Automatic capture did not start run {runIndex + 1}.");
            Require(
                settings.SubstepCount ==
                Stage3ContactDiagnosticAutoCapture.GetSubstepsForRun(runIndex) &&
                settings.IterationCount ==
                Stage3ContactDiagnosticAutoCapture.GetIterationsForRun(runIndex),
                $"Automatic capture applied the wrong configuration for run {runIndex + 1}.");
            Require(
                settings.EnableFatAabbCache ==
                Stage3ContactDiagnosticAutoCapture.IsCacheEnabledForRun(runIndex),
                $"Automatic capture applied the wrong cache mode for run {runIndex + 1}.");

            if (runIndex == 0)
                firstLabel = runLabel;
            else if (runIndex == 4)
                middleLabel = runLabel;
            else if (runIndex == 8)
                finalLabel = runLabel;

            Require(!automaticCapture.Tick(
                    ref settings, startTime + 0.1d, true, out _, out _),
                "Automatic capture attempted to overlap recordings.");
            automaticCapture.Tick(
                ref settings,
                startTime + 10.1d,
                false,
                out _,
                out completed);
            if (runIndex < 8)
            {
                Require(!completed && automaticCapture.Active,
                    "Automatic capture completed before all configurations ran.");
                startTime += 10.1d +
                             Stage3ContactDiagnosticAutoCapture.TransitionWarmupSeconds;
            }
        }

        Require(firstLabel == "fat-aabb-r01-s1-i8-off-before" &&
                middleLabel == "fat-aabb-r02-s2-i4-on" &&
                finalLabel == "fat-aabb-r03-s4-i2-off-after",
            "Automatic capture produced unexpected configuration labels.");
        Require(completed && !automaticCapture.Active,
            "Automatic capture did not complete all nine recordings.");
        Require(settings.SubstepCount == original.SubstepCount &&
                settings.IterationCount == original.IterationCount &&
                settings.PredictiveSkin == original.PredictiveSkin &&
                settings.EnablePredictivePairGeneration == original.EnablePredictivePairGeneration &&
                settings.EnablePredictiveContacts == original.EnablePredictiveContacts &&
                settings.EnableDiagnostics == original.EnableDiagnostics &&
                settings.DiagnosticCaptureDuration == original.DiagnosticCaptureDuration &&
                settings.DiagnosticCaptureInterval == original.DiagnosticCaptureInterval &&
                settings.EnableFatAabbCache == original.EnableFatAabbCache &&
                settings.FatAabbCacheMargin == original.FatAabbCacheMargin,
            "Automatic capture did not restore the original settings.");
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
        float3 shellVelocity = SoftAvoidanceMath.CalculateUnitVelocity(
            new float3(-0.5f, 0, 0),
            new float3(0.5f, 0, 0),
            0.4f,
            0.4f,
            1f,
            0.25f);
        float3 outsideShellVelocity = SoftAvoidanceMath.CalculateUnitVelocity(
            new float3(-0.5f, 0, 0),
            new float3(0.5f, 0, 0),
            0.4f,
            0.4f,
            1f,
            0.1f);
        Require(math.lengthsq(shellVelocity) > 0f &&
                math.lengthsq(outsideShellVelocity) == 0f,
            "Soft avoidance did not use radiusA + radiusB + softShell activation distance.");

        float fullStepAlpha = SoftAvoidanceMath.CalculateBufferAlpha(4f, 1f);
        float quarterStepAlpha = SoftAvoidanceMath.CalculateBufferAlpha(4f, 0.25f);
        float recomposedAlpha = 1f - math.pow(1f - quarterStepAlpha, 4f);
        Require(math.abs(fullStepAlpha - recomposedAlpha) <= 0.00001f,
            "Soft avoidance velocity buffer changed response when substep count changed.");

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
            softAvoidanceResponseRate: 1f,
            softAvoidanceShell: 0.8f);
        ScenarioResult cachedResult = RunScenario(
            bodies,
            iterationCount: 1,
            skin: 0f,
            substepCount: 4,
            enableFatAabbCache: true,
            fatAabbMargin: 0.25f,
            softAvoidanceResponseRate: 1f,
            softAvoidanceShell: 0.8f);
        Require(result.Statistics.SoftAvoidanceEvaluationCount == 4,
            "Soft avoidance was not recomputed once per substep.");
        Require(math.distance(result.Positions[0], result.Positions[1]) > 0.8f,
            "Per-substep soft avoidance did not separate nearby units.");
        RequirePositionsEqual(
            result.Positions,
            cachedResult.Positions,
            "Fat AABB raw candidates changed soft avoidance positions.");
        Require(cachedResult.Statistics.SoftAvoidanceFatAabbUseCount == 4 &&
                cachedResult.Statistics.SoftAvoidanceCandidatePairCount > 0 &&
                cachedResult.Statistics.SoftAvoidanceActivatedPairCount > 0,
            "Soft avoidance did not consume Fat AABB raw candidates for every substep.");
        return result;
    }

    private static ScenarioResult ValidateRvoVelocitySolver()
    {
        bool approachingActivated = SoftAvoidanceMath.TryCalculatePairVelocities(
            SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle,
            new float3(-1f, 0, 0),
            new float3(1f, 0, 0),
            new float3(1f, 0, 0),
            new float3(-1f, 0, 0),
            0.25f,
            0.25f,
            1f,
            1f,
            1f,
            1f,
            0.3f,
            1f,
            0.1f,
            new float3(-1f, 0, 0),
            out float3 correctionA,
            out float3 correctionB);
        bool separatingActivated = SoftAvoidanceMath.TryCalculatePairVelocities(
            SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle,
            new float3(-1f, 0, 0),
            new float3(1f, 0, 0),
            new float3(-1f, 0, 0),
            new float3(1f, 0, 0),
            0.25f,
            0.25f,
            1f,
            1f,
            1f,
            1f,
            0.3f,
            1f,
            0.1f,
            new float3(-1f, 0, 0),
            out _,
            out _);
        Require(approachingActivated && correctionA.x < 0f && correctionB.x > 0f,
            "RVO solver did not reduce a reciprocal head-on closing velocity.");
        Require(!separatingActivated,
            "RVO solver modified units that were already separating.");

        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(-1f, 0, 0), new float3(1f, 0, 0), 0.25f),
            CreateBody(new float3(1f, 0, 0), new float3(-1f, 0, 0), 0.25f)
        };
        ScenarioResult result = RunScenario(
            bodies,
            iterationCount: 1,
            skin: 0f,
            enablePredictiveContacts: false,
            enableFatAabbCache: true,
            softAvoidanceResponseRate: 4f,
            softAvoidanceShell: 0.3f,
            enablePredictivePairGeneration: false,
            softAvoidanceVelocitySolver:
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle,
            rvoTimeHorizon: 1f);
        Require(result.Statistics.SoftAvoidanceActivatedPairCount > 0 &&
                result.Statistics.SoftAvoidanceFatAabbUseCount == 1,
            "Runtime RVO mode did not activate through the configured Fat AABB candidate path.");
        Require(result.Positions[0].x < result.Positions[1].x,
            "Runtime RVO mode did not prevent the head-on endpoint exchange.");
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
        Stage3ContactIterationDiagnostic firstIteration =
            eightIterations.IterationDiagnostics[0];
        Stage3ContactIterationDiagnostic finalIteration =
            eightIterations.IterationDiagnostics[eightIterations.IterationDiagnostics.Length - 1];
        Require(firstIteration.MaxConstraintViolationBeforeSolve > 0f,
            "The pre-solve residual did not expose the initial dense-contact violation.");
        Require(finalIteration.MaxConstraintViolation <=
                firstIteration.MaxConstraintViolationBeforeSolve,
            "The final post-solve residual exceeded the initial pre-solve residual.");
        return (oneIteration, eightIterations);
    }

    private static ScenarioResult ValidatePredictiveGenerationToggle()
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
            enablePredictivePairGeneration: false);
        Require(result.Positions[0].x > result.Positions[1].x,
            "Disabling Predictive Pair generation did not restore the crossing baseline.");
        Require(result.Statistics.ContactPairCount == 0 &&
                result.Statistics.PredictiveGeneratedPairCount == 0 &&
                result.Statistics.PredictivePairCount == 0,
            "Disabling Predictive Pair generation still retained a swept Pair.");
        return result;
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

    private static ScenarioResult ValidateFatAabbCache()
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
        var cacheState = new NativeReference<FatAabbCacheState>(Allocator.TempJob);

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
                enableFatAabbCache: true,
                fatAabbMargin: 0.25f,
                previousProxies: previousProxies,
                previousPairs: previousPairs,
                cacheState: cacheState);
            secondFrame = RunScenarioWithCache(
                denseBodies,
                iterationCount: 4,
                skin: 0.05f,
                enablePredictiveContacts: true,
                includeWall: false,
                substepCount: 2,
                enableFatAabbCache: true,
                fatAabbMargin: 0.25f,
                previousProxies: previousProxies,
                previousPairs: previousPairs,
                cacheState: cacheState);
        }
        finally
        {
            cacheState.Dispose();
            previousPairs.Dispose();
            previousProxies.Dispose();
        }

        ScenarioResult denseReference = RunScenario(
            denseBodies,
            iterationCount: 4,
            skin: 0.05f,
            substepCount: 2,
            enableFatAabbCache: false);
        Require(secondFrame.ShadowStatistics.CacheValidAtFrameStart != 0 &&
                secondFrame.ShadowStatistics.CacheReuseCount == 1 &&
                secondFrame.ShadowStatistics.CacheRebuildCount == 0,
            "Stable dense contacts did not reuse the persistent Fat AABB cache for the timestep ContactSet.");
        Require(secondFrame.ShadowStatistics.FullBroadPhaseFallbackCount == 0,
            "Stable dense contacts unexpectedly fell back to the full Broad Phase.");
        Require(secondFrame.ShadowStatistics.CachePairMappingBuildCount == 1 &&
                secondFrame.ShadowStatistics.CachePairMappingReuseCount == 0,
            "Contact-only timestep unexpectedly remapped Fat AABB pairs per substep.");
        int previousFullBodyCheckCount = denseBodies.Length * 4 * 2 * 2;
        Require(secondFrame.ShadowStatistics.CorrectedBodyValidationCount > 0 &&
                secondFrame.ShadowStatistics.CorrectedBodyValidationCount <
                previousFullBodyCheckCount,
            "Corrected-body Fat AABB validation did not reduce the previous full-body scan count.");
        RequirePositionsEqual(
            denseReference.Positions,
            secondFrame.Positions,
            "Fat AABB reuse changed a stable dense-contact solver position.");

        FlowMovementFrameState[] wallBodies =
        {
            CreateBody(new float3(1.4f, 0, 0.5f), float3.zero, 0.5f),
            CreateBody(new float3(2.42f, 0, 0.5f), float3.zero, 0.5f)
        };
        ScenarioResult cacheOff = RunScenario(
            wallBodies,
            iterationCount: 8,
            skin: 0.1f,
            includeWall: true,
            enableFatAabbCache: false);
        ScenarioResult cacheOn = RunScenario(
            wallBodies,
            iterationCount: 8,
            skin: 0.1f,
            includeWall: true,
            enableFatAabbCache: true,
            fatAabbMargin: 0.001f,
            timestepContactMargin: 0.001f);
        RequirePositionsEqual(
            cacheOff.Positions,
            cacheOn.Positions,
            "Fat AABB fallback changed a wall-contact solver position.");
        Require(cacheOn.ShadowStatistics.PostSolveInvalidationCount > 0 &&
                cacheOn.ShadowStatistics.FullBroadPhaseFallbackCount > 0 &&
                cacheOn.Statistics.TimestepContactSetFullRebuildCount > 0,
            "Small Fat AABB margin did not trigger the wall-driven safe fallback.");

        var toggleProxies = new NativeList<ShadowFatBodyProxy>(Allocator.TempJob);
        var togglePairs = new NativeList<ShadowEntityPair>(Allocator.TempJob);
        var toggleState = new NativeReference<FatAabbCacheState>(Allocator.TempJob);
        ScenarioResult enabledAgain;
        try
        {
            RunScenarioWithCache(
                denseBodies, 4, 0.05f, true, false, 2, true, 0.25f,
                toggleProxies, togglePairs, toggleState);
            ScenarioResult disabled = RunScenarioWithCache(
                denseBodies, 4, 0.05f, true, false, 2, false, 0.25f,
                toggleProxies, togglePairs, toggleState);
            Require(toggleState.Value.IsValid == 0 &&
                    toggleProxies.Length == 0 && togglePairs.Length == 0,
                "Disabling Fat AABB cache did not clear persistent cache state.");
            RequirePositionsEqual(
                denseReference.Positions,
                disabled.Positions,
                "Disabling Fat AABB cache changed the uncached solver result.");
            enabledAgain = RunScenarioWithCache(
                denseBodies, 4, 0.05f, true, false, 2, true, 0.25f,
                toggleProxies, togglePairs, toggleState);
        }
        finally
        {
            toggleState.Dispose();
            togglePairs.Dispose();
            toggleProxies.Dispose();
        }
        Require(enabledAgain.ShadowStatistics.CacheValidAtFrameStart == 0 &&
                enabledAgain.ShadowStatistics.CacheRebuildCount > 0,
            "Re-enabling Fat AABB cache reused stale disabled-state data instead of rebuilding.");

        return new ScenarioResult
        {
            Positions = cacheOn.Positions,
            Statistics = cacheOn.Statistics,
            ShadowStatistics = new ShadowNeighborCacheStatistics
            {
                CacheReuseCount = secondFrame.ShadowStatistics.CacheReuseCount,
                FullBroadPhaseFallbackCount =
                    cacheOn.ShadowStatistics.FullBroadPhaseFallbackCount,
                PostSolveInvalidationCount =
                    cacheOn.ShadowStatistics.PostSolveInvalidationCount,
                CachePairMappingBuildCount =
                    secondFrame.ShadowStatistics.CachePairMappingBuildCount,
                CachePairMappingReuseCount =
                    secondFrame.ShadowStatistics.CachePairMappingReuseCount,
                CorrectedBodyValidationCount =
                    secondFrame.ShadowStatistics.CorrectedBodyValidationCount
            }
        };
    }

    private static void RequirePositionsEqual(
        float3[] expected,
        float3[] actual,
        string message)
    {
        Require(expected.Length == actual.Length, message + " Body count differs.");
        for (int i = 0; i < expected.Length; i++)
        {
            Require(math.distance(expected[i], actual[i]) <= PositionTolerance,
                message + $" Body {i} differs.");
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
        bool enablePredictiveContacts = true,
        bool includeWall = false,
        int substepCount = 1,
        bool enableFatAabbCache = false,
        float fatAabbMargin = 0.25f,
        float softAvoidanceResponseRate = 0f,
        float softAvoidanceShell = 0f,
        float settledSoftAvoidanceMultiplier = 1.5f,
        bool enablePredictivePairGeneration = true,
        SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver =
            SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer,
        float rvoTimeHorizon = 0.5f,
        float timestepContactMargin = 0.25f,
        bool enableTimestepContactSetCache = true)
    {
        var previousProxies = new NativeList<ShadowFatBodyProxy>(Allocator.TempJob);
        var previousPairs = new NativeList<ShadowEntityPair>(Allocator.TempJob);
        var cacheState = new NativeReference<FatAabbCacheState>(Allocator.TempJob);
        try
        {
            return RunScenarioWithCache(
                sourceBodies,
                iterationCount,
                skin,
                enablePredictiveContacts,
                includeWall,
                substepCount,
                enableFatAabbCache,
                fatAabbMargin,
                previousProxies,
                previousPairs,
                cacheState,
                softAvoidanceResponseRate,
                softAvoidanceShell,
                settledSoftAvoidanceMultiplier,
                enablePredictivePairGeneration,
                softAvoidanceVelocitySolver,
                rvoTimeHorizon,
                timestepContactMargin,
                enableTimestepContactSetCache);
        }
        finally
        {
            cacheState.Dispose();
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
        bool enableFatAabbCache,
        float fatAabbMargin,
        NativeList<ShadowFatBodyProxy> previousProxies,
        NativeList<ShadowEntityPair> previousPairs,
        NativeReference<FatAabbCacheState> cacheState,
        float softAvoidanceResponseRate = 0f,
        float softAvoidanceShell = 0f,
        float settledSoftAvoidanceMultiplier = 1.5f,
        bool enablePredictivePairGeneration = true,
        SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver =
            SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer,
        float rvoTimeHorizon = 0.5f,
        float timestepContactMargin = 0.25f,
        bool enableTimestepContactSetCache = true)
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
        var timestepInteractionPairs =
            new NativeList<UnitCollisionPair>(32, Allocator.TempJob);
        var timestepContactPairs = new NativeList<UnitCollisionPair>(16, Allocator.TempJob);
        var shadowEntries = new NativeList<SweptDiscCellEntry>(32, Allocator.TempJob);
        var shadowBodyPairs = new NativeList<UnitCollisionPair>(32, Allocator.TempJob);
        var shadowCurrentProxies = new NativeList<ShadowFatBodyProxy>(16, Allocator.TempJob);
        var shadowCurrentPairs = new NativeList<ShadowEntityPair>(32, Allocator.TempJob);
        var currentBodyIndexByEntity = new NativeParallelHashMap<Entity, int>(
            math.max(preparedBodies.Length, 1),
            Allocator.TempJob);
        var mappedFatCachePairs = new NativeList<UnitCollisionPair>(32, Allocator.TempJob);
        var correctedBodyFlags = new NativeArray<byte>(
            preparedBodies.Length,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var correctedBodyIndices = new NativeList<int>(16, Allocator.TempJob);
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
        var heatSamples = new NativeArray<Stage3ContactHeatSample>(
            preparedBodies.Length,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);

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
                SoftAvoidanceResponseRate = softAvoidanceResponseRate,
                SoftAvoidanceShell = softAvoidanceShell,
                SettledSoftAvoidanceMultiplier = settledSoftAvoidanceMultiplier,
                SoftAvoidanceVelocitySolver = softAvoidanceVelocitySolver,
                RvoTimeHorizon = rvoTimeHorizon,
                EnablePredictivePairGeneration = enablePredictivePairGeneration,
                EnablePredictiveContacts = enablePredictiveContacts,
                EnableDiagnostics = true,
                EnablePersistentContactCache = enableFatAabbCache && enableTimestepContactSetCache,
                EnableTimestepContactSetCache = enableTimestepContactSetCache,
                FatAabbCacheMargin = fatAabbMargin,
                TimestepContactMargin = timestepContactMargin,
                DiagnosticSelectedEntity = Entity.Null,
                GridOrigin = gridOrigin,
                GridDimensions = gridDimensions,
                CellRadius = 0.5f,
                Grid = grid,
                SweptCellEntries = entries,
                Pairs = pairs,
                TimestepInteractionPairs = timestepInteractionPairs,
                TimestepContactPairs = timestepContactPairs,
                ShadowCellEntries = shadowEntries,
                ShadowBodyPairs = shadowBodyPairs,
                ShadowCurrentProxies = shadowCurrentProxies,
                ShadowCurrentPairs = shadowCurrentPairs,
                CurrentBodyIndexByEntity = currentBodyIndexByEntity,
                MappedFatCachePairs = mappedFatCachePairs,
                CorrectedBodyFlags = correctedBodyFlags,
                CorrectedBodyIndices = correctedBodyIndices,
                ShadowPreviousProxies = previousProxies,
                ShadowPreviousPairs = previousPairs,
                FatAabbCacheState = cacheState,
                States = states,
                Statistics = statistics,
                ShadowStatistics = shadowStatistics,
                IterationDiagnostics = iterationDiagnostics,
                PairDiagnostics = pairDiagnostics,
                SelectedBodyDiagnostic = selectedBodyDiagnostic,
                HeatSamples = heatSamples
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
                IterationDiagnostics = iterationDiagnostics.AsArray().ToArray(),
                HeatSamples = heatSamples.ToArray()
            };
        }
        finally
        {
            selectedBodyDiagnostic.Dispose();
            heatSamples.Dispose();
            pairDiagnostics.Dispose();
            iterationDiagnostics.Dispose();
            shadowStatistics.Dispose();
            statistics.Dispose();
            correctedBodyIndices.Dispose();
            correctedBodyFlags.Dispose();
            mappedFatCachePairs.Dispose();
            currentBodyIndexByEntity.Dispose();
            shadowCurrentPairs.Dispose();
            shadowCurrentProxies.Dispose();
            shadowBodyPairs.Dispose();
            shadowEntries.Dispose();
            pairs.Dispose();
            timestepInteractionPairs.Dispose();
            timestepContactPairs.Dispose();
            entries.Dispose();
            grid.Dispose();
            states.Dispose();
        }
    }

    private static void ValidateFatAabbDoesNotSweepFromWorldOrigin()
    {
        FlowMovementFrameState[] bodies =
        {
            CreateBody(new float3(5f, 0f, 5f), float3.zero, 0.25f),
            CreateBody(new float3(8f, 0f, 8f), float3.zero, 0.25f)
        };

        ScenarioResult result = RunScenario(
            bodies,
            iterationCount: 1,
            skin: 0f,
            enableFatAabbCache: true,
            fatAabbMargin: 0.05f,
            softAvoidanceResponseRate: 4f,
            softAvoidanceShell: 0.2f);
        Require(result.ShadowStatistics.CachedCandidatePairCount == 0 &&
                result.Statistics.CandidatePairCount == 0 &&
                result.Statistics.SoftAvoidanceCandidatePairCount == 0,
            "First-substep Fat AABBs swept from world origin before prediction.");
    }

    private static void ValidateDiagnosticReadbackIsolation()
    {
        var settings = new UnitContactSolverSettings
        {
            EnableFatAabbCache = true,
            EnableDiagnostics = false,
            VisualizeContactHeatmap = false
        };
        Require(!Stage3ContactDiagnosticReadback.Required(settings),
            "Fat AABB alone unexpectedly forced diagnostic readback synchronization.");

        settings.EnableFatAabbCache = false;
        settings.VisualizeContactHeatmap = true;
        Require(Stage3ContactDiagnosticReadback.Required(settings),
            "The regular heatmap did not request its published contact samples.");

        settings.VisualizeContactHeatmap = false;
        settings.EnableDiagnostics = true;
        Require(Stage3ContactDiagnosticReadback.Required(settings),
            "Explicit diagnostics did not request diagnostic readback synchronization.");
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
        public Stage3ContactHeatSample[] HeatSamples;
    }
}
}
