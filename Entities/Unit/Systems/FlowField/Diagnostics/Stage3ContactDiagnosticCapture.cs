using System;
using System.Collections.Generic;
using System.IO;
using Unity.Entities;
using UnityEngine;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Diagnostics
{

public sealed class Stage3ContactDiagnosticCaptureSession
{
    private const int MaxSamples = 2000;
    private readonly List<Stage3ContactDiagnosticCaptureSample> _samples = new(MaxSamples);
    private double _startSimulationTime;
    private double _nextSampleTime;
    private float _duration;
    private float _interval;
    private string _runLabel = string.Empty;

    public bool Active { get; private set; }
    public string LastOutputPath { get; private set; } = string.Empty;
    public int SampleCount => _samples.Count;
    public float Duration => _duration;
    public float Interval => _interval;

    public void Start(
        double simulationTime,
        float duration,
        float interval,
        string runLabel = "")
    {
        _samples.Clear();
        _duration = Mathf.Clamp(duration, 0.5f, 60f);
        _interval = Mathf.Clamp(interval, 0.05f, 5f);
        _startSimulationTime = simulationTime;
        _nextSampleTime = simulationTime + _interval;
        _runLabel = SanitizeRunLabel(runLabel);
        LastOutputPath = string.Empty;
        Active = true;
    }

    public float GetElapsed(double simulationTime)
    {
        return Active ? Mathf.Max(0f, (float)(simulationTime - _startSimulationTime)) : 0f;
    }

    public bool ShouldSample(double simulationTime)
    {
        return Active && simulationTime + 0.000001d >= _nextSampleTime;
    }

    public void AddSample(
        double simulationTime,
        UnitContactSolverSettings settings,
        FlowFieldSettings flowSettings,
        PredictiveDiscContactStatistics statistics,
        ShadowNeighborCacheStatistics shadow,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterations)
    {
        if (!Active || _samples.Count >= MaxSamples)
            return;

        int lastSubstep = iterations.Length > 0
            ? iterations[iterations.Length - 1].SubstepIndex
            : -1;
        var residualBefore = new List<float>(settings.IterationCount);
        var residualAfter = new List<float>(settings.IterationCount);
        for (int i = 0; i < iterations.Length; i++)
        {
            Stage3ContactIterationDiagnostic iteration = iterations[i];
            if (iteration.SubstepIndex != lastSubstep)
                continue;
            residualBefore.Add(iteration.MaxConstraintViolationBeforeSolve);
            residualAfter.Add(iteration.MaxConstraintViolation);
        }

        _samples.Add(new Stage3ContactDiagnosticCaptureSample
        {
            SimulationTime = simulationTime,
            CaptureTime = (float)(simulationTime - _startSimulationTime),
            Frame = Time.frameCount,
            Substeps = settings.SubstepCount,
            Iterations = settings.IterationCount,
            PredictiveGenerationEnabled = settings.EnablePredictivePairGeneration,
            SideExchangeConstraintEnabled = settings.EnablePredictiveContacts,
            SoftAvoidanceVelocitySolver = flowSettings.SoftAvoidanceVelocitySolver.ToString(),
            SoftAvoidanceResponseRate = flowSettings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = flowSettings.SoftAvoidanceShell,
            RvoTimeHorizon = flowSettings.RvoTimeHorizon,
            CandidatePairs = statistics.CandidatePairCount,
            ContactPairs = statistics.ContactPairCount,
            ActualGeneratedPairs = statistics.ActualGeneratedPairCount,
            PredictiveGeneratedPairs = statistics.PredictiveGeneratedPairCount,
            SideExchangeRiskPairs = statistics.PotentialPredictivePairCount,
            SideExchangePairs = statistics.PredictivePairCount,
            ActiveConstraints = statistics.ActiveConstraintCount,
            PredictiveActivatedConstraints = statistics.PredictiveActivatedCount,
            UnactivatedPairs = statistics.UnactivatedPairCount,
            MaxPenetration = statistics.MaxPenetration,
            AveragePenetration = statistics.AveragePenetration,
            TotalContactCorrection = statistics.TotalContactPositionCorrection,
            MaxContactCorrection = statistics.MaxContactPositionCorrection,
            TotalWallCorrection = statistics.TotalWallPositionCorrection,
            MaxWallCorrection = statistics.MaxWallPositionCorrection,
            AverageSpeedBefore = statistics.AverageSpeedBeforeContact,
            AverageSpeedAfter = statistics.AverageSpeedAfterContact,
            MaxVelocityChange = statistics.MaxVelocityChange,
            SoftAvoidanceMicroseconds = statistics.SoftAvoidanceNanoseconds / 1000f,
            SoftAvoidanceCandidatePairs = statistics.SoftAvoidanceCandidatePairCount,
            SoftAvoidanceActivatedPairs = statistics.SoftAvoidanceActivatedPairCount,
            SoftAvoidanceFatAabbUses = statistics.SoftAvoidanceFatAabbUseCount,
            PairGenerationMicroseconds = statistics.PairGenerationNanoseconds / 1000f,
            AverageIterationMicroseconds = statistics.AverageIterationNanoseconds / 1000f,
            SolverMicroseconds = statistics.SolverNanoseconds / 1000f,
            FatAabbCacheEnabled = settings.EnableFatAabbCache,
            FatAabbCacheValidAtFrameStart = shadow.CacheValidAtFrameStart != 0,
            FatAabbCacheValidAtFrameEnd = shadow.CacheValidAtFrameEnd != 0,
            FatAabbCacheAgeFrames = shadow.CacheAgeFrames,
            FatAabbCacheUses = shadow.CacheUseCount,
            FatAabbCacheReuses = shadow.CacheReuseCount,
            FatAabbCacheRebuilds = shadow.CacheRebuildCount,
            FatAabbCacheInvalidations = shadow.CacheInvalidationCount,
            FatAabbEntitySetInvalidations = shadow.EntitySetInvalidationCount,
            FatAabbBoundsInvalidations = shadow.BoundsInvalidationCount,
            FatAabbPostSolveInvalidations = shadow.PostSolveInvalidationCount,
            FatAabbFullBroadPhaseFallbacks = shadow.FullBroadPhaseFallbackCount,
            FatAabbCachedCandidatePairs = shadow.CachedCandidatePairCount,
            FatAabbNarrowPhasePairChecks = shadow.CachedNarrowPhasePairCheckCount,
            FatAabbMappingBuilds = shadow.CachePairMappingBuildCount,
            FatAabbMappingReuses = shadow.CachePairMappingReuseCount,
            FatAabbCorrectedBodyChecks = shadow.CorrectedBodyValidationCount,
            FatAabbBuildMicroseconds = shadow.CacheBuildNanoseconds / 1000f,
            FatAabbValidationMicroseconds = shadow.ValidationNanoseconds / 1000f,
            FatAabbMappingMicroseconds = shadow.CachePairMappingNanoseconds / 1000f,
            ShadowPreviousHits = shadow.PreviousFramePairHitCount,
            ShadowPreviousMisses = shadow.PreviousFramePairMissCount,
            ShadowCurrentHits = shadow.CurrentFramePairHitCount,
            ShadowCurrentMisses = shadow.CurrentFramePairMissCount,
            ShadowFinalEscapes = shadow.PreviousFrameFinalEscapeBodyCount +
                                 shadow.CurrentFrameFinalEscapeBodyCount,
            ResidualBeforeByIteration = residualBefore.ToArray(),
            ResidualAfterByIteration = residualAfter.ToArray()
        });

        _nextSampleTime = simulationTime + _interval;
    }

    public bool ReachedLimit(double simulationTime)
    {
        return Active &&
               (simulationTime - _startSimulationTime >= _duration ||
                _samples.Count >= MaxSamples);
    }

    public string StopAndWrite()
    {
        if (!Active)
            return LastOutputPath;

        Active = false;
        string outputDirectory = Path.Combine(
            Application.persistentDataPath,
            "Stage3ContactDiagnostics");
        Directory.CreateDirectory(outputDirectory);
        string labelSegment = string.IsNullOrEmpty(_runLabel)
            ? string.Empty
            : $"{_runLabel}-";
        string outputPath = Path.Combine(
            outputDirectory,
            $"stage3-contact-{labelSegment}{DateTime.Now:yyyyMMdd-HHmmss-fff}.json");
        var document = new Stage3ContactDiagnosticCaptureDocument
        {
            Format = "Stage3ContactDiagnostic/v2",
            RunLabel = _runLabel,
            CapturedAtUtc = DateTime.UtcNow.ToString("O"),
            RequestedDurationSeconds = _duration,
            SampleIntervalSeconds = _interval,
            SampleCount = _samples.Count,
            Samples = _samples.ToArray()
        };
        File.WriteAllText(outputPath, JsonUtility.ToJson(document, true));
        LastOutputPath = outputPath;
        return outputPath;
    }

    private static string SanitizeRunLabel(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
            return string.Empty;

        char[] buffer = new char[value.Length];
        int length = 0;
        for (int i = 0; i < value.Length; i++)
        {
            char character = value[i];
            if (char.IsLetterOrDigit(character) || character == '-' || character == '_')
                buffer[length++] = character;
        }
        return length > 0 ? new string(buffer, 0, length) : string.Empty;
    }
}

[Serializable]
public sealed class Stage3ContactDiagnosticCaptureDocument
{
    public string Format;
    public string RunLabel;
    public string CapturedAtUtc;
    public float RequestedDurationSeconds;
    public float SampleIntervalSeconds;
    public int SampleCount;
    public Stage3ContactDiagnosticCaptureSample[] Samples;
}

[Serializable]
public sealed class Stage3ContactDiagnosticCaptureSample
{
    public double SimulationTime;
    public float CaptureTime;
    public int Frame;
    public int Substeps;
    public int Iterations;
    public bool PredictiveGenerationEnabled;
    public bool SideExchangeConstraintEnabled;
    public string SoftAvoidanceVelocitySolver;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float RvoTimeHorizon;
    public int CandidatePairs;
    public int ContactPairs;
    public int ActualGeneratedPairs;
    public int PredictiveGeneratedPairs;
    public int SideExchangeRiskPairs;
    public int SideExchangePairs;
    public int ActiveConstraints;
    public int PredictiveActivatedConstraints;
    public int UnactivatedPairs;
    public float MaxPenetration;
    public float AveragePenetration;
    public float TotalContactCorrection;
    public float MaxContactCorrection;
    public float TotalWallCorrection;
    public float MaxWallCorrection;
    public float AverageSpeedBefore;
    public float AverageSpeedAfter;
    public float MaxVelocityChange;
    public float SoftAvoidanceMicroseconds;
    public int SoftAvoidanceCandidatePairs;
    public int SoftAvoidanceActivatedPairs;
    public int SoftAvoidanceFatAabbUses;
    public float PairGenerationMicroseconds;
    public float AverageIterationMicroseconds;
    public float SolverMicroseconds;
    public bool FatAabbCacheEnabled;
    public bool FatAabbCacheValidAtFrameStart;
    public bool FatAabbCacheValidAtFrameEnd;
    public int FatAabbCacheAgeFrames;
    public int FatAabbCacheUses;
    public int FatAabbCacheReuses;
    public int FatAabbCacheRebuilds;
    public int FatAabbCacheInvalidations;
    public int FatAabbEntitySetInvalidations;
    public int FatAabbBoundsInvalidations;
    public int FatAabbPostSolveInvalidations;
    public int FatAabbFullBroadPhaseFallbacks;
    public int FatAabbCachedCandidatePairs;
    public int FatAabbNarrowPhasePairChecks;
    public int FatAabbMappingBuilds;
    public int FatAabbMappingReuses;
    public int FatAabbCorrectedBodyChecks;
    public float FatAabbBuildMicroseconds;
    public float FatAabbValidationMicroseconds;
    public float FatAabbMappingMicroseconds;

    // 保留 v1 字段，便于旧分析脚本继续读取同一 JSON 类型。
    public int ShadowPreviousHits;
    public int ShadowPreviousMisses;
    public int ShadowCurrentHits;
    public int ShadowCurrentMisses;
    public int ShadowFinalEscapes;
    public float[] ResidualBeforeByIteration;
    public float[] ResidualAfterByIteration;
}
}
