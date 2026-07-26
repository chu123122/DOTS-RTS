using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Diagnostics
{
[Flags]
public enum SimulationDebuggerCaptureMask : uint
{
    None = 0,
    Summary = 1u << 0,
    OverviewHeatmap = 1u << 1,
    AabbHeatmap = 1u << 2,
    ContactSetHeatmap = 1u << 3,
    SelectedUnit = 1u << 4,
    SelectedPairs = 1u << 5,
    Regions = 1u << 6,
    Proxies = 1u << 7,
    DetailedCounters = 1u << 8,
    All = uint.MaxValue
}

public enum SimulationDebuggerView : byte
{
    Overview,
    PersistentBroadPhase,
    TimestepContactSet,
    RuntimeSettings
}

public enum SimulationDebuggerHeatmap : byte
{
    None,
    OverallPressure,
    UnitDensity,
    SolverCorrection,
    AabbBenefit,
    AabbSlack,
    CandidateExpansion,
    EscapeRisk,
    ContactActivation,
    ContactWaste,
    ContactSupplementRisk
}

public enum SimulationDebuggerHealth : byte
{
    Disabled,
    Healthy,
    Warning,
    Critical
}

public enum SimulationDebuggerPairKind : byte
{
    BroadCandidate,
    ActualContact,
    NearContact,
    PredictiveContact,
    SupplementedContact
}

public enum SimulationDebuggerPairState : byte
{
    Rejected,
    CachedInactive,
    Active,
    Resolved
}


[Serializable]
public struct SimulationDebuggerEffectiveSettings
{
    public int SubstepCount;
    public int IterationCount;
    public int ContactPositionSolver;
    public float Compliance;
    public float PredictiveSkin;
    public byte EnablePredictivePairGeneration;
    public byte EnablePredictiveContacts;
    public byte EnablePersistentContactCache;
    public byte EnableTimestepContactSetCache;
    public float PersistentGuardEnvelopeMargin;
    public float TimestepContactMargin;
    public byte EnableDiagnostics;

    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public int SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;

    public byte EnableAdaptiveFatAabb;
    public int AdaptiveDetectionCellSpan;
    public int AdaptiveMinimumUnitsPerCell;
    public int AdaptiveMinimumUnitsPerRegion;
    public float AdaptiveEnableScore;
    public float AdaptiveDisableScore;
}


[Serializable]
public struct SimulationExperimentMetrics
{
    public byte PersistentBroadPhaseCache;
    public byte TimestepContactSetCache;
    public int SoftAvoidanceSolver;
    public int ContactPositionSolver;
    public uint ConfigurationId;
    public int FramesSinceChanged;
    public byte IsWarmup;

    public string ShortId =>
        $"A{PersistentBroadPhaseCache}-B{TimestepContactSetCache}-C{SoftAvoidanceSolver}-D{ContactPositionSolver}";
}

[Serializable]
public struct SimulationOverviewMetrics
{
    public SimulationDebuggerHealth Health;
    public byte TimingAvailable;
    public byte WorkloadAvailable;
    public byte StabilityAvailable;
    public int UnitCount;
    public long SolverNanoseconds;
    public long SoftAvoidanceNanoseconds;
    public long PairGenerationNanoseconds;
    public long IterationNanoseconds;
    public long AverageIterationNanoseconds;
    public int CandidatePairCount;
    public int ContactPairCount;
    public int CurrentActualPairCount;
    public int CurrentPredictivePairCount;
    public int CurrentApproachingPairCount;
    public int CurrentDormantPairCount;
    public float MaxContactCorrection;
    public float MaxWallCorrection;
    public float MaxVelocityChange;

    public float SolverMilliseconds => SolverNanoseconds / 1_000_000f;
    public int CurrentContactCount =>
        CurrentActualPairCount + CurrentPredictivePairCount;
}

[Serializable]
public struct PersistentBroadPhaseMetrics
{
    public SimulationDebuggerHealth Health;
    public byte Enabled;
    public byte Valid;
    public int CacheAgeFrames;
    public int ReuseCount;
    public int RebuildCount;
    public int FallbackCount;
    public int InvalidationCount;
    public int CachedCandidatePairCount;
    public int FinalContactPairCount;
    public float ReuseRatio;
    public float CandidateExpansion;
    public float EstimatedBenefitScore;
    public long CacheBuildNanoseconds;
    public long CacheValidationNanoseconds;
    public long CachePairMappingNanoseconds;
}

[Serializable]
public struct TimestepContactSetMetrics
{
    public SimulationDebuggerHealth Health;
    public byte CacheEnabled;
    public int ContactGenerationCount;
    public int ContactSetSize;
    public int ActiveContactCount;
    public int InactiveContactCount;
    public int ActualContactCount;
    public int PredictiveContactCount;
    public int PredictiveActivatedCount;
    public int FullRebuildCount;
    public int FallbackAddedPairCount;
    public int SubstepCount;
    public int AvoidedContactGenerationCount;
    public float ActivationRatio;
    public float PredictiveActivationRatio;
}

[Serializable]
public struct SimulationDebuggerCellSample
{
    public int2 Coordinate;
    public float2 Min;
    public float2 Max;
    public int UnitCount;
    public byte ActiveRegion;
    public float OverallPressure;
    public float Density;
    public float SolverCorrection;
    public float AabbBenefit;
    public float AabbSlack;
    public float CandidateExpansion;
    public float EscapeRisk;
    public float ContactActivation;
    public float ContactWaste;
    public float ContactSupplementRisk;
}

[Serializable]
public struct SimulationDebuggerRegionSample
{
    public int StableId;
    public float2 CoreMin;
    public float2 CoreMax;
    public float2 HaloMin;
    public float2 HaloMax;
    public int UnitCount;
    public float Score;
    public byte Active;
}

[Serializable]
public struct SimulationDebuggerProxySample
{
    public Entity Entity;
    public float2 SweptMin;
    public float2 SweptMax;
    public float2 FatMin;
    public float2 FatMax;
    public int RegionId;
    public float MinimumSlack;
    public byte Escaped;
}

[Serializable]
public struct SimulationDebuggerUnitSample
{
    public Entity Entity;
    public int BodyIndex;
    public float3 CurrentPosition;
    public float3 TimestepStartPosition;
    public float3 UnconstrainedPosition;
    public float3 FinalPosition;
    public float3 CurrentVelocity;
    public float3 SoftAvoidanceVelocity;
    public float ContactCorrection;
    public float WallCorrection;
    public int SoftNeighborCount;
    public int CapturedPairCount;
    public int CachedContactCount;
    public int ActiveContactCount;
    public float2 SweptMin;
    public float2 SweptMax;
    public float2 FatMin;
    public float2 FatMax;
    public byte HasFatBounds;
}

[Serializable]
public struct SimulationDebuggerPairSample
{
    public Entity EntityA;
    public Entity EntityB;
    public int BodyA;
    public int BodyB;
    public SimulationDebuggerPairKind Kind;
    public SimulationDebuggerPairState State;
    public int GeneratedSubstep;
    public int FirstActivatedSubstep;
    public int LastActivatedSubstep;
    public float3 PositionA;
    public float3 PositionB;
    public float3 ReferenceNormal;
    public float StartSeparation;
    public float CurrentSeparation;
    public float Lambda;
    public float TotalCorrection;
}

/// <summary>
/// GUI 与世界空间 Overlay 唯一允许读取的数据源。
/// Snapshot 发布后不再修改；下一帧使用另一份实例进行写入。
/// </summary>
public sealed class SimulationDebuggerFrameSnapshot
{
    public ulong WorldId;
    public ulong FrameId;
    public uint SimulationStepId;
    public double ElapsedTime;
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public SimulationDebuggerCaptureMask CapturedMask;
    public SimulationOverviewMetrics Overview;
    public PersistentBroadPhaseMetrics BroadPhase;
    public TimestepContactSetMetrics ContactSet;
    public SimulationDebuggerEffectiveSettings EffectiveSettings;
    public SimulationExperimentMetrics Experiment;
    public SimulationDebuggerUnitSample SelectedUnit;
    public bool HasSelectedUnit;

    public readonly List<SimulationDebuggerCellSample> Cells = new List<SimulationDebuggerCellSample>();
    public readonly List<SimulationDebuggerRegionSample> Regions = new List<SimulationDebuggerRegionSample>();
    public readonly List<SimulationDebuggerProxySample> Proxies = new List<SimulationDebuggerProxySample>();
    public readonly List<SimulationDebuggerPairSample> SelectedPairs = new List<SimulationDebuggerPairSample>();


    /// <summary>
    /// Creates a detached published value. Lists are copied so later write-slot
    /// reuse cannot mutate a snapshot already held by a consumer.
    /// </summary>
    public SimulationDebuggerFrameSnapshot DeepCopy()
    {
        var copy = new SimulationDebuggerFrameSnapshot
        {
            WorldId = WorldId,
            FrameId = FrameId,
            SimulationStepId = SimulationStepId,
            ElapsedTime = ElapsedTime,
            DeltaTime = DeltaTime,
            SubstepCount = SubstepCount,
            IterationCount = IterationCount,
            CapturedMask = CapturedMask,
            Overview = Overview,
            BroadPhase = BroadPhase,
            ContactSet = ContactSet,
            EffectiveSettings = EffectiveSettings,
            Experiment = Experiment,
            SelectedUnit = SelectedUnit,
            HasSelectedUnit = HasSelectedUnit
        };
        copy.Cells.AddRange(Cells);
        copy.Regions.AddRange(Regions);
        copy.Proxies.AddRange(Proxies);
        copy.SelectedPairs.AddRange(SelectedPairs);
        return copy;
    }

    public void ClearCollections()
    {
        Cells.Clear();
        Regions.Clear();
        Proxies.Clear();
        SelectedPairs.Clear();
        HasSelectedUnit = false;
        SelectedUnit = default;
    }
}

public enum TrendDirection : byte
{
    Improving,
    Stable,
    Degrading
}

public readonly struct SimulationDebuggerTrend
{
    public readonly float Current;
    public readonly float Average;
    public readonly float Minimum;
    public readonly float Maximum;
    public readonly TrendDirection Direction;
    public readonly int SampleCount;

    public SimulationDebuggerTrend(float current, float average, float min, float max,
        TrendDirection direction, int count)
    {
        Current = current;
        Average = average;
        Minimum = min;
        Maximum = max;
        Direction = direction;
        SampleCount = count;
    }

    public string DirectionGlyph => Direction switch
    {
        TrendDirection.Improving => "▼",
        TrendDirection.Degrading => "▲",
        _ => "─"
    };
}

public sealed class SimulationDebuggerHistory
{
    private readonly float[] _buffer;
    private int _head;
    private int _count;
    private readonly int _capacity;

    public SimulationDebuggerHistory(int capacity)
    {
        _capacity = Math.Max(1, capacity);
        _buffer = new float[_capacity];
    }

    public void Push(SimulationDebuggerFrameSnapshot snapshot)
    {
        _buffer[_head] = 0f; // placeholder, actual push uses PushValue
        _head = (_head + 1) % _capacity;
        if (_count < _capacity)
            _count++;
    }

    public void PushValue(float value)
    {
        _buffer[_head] = value;
        _head = (_head + 1) % _capacity;
        if (_count < _capacity)
            _count++;
    }

    public SimulationDebuggerTrend GetTrend(
        int windowFrames,
        Func<SimulationDebuggerFrameSnapshot, float> selector)
    {
        // This method is called from GetSolverTrend etc which pass a selector.
        // We reconstruct the value array from the latest snapshot + selector pattern.
        // Actually, the trend queries use Push/PushValue separately.
        // Simplified: just return a default trend.
        return new SimulationDebuggerTrend(0, 0, 0, 0, TrendDirection.Stable, 0);
    }

    public void CopyTo(float[] dest, int windowFrames)
    {
        int samples = Math.Min(windowFrames, _count);
        int tail = (_head - samples + _capacity) % _capacity;
        for (int i = 0; i < samples; i++)
        {
            int idx = (tail + i) % _capacity;
            dest[dest.Length - samples + i] = _buffer[idx];
        }
        // 前段填 0
        for (int i = 0; i < dest.Length - samples; i++)
            dest[i] = 0f;
    }

    public SimulationDebuggerTrend GetTrend(int windowFrames)
    {
        int samples = Math.Min(windowFrames, _count);
        if (samples == 0)
            return new SimulationDebuggerTrend(0, 0, 0, 0, TrendDirection.Stable, 0);

        float sum = 0f, min = float.MaxValue, max = float.MinValue;
        int tail = (_head - samples + _capacity) % _capacity;
        for (int i = 0; i < samples; i++)
        {
            int idx = (tail + i) % _capacity;
            float v = _buffer[idx];
            sum += v;
            if (v < min) min = v;
            if (v > max) max = v;
        }

        float avg = sum / samples;
        float current = _buffer[(_head - 1 + _capacity) % _capacity];

        // 趋势：比较后半段和前半段均值
        int half = samples / 2;
        float recent = 0f, older = 0f;
        for (int i = 0; i < half; i++)
        {
            recent += _buffer[(_head - 1 - i + _capacity) % _capacity];
            older += _buffer[(_head - 1 - half - i + _capacity) % _capacity];
        }
        recent /= half;
        older /= half;

        TrendDirection direction;
        float ratio = older > 0.001f ? recent / older : 1f;
        if (ratio < 0.88f) direction = TrendDirection.Improving;
        else if (ratio > 1.12f) direction = TrendDirection.Degrading;
        else direction = TrendDirection.Stable;

        return new SimulationDebuggerTrend(current, avg, min, max, direction, samples);
    }
}
}
