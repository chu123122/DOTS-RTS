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
    public float Compliance;
    public float PredictiveSkin;
    public byte EnablePredictivePairGeneration;
    public byte EnablePredictiveContacts;
    public byte EnableFatAabbCache;
    public float FatAabbCacheMargin;
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
public struct SimulationOverviewMetrics
{
    public SimulationDebuggerHealth Health;
    public int UnitCount;
    public long SolverNanoseconds;
    public long SoftAvoidanceNanoseconds;
    public long PairGenerationNanoseconds;
    public long IterationNanoseconds;
    public int CandidatePairCount;
    public int ContactPairCount;
    public float MaxContactCorrection;
    public float MaxWallCorrection;
    public float MaxVelocityChange;

    public float SolverMilliseconds => SolverNanoseconds / 1_000_000f;
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
}

[Serializable]
public struct TimestepContactSetMetrics
{
    public SimulationDebuggerHealth Health;
    public int ContactSetSize;
    public int ActiveContactCount;
    public int InactiveContactCount;
    public int ActualContactCount;
    public int PredictiveContactCount;
    public int PredictiveActivatedCount;
    public int SupplementOrFallbackCount;
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
    public int CandidatePairCount;
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
    public ulong FrameId;
    public double ElapsedTime;
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public SimulationDebuggerCaptureMask CapturedMask;
    public SimulationOverviewMetrics Overview;
    public PersistentBroadPhaseMetrics BroadPhase;
    public TimestepContactSetMetrics ContactSet;
    public SimulationDebuggerEffectiveSettings EffectiveSettings;
    public SimulationDebuggerUnitSample SelectedUnit;
    public bool HasSelectedUnit;

    public readonly List<SimulationDebuggerCellSample> Cells = new List<SimulationDebuggerCellSample>();
    public readonly List<SimulationDebuggerRegionSample> Regions = new List<SimulationDebuggerRegionSample>();
    public readonly List<SimulationDebuggerProxySample> Proxies = new List<SimulationDebuggerProxySample>();
    public readonly List<SimulationDebuggerPairSample> SelectedPairs = new List<SimulationDebuggerPairSample>();

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
}
