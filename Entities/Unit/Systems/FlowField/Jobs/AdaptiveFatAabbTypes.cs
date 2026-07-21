using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
public struct AdaptiveFatAabbCellHistory
{
    public ulong OccupancyBloom;
    public float SmoothedScore;
    public float SmoothedCorrection;
    public float SmoothedEscapePenalty;
    public ushort EnableStreak;
    public ushort DisableStreak;
    public byte Active;
}

public struct AdaptiveFatAabbCellMetric
{
    public int UnitCount;
    public float SpeedSum;
    public float CorrectionSum;
    public float Score;
    public float DensityScore;
    public float PersistenceScore;
    public float PressureScore;
    public float EscapeRiskScore;
    public ulong OccupancyBloom;
    public int RegionIndex;
    public byte Active;
}

public struct AdaptiveFatAabbRegion
{
    public int StableId;
    public int2 MinCell;
    public int2 MaxCell;
    public int UnitCount;
    public float AverageScore;
    public byte Active;
}

public struct AdaptiveFatAabbRegionHistory
{
    public int StableId;
    public int2 MinCell;
    public int2 MaxCell;
    public float LastScore;
    public int AgeFrames;
    public int MissingFrames;
}

public struct AdaptiveFatAabbBodyRouting
{
    public int CoreRegionIndex;
    public int FatRegionIndex;
    public byte IsCore;
    public byte IsBoundary;
    public byte IsFatParticipant;
    public byte UseNormalBroadPhase;
}

public struct AdaptiveFatAabbDebugCell
{
    public float2 Min;
    public float2 Max;
    public float Score;
    public float DensityScore;
    public float PersistenceScore;
    public float PressureScore;
    public float EscapeRiskScore;
    public float AverageCorrection;
    public float CachePenalty;
    public int UnitCount;
    public byte Active;
}

public struct AdaptiveFatAabbDebugRegion
{
    public float2 CoreMin;
    public float2 CoreMax;
    public float2 HaloMin;
    public float2 HaloMax;
    public float Score;
    public int StableId;
    public int UnitCount;
    public byte Active;
}

public struct AdaptiveFatAabbDebugProxy
{
    public Entity Entity;
    public float2 CoreMin;
    public float2 CoreMax;
    public float2 FatMin;
    public float2 FatMax;
    public float MinimumSlack;
    public int RegionIndex;
    public byte Escaped;
}

public struct AdaptiveFatAabbCacheFeedback
{
    public float EscapePenalty;
    public float CandidateExpansionRatio;
    public float ReuseRatio;
    public int ReuseCount;
    public int RebuildCount;
    public int FallbackCount;
}
}
