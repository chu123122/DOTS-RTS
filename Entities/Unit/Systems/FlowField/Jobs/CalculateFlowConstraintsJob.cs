using System.Collections.Generic;
using Unity.Mathematics;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

public enum UnitContactMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// 单个 timestep 内复用的轻量单位接触约束。
/// PredictiveNormal 在 timestep 分类时固定；lambda 仍在每个 substep 开始时清零。
/// </summary>
public struct UnitCollisionPair
{
    public int BodyA;
    public int BodyB;
    public float Lambda;
    public float3 PredictiveNormal;
    public UnitContactMode ContactMode;
    public byte WasActivated;
    public byte WasActivatedThisTimestep;
    public byte WasCorrectedThisTimestep;
    public byte IsDormant;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
}

/// <summary>
/// 一个单位的 swept disc AABB 覆盖到的 Spatial Cell。
/// </summary>
public struct SweptDiscCellEntry
{
    public int CellIndex;
    public int BodyIndex;
}

public struct SweptDiscCellEntryComparer : IComparer<SweptDiscCellEntry>
{
    public int Compare(SweptDiscCellEntry x, SweptDiscCellEntry y)
    {
        int cellComparison = x.CellIndex.CompareTo(y.CellIndex);
        return cellComparison != 0
            ? cellComparison
            : x.BodyIndex.CompareTo(y.BodyIndex);
    }
}

public struct UnitCollisionPairComparer : IComparer<UnitCollisionPair>
{
    public int Compare(UnitCollisionPair x, UnitCollisionPair y)
    {
        int bodyAComparison = x.BodyA.CompareTo(y.BodyA);
        return bodyAComparison != 0
            ? bodyAComparison
            : x.BodyB.CompareTo(y.BodyB);
    }
}
}
