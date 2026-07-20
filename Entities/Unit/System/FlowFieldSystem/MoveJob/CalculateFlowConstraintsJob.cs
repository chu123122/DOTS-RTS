using System.Collections.Generic;
using Unity.Mathematics;

public enum UnitContactMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// 单个 substep 内复用的轻量单位接触约束。
/// InitialNormal 不重复保存；Predictive 模式始终由双方 StartPosition 稳定推导。
/// </summary>
public struct UnitCollisionPair
{
    public int BodyA;
    public int BodyB;
    public float Lambda;
    public UnitContactMode ContactMode;
    public byte WasActivated;
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
