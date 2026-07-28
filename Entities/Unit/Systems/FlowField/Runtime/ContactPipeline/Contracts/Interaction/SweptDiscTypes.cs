using System.Collections.Generic;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 帧级 swept-disc 源产出的一个空间 cell 成员关系。
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
}
