using System.Collections.Generic;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// One spatial-cell membership produced by the frame-local swept-disc source.
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
