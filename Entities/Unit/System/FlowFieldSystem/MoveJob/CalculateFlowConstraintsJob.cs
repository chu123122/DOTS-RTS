using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
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

/// <summary>
/// 保留现有墙壁位置投影。Stage 3 只升级动态单位圆盘接触。
/// </summary>
[BurstCompile]
public partial struct CalculateWallConstraintsJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute([EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 positionCorrection = float3.zero;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = state.CellPosition + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost != 0)
                    continue;

                float3 wallPosition = GridOrigin + new float3(
                    checkCell.x * CellRadius * 2 + CellRadius,
                    state.PredictedPosition.y,
                    checkCell.y * CellRadius * 2 + CellRadius);

                AccumulateWallConstraint(state.PredictedPosition, wallPosition, ref positionCorrection);
            }
        }

        state.PositionCorrection = positionCorrection;
        States[entityIndex] = state;
    }

    private void AccumulateWallConstraint(
        float3 position,
        float3 wallPosition,
        ref float3 positionCorrection)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float wallHardRadius = CellRadius + 0.5f;
        if (dist >= wallHardRadius) return;

        float3 pushDirection = diff / dist;
        float penetration = wallHardRadius - dist;
        positionCorrection += pushDirection * (penetration * 0.5f);
    }
}
