using System.Numerics;
using Unity.Mathematics;
using System.Runtime.CompilerServices; // 用于提示内联

public static class FlowFieldUtils
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int GetFlatIndex(int2 cellPos, int2 gridDimensions)
    {
        return cellPos.y * gridDimensions.x + cellPos.x;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 GetCellPosFromIndex(int index, int2 gridDimensions)
    {
        return new int2(index % gridDimensions.x, index / gridDimensions.x);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 WorldToCell(float3 worldPos, float3 gridOrigin, float cellRadius)
    {
        // 简单的网格映射
        float cellSize = cellRadius * 2;
        float3 localPos = worldPos - gridOrigin;
        return new int2((int)(localPos.x / cellSize), (int)(localPos.z / cellSize));
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int2 GetDirectionOffset(int index)
    {
        // 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
        switch (index)
        {
            case 0: return new int2(0, 1);
            case 1: return new int2(1, 1);
            case 2: return new int2(1, 0);
            case 3: return new int2(1, -1);
            case 4: return new int2(0, -1);
            case 5: return new int2(-1, -1);
            case 6: return new int2(-1, 0);
            case 7: return new int2(-1, 1);
            default: return int2.zero;
        }
    }
}