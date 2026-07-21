using Unity.Mathematics;
using UnityEngine;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private void EnsureAdaptiveFatAabbHistory(
        int2 flowGridDimensions,
        AdaptiveFatAabbSettings settings)
    {
        int span = math.max(1, settings.DetectionCellSpan);
        int2 dimensions = new int2(
            math.max(1, (flowGridDimensions.x + span - 1) / span),
            math.max(1, (flowGridDimensions.y + span - 1) / span));
        int requiredLength = dimensions.x * dimensions.y;
        if (_adaptiveCellDimensions.Equals(dimensions) &&
            _adaptiveCellSpan == span &&
            _adaptiveCellHistory.Length == requiredLength)
            return;

        Dependency.Complete();
        _adaptiveCellDimensions = dimensions;
        _adaptiveCellSpan = span;
        _adaptiveCellHistory.ResizeUninitialized(requiredLength);
        for (int i = 0; i < requiredLength; i++)
            _adaptiveCellHistory[i] = default;
        _adaptiveRegions.Clear();
        _adaptiveDebugCells.Clear();
        _adaptiveDebugRegions.Clear();
        _adaptiveDebugProxies.Clear();
        _adaptiveRegionHistory.Clear();
        _adaptiveNextRegionId.Value = 1;
        _adaptiveCacheFeedback.Value = default;
        _shadowPreviousProxies.Clear();
        _shadowPreviousPairs.Clear();
        _fatAabbCacheState.Value = default;
    }

    private void DrawAdaptiveFatAabbDebug(AdaptiveFatAabbSettings settings)
    {
        if (settings.DrawDebug == 0 || !_adaptiveDebugCells.IsCreated)
            return;

        // Debug 绘制允许同步上一帧结果；关闭 DrawDebug 时不会引入这个同步点。
        Dependency.Complete();
        float y = settings.DebugHeight;

        for (int i = 0; i < _adaptiveDebugCells.Length; i++)
        {
            AdaptiveFatAabbDebugCell cell = _adaptiveDebugCells[i];
            Color heat = Color.Lerp(
                new Color(0.15f, 0.35f, 1f, 1f),
                new Color(1f, 0.15f, 0.05f, 1f),
                math.saturate(cell.Score));
            if (cell.Active != 0)
                heat = Color.Lerp(heat, Color.yellow, 0.45f);
            DrawDebugRect(cell.Min, cell.Max, y, heat);
        }

        for (int i = 0; i < _adaptiveDebugRegions.Length; i++)
        {
            AdaptiveFatAabbDebugRegion region = _adaptiveDebugRegions[i];
            DrawDebugRect(region.CoreMin, region.CoreMax, y + 0.02f, Color.yellow);
            DrawDebugRect(region.HaloMin, region.HaloMax, y + 0.04f, Color.cyan);
        }

        for (int i = 0; i < _adaptiveDebugProxies.Length; i++)
        {
            AdaptiveFatAabbDebugProxy proxy = _adaptiveDebugProxies[i];
            DrawDebugRect(proxy.CoreMin, proxy.CoreMax, y + 0.06f, Color.white);
            DrawDebugRect(proxy.FatMin, proxy.FatMax, y + 0.08f, Color.magenta);
        }
    }

    private static void DrawDebugRect(float2 min, float2 max, float y, Color color)
    {
        Vector3 a = new Vector3(min.x, y, min.y);
        Vector3 b = new Vector3(max.x, y, min.y);
        Vector3 c = new Vector3(max.x, y, max.y);
        Vector3 d = new Vector3(min.x, y, max.y);
        Debug.DrawLine(a, b, color, 0f, false);
        Debug.DrawLine(b, c, color, 0f, false);
        Debug.DrawLine(c, d, color, 0f, false);
        Debug.DrawLine(d, a, color, 0f, false);
    }
}
}
