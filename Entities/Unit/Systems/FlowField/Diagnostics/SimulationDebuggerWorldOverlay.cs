using Unity.Mathematics;
using UnityEngine;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Draws all world-space debugger views from the same immutable snapshot as the GUI.
/// Heatmaps use one translucent quad per diagnostic cell; selected-unit geometry uses
/// bounded line overlays so normal gameplay never renders every pair in the world.
/// </summary>
public sealed class SimulationDebuggerWorldOverlay : MonoBehaviour
{
    [Range(0f, 1f)] public float HeatmapOpacity = 0.28f;
    [Min(0f)] public float HeatmapHeight = 0.08f;
    [Min(0f)] public float BoundsHeight = 0.13f;
    [Min(0f)] public float PairHeight = 0.18f;
    public bool DrawRegions = true;
    public bool DrawSelectedBroadCells = true;

    private Material _material;

    private void OnDisable()
    {
        if (_material != null)
            Destroy(_material);
        _material = null;
    }

    private void LateUpdate()
    {
        if (!SimulationDebuggerRuntime.OverlayEnabled ||
            !SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot))
            return;

        DrawRegionsAndSelection(snapshot);
    }

    private void OnRenderObject()
    {
        if (!SimulationDebuggerRuntime.OverlayEnabled ||
            SimulationDebuggerRuntime.ActiveHeatmap == SimulationDebuggerHeatmap.None ||
            !SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot) ||
            snapshot.Cells.Count == 0)
            return;

        EnsureMaterial();
        _material.SetPass(0);
        GL.PushMatrix();
        GL.MultMatrix(Matrix4x4.identity);
        GL.Begin(GL.QUADS);
        for (int i = 0; i < snapshot.Cells.Count; i++)
        {
            SimulationDebuggerCellSample cell = snapshot.Cells[i];
            float value = GetHeatmapValue(cell, SimulationDebuggerRuntime.ActiveHeatmap);
            if (value <= 0.001f && cell.UnitCount == 0)
                continue;
            Color color = HeatmapColor(
                SimulationDebuggerRuntime.ActiveHeatmap,
                value,
                HeatmapOpacity);
            GL.Color(color);
            GL.Vertex3(cell.Min.x, HeatmapHeight, cell.Min.y);
            GL.Vertex3(cell.Max.x, HeatmapHeight, cell.Min.y);
            GL.Vertex3(cell.Max.x, HeatmapHeight, cell.Max.y);
            GL.Vertex3(cell.Min.x, HeatmapHeight, cell.Max.y);
        }
        GL.End();
        GL.PopMatrix();
    }

    private void DrawRegionsAndSelection(SimulationDebuggerFrameSnapshot snapshot)
    {
        if (DrawRegions &&
            SimulationDebuggerRuntime.ActiveView == SimulationDebuggerView.PersistentBroadPhase)
        {
            for (int i = 0; i < snapshot.Regions.Count; i++)
            {
                SimulationDebuggerRegionSample region = snapshot.Regions[i];
                DrawRect(region.CoreMin, region.CoreMax, BoundsHeight, new Color(1f, 0.75f, 0.12f));
                DrawRect(region.HaloMin, region.HaloMax, BoundsHeight + 0.01f, new Color(0.15f, 0.85f, 0.9f));
            }
        }

        if (!snapshot.HasSelectedUnit)
            return;

        SimulationDebuggerUnitSample unit = snapshot.SelectedUnit;
        if (unit.HasFatBounds != 0)
        {
            DrawRect(unit.SweptMin, unit.SweptMax, BoundsHeight + 0.02f, new Color(0.15f, 0.85f, 0.9f));
            DrawRect(unit.FatMin, unit.FatMax, BoundsHeight + 0.04f, new Color(0.9f, 0.2f, 0.85f));
            if (DrawSelectedBroadCells)
                DrawCoveredCells(snapshot, unit.SweptMin, unit.SweptMax);
        }

        Vector3 start = ToVector(unit.TimestepStartPosition, PairHeight);
        Vector3 unconstrained = ToVector(unit.UnconstrainedPosition, PairHeight);
        Vector3 final = ToVector(unit.FinalPosition, PairHeight);
        Debug.DrawLine(start, unconstrained, new Color(0.2f, 0.55f, 1f), 0f, false);
        Debug.DrawLine(unconstrained, final, new Color(1f, 0.45f, 0.12f), 0f, false);

        int count = Mathf.Min(
            SimulationDebuggerRuntime.MaximumVisualizedPairs,
            snapshot.SelectedPairs.Count);
        for (int i = 0; i < count; i++)
        {
            SimulationDebuggerPairSample pair = snapshot.SelectedPairs[i];
            Debug.DrawLine(
                ToVector(pair.PositionA, PairHeight + 0.01f),
                ToVector(pair.PositionB, PairHeight + 0.01f),
                PairColor(pair),
                0f,
                false);
        }
    }

    private void DrawCoveredCells(
        SimulationDebuggerFrameSnapshot snapshot,
        float2 sweptMin,
        float2 sweptMax)
    {
        for (int i = 0; i < snapshot.Cells.Count; i++)
        {
            SimulationDebuggerCellSample cell = snapshot.Cells[i];
            bool overlaps = cell.Min.x <= sweptMax.x && cell.Max.x >= sweptMin.x &&
                            cell.Min.y <= sweptMax.y && cell.Max.y >= sweptMin.y;
            if (overlaps)
                DrawRect(cell.Min, cell.Max, BoundsHeight + 0.025f, new Color(0.45f, 0.65f, 1f));
        }
    }

    private static float GetHeatmapValue(
        SimulationDebuggerCellSample cell,
        SimulationDebuggerHeatmap mode)
    {
        return math.saturate(mode switch
        {
            SimulationDebuggerHeatmap.OverallPressure => cell.OverallPressure,
            SimulationDebuggerHeatmap.UnitDensity => cell.Density,
            SimulationDebuggerHeatmap.SolverCorrection => cell.SolverCorrection,
            SimulationDebuggerHeatmap.AabbBenefit => cell.AabbBenefit,
            SimulationDebuggerHeatmap.AabbSlack => cell.AabbSlack,
            SimulationDebuggerHeatmap.CandidateExpansion => cell.CandidateExpansion,
            SimulationDebuggerHeatmap.EscapeRisk => cell.EscapeRisk,
            SimulationDebuggerHeatmap.ContactActivation => cell.ContactActivation,
            SimulationDebuggerHeatmap.ContactWaste => cell.ContactWaste,
            SimulationDebuggerHeatmap.ContactSupplementRisk => cell.ContactSupplementRisk,
            _ => 0f
        });
    }

    private static Color HeatmapColor(
        SimulationDebuggerHeatmap mode,
        float value,
        float opacity)
    {
        Color low;
        Color high;
        switch (mode)
        {
            case SimulationDebuggerHeatmap.AabbBenefit:
            case SimulationDebuggerHeatmap.AabbSlack:
            case SimulationDebuggerHeatmap.ContactActivation:
                low = new Color(0.9f, 0.2f, 0.18f, opacity * 0.35f);
                high = new Color(0.15f, 0.8f, 0.42f, opacity);
                break;
            default:
                low = new Color(0.12f, 0.4f, 0.95f, opacity * 0.25f);
                high = new Color(0.95f, 0.18f, 0.08f, opacity);
                break;
        }
        Color result = Color.Lerp(low, high, Mathf.Clamp01(value));
        result.a = Mathf.Lerp(low.a, high.a, Mathf.Clamp01(value));
        return result;
    }

    private static Color PairColor(SimulationDebuggerPairSample pair)
    {
        if (pair.Kind == SimulationDebuggerPairKind.SupplementedContact)
            return new Color(0.2f, 0.55f, 1f);
        if (pair.State == SimulationDebuggerPairState.Active)
        {
            return pair.Kind == SimulationDebuggerPairKind.PredictiveContact
                ? new Color(1f, 0.48f, 0.08f)
                : new Color(1f, 0.15f, 0.1f);
        }
        if (pair.Kind == SimulationDebuggerPairKind.PredictiveContact)
            return new Color(1f, 0.8f, 0.15f);
        return new Color(0.58f, 0.62f, 0.68f);
    }

    private static Vector3 ToVector(float3 value, float y)
    {
        return new Vector3(value.x, Mathf.Max(y, value.y + 0.02f), value.z);
    }

    private static void DrawRect(float2 min, float2 max, float y, Color color)
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

    private void EnsureMaterial()
    {
        if (_material != null)
            return;
        Shader shader = Shader.Find("Hidden/Internal-Colored");
        if (shader == null)
            return;
        _material = new Material(shader)
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        _material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        _material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        _material.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
        _material.SetInt("_ZWrite", 0);
    }
}
}
