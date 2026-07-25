using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
#if RTS_CONTACT_DIAGNOSTICS
    private struct SimulationDebuggerCellAccumulator
    {
        public int UnitCount;
        public int ContactPairDegree;
        public int ActivePairDegree;
        public int PredictivePairDegree;
        public float MaximumContactCorrection;
        public int EscapedUnitCount;
        public int FallbackUnitCount;

        public int ProxyCount;
        public int InvalidProxyCount;
        public float AabbSlackSum;
        public float CandidateExpansionSum;
    }
#endif

    private void CaptureSpatialDiagnostics(
        SimulationDebuggerFrameSnapshot snapshot,
        FlowFieldGrid gridComponent,
        SimulationDebuggerCaptureMask captureMask)
    {
#if RTS_CONTACT_DIAGNOSTICS
        bool captureCells = (captureMask & (
            SimulationDebuggerCaptureMask.OverviewHeatmap |
            SimulationDebuggerCaptureMask.AabbHeatmap |
            SimulationDebuggerCaptureMask.ContactSetHeatmap |
            SimulationDebuggerCaptureMask.SelectedUnit)) != 0;
        bool captureProxies = (captureMask & (
            SimulationDebuggerCaptureMask.AabbHeatmap |
            SimulationDebuggerCaptureMask.Proxies |
            SimulationDebuggerCaptureMask.SelectedUnit)) != 0;
        if ((!captureCells && !captureProxies) ||
            !gridComponent.Grid.IsCreated ||
            gridComponent.GridDimensions.x <= 0 ||
            gridComponent.GridDimensions.y <= 0 ||
            gridComponent.CellRadius <= 0f)
            return;

        var cells = new Dictionary<int, SimulationDebuggerCellAccumulator>();

        if (captureCells &&
            _incrementalDiagnosticsEntity != Entity.Null &&
            EntityManager.Exists(_incrementalDiagnosticsEntity) &&
            EntityManager.HasBuffer<Stage3ContactHeatSample>(
                _incrementalDiagnosticsEntity))
        {
            DynamicBuffer<Stage3ContactHeatSample> heatSamples =
                EntityManager.GetBuffer<Stage3ContactHeatSample>(
                    _incrementalDiagnosticsEntity);
            for (int i = 0; i < heatSamples.Length; i++)
            {
                Stage3ContactHeatSample sample = heatSamples[i];
                if (!TryResolveDiagnosticCell(
                        sample.Position,
                        gridComponent,
                        out int flatIndex,
                        out _))
                    continue;

                cells.TryGetValue(
                    flatIndex,
                    out SimulationDebuggerCellAccumulator accumulator);
                accumulator.UnitCount++;
                accumulator.ContactPairDegree += math.max(
                    0,
                    sample.ContactPairDegree);
                accumulator.ActivePairDegree += math.max(
                    0,
                    sample.ActivePairDegree);
                accumulator.PredictivePairDegree += math.max(
                    0,
                    sample.PredictivePairDegree);
                accumulator.MaximumContactCorrection = math.max(
                    accumulator.MaximumContactCorrection,
                    math.max(0f, sample.ContactCorrection));
                accumulator.EscapedUnitCount += sample.Escaped != 0 ? 1 : 0;
                accumulator.FallbackUnitCount +=
                    sample.HasFallbackPair != 0 ? 1 : 0;
                cells[flatIndex] = accumulator;
            }
        }

        if (_candidateStore.SweptProxies.IsCreated)
        {
            for (int i = 0; i < _candidateStore.SweptProxies.Length; i++)
            {
                PersistentSweptProxy proxy = _candidateStore.SweptProxies[i];
                if (captureProxies)
                {
                    float minimumSlack = CalculateMinimumProxySlack(proxy);
                    snapshot.Proxies.Add(new SimulationDebuggerProxySample
                    {
                        Entity = proxy.Entity,
                        SweptMin = proxy.TightMin,
                        SweptMax = proxy.TightMax,
                        FatMin = proxy.GuardMin,
                        FatMax = proxy.GuardMax,
                        RegionId = -1,
                        MinimumSlack = minimumSlack,
                        Escaped = (byte)(
                            proxy.IsValid == 0 || minimumSlack < 0f ? 1 : 0)
                    });
                }

                if (!captureCells)
                    continue;

                float2 center = (proxy.TightMin + proxy.TightMax) * 0.5f;
                float3 worldCenter = new float3(
                    center.x,
                    gridComponent.GridOrigin.y,
                    center.y);
                if (!TryResolveDiagnosticCell(
                        worldCenter,
                        gridComponent,
                        out int flatIndex,
                        out _))
                    continue;

                cells.TryGetValue(
                    flatIndex,
                    out SimulationDebuggerCellAccumulator accumulator);
                accumulator.ProxyCount++;
                float minimumProxySlack = CalculateMinimumProxySlack(proxy);
                accumulator.InvalidProxyCount +=
                    proxy.IsValid == 0 || minimumProxySlack < 0f ? 1 : 0;
                accumulator.AabbSlackSum += math.max(0f, minimumProxySlack);
                accumulator.CandidateExpansionSum +=
                    CalculateProxyExpansion(proxy);
                cells[flatIndex] = accumulator;
            }
        }

        if (!captureCells || cells.Count == 0)
            return;

        var orderedCellIndices = new List<int>(cells.Keys);
        orderedCellIndices.Sort();
        float cellSize = gridComponent.CellRadius * 2f;
        float cellAreaScale = math.max(0.0001f, cellSize);
        const float densityReferenceUnitCount = 8f;
        const float criticalCorrection = 0.25f;
        const float expansionReference = 3f;

        for (int i = 0; i < orderedCellIndices.Count; i++)
        {
            int flatIndex = orderedCellIndices[i];
            SimulationDebuggerCellAccumulator accumulator = cells[flatIndex];
            int2 coordinate = new int2(
                flatIndex % gridComponent.GridDimensions.x,
                flatIndex / gridComponent.GridDimensions.x);
            float2 minimum = gridComponent.GridOrigin.xz +
                             new float2(coordinate.x, coordinate.y) * cellSize;
            float2 maximum = minimum + new float2(cellSize);

            float density = math.saturate(
                accumulator.UnitCount / densityReferenceUnitCount);
            float contactActivation = accumulator.ContactPairDegree > 0
                ? math.saturate(
                    accumulator.ActivePairDegree /
                    (float)accumulator.ContactPairDegree)
                : 0f;
            float contactWaste = accumulator.ContactPairDegree > 0
                ? math.saturate(
                    (accumulator.ContactPairDegree -
                     accumulator.ActivePairDegree) /
                    (float)accumulator.ContactPairDegree)
                : 0f;
            float solverCorrection = math.saturate(
                accumulator.MaximumContactCorrection / criticalCorrection);
            float heatEscapeRisk = accumulator.UnitCount > 0
                ? accumulator.EscapedUnitCount /
                  (float)accumulator.UnitCount
                : 0f;
            float proxyEscapeRisk = accumulator.ProxyCount > 0
                ? accumulator.InvalidProxyCount /
                  (float)accumulator.ProxyCount
                : 0f;
            float escapeRisk = math.saturate(
                math.max(heatEscapeRisk, proxyEscapeRisk));
            float supplementRisk = accumulator.UnitCount > 0
                ? math.saturate(
                    accumulator.FallbackUnitCount /
                    (float)accumulator.UnitCount)
                : 0f;
            float aabbSlack = accumulator.ProxyCount > 0
                ? math.saturate(
                    accumulator.AabbSlackSum /
                    accumulator.ProxyCount /
                    cellAreaScale)
                : 0f;
            float candidateExpansion = accumulator.ProxyCount > 0
                ? math.saturate(
                    accumulator.CandidateExpansionSum /
                    accumulator.ProxyCount /
                    expansionReference)
                : 0f;
            float aabbBenefit = math.saturate(
                aabbSlack * (1f - candidateExpansion));
            float overallPressure = math.saturate(
                density * 0.30f +
                contactActivation * 0.20f +
                solverCorrection * 0.20f +
                escapeRisk * 0.15f +
                supplementRisk * 0.15f);

            snapshot.Cells.Add(new SimulationDebuggerCellSample
            {
                Coordinate = coordinate,
                Min = minimum,
                Max = maximum,
                UnitCount = accumulator.UnitCount,
                ActiveRegion = (byte)(
                    accumulator.ProxyCount > 0 ? 1 : 0),
                OverallPressure = overallPressure,
                Density = density,
                SolverCorrection = solverCorrection,
                AabbBenefit = aabbBenefit,
                AabbSlack = aabbSlack,
                CandidateExpansion = candidateExpansion,
                EscapeRisk = escapeRisk,
                ContactActivation = contactActivation,
                ContactWaste = contactWaste,
                ContactSupplementRisk = supplementRisk
            });
        }

        // Adaptive region execution has been retired. Regions remain empty until a
        // new authoritative region source exists; presentation must not synthesize
        // fake region identities from per-cell diagnostics.
#endif
    }

#if RTS_CONTACT_DIAGNOSTICS
    private static bool TryResolveDiagnosticCell(
        float3 position,
        FlowFieldGrid gridComponent,
        out int flatIndex,
        out int2 coordinate)
    {
        coordinate = FlowFieldUtils.WorldToCell(
            position,
            gridComponent.GridOrigin,
            gridComponent.CellRadius);
        if (coordinate.x < 0 ||
            coordinate.x >= gridComponent.GridDimensions.x ||
            coordinate.y < 0 ||
            coordinate.y >= gridComponent.GridDimensions.y)
        {
            flatIndex = -1;
            return false;
        }

        flatIndex = FlowFieldUtils.GetFlatIndex(
            coordinate,
            gridComponent.GridDimensions);
        return true;
    }

    private static float CalculateMinimumProxySlack(
        PersistentSweptProxy proxy)
    {
        float2 lower = proxy.TightMin - proxy.GuardMin;
        float2 upper = proxy.GuardMax - proxy.TightMax;
        return math.cmin(math.min(lower, upper));
    }

    private static float CalculateProxyExpansion(
        PersistentSweptProxy proxy)
    {
        float2 tightSize = math.max(
            proxy.TightMax - proxy.TightMin,
            new float2(0.0001f));
        float2 guardSize = math.max(
            proxy.GuardMax - proxy.GuardMin,
            tightSize);
        float tightArea = tightSize.x * tightSize.y;
        float guardArea = guardSize.x * guardSize.y;
        return math.max(0f, guardArea / tightArea - 1f);
    }
#endif
}
}
