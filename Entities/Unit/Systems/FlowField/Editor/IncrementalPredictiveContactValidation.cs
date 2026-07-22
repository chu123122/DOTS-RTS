#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Editor
{
public static class IncrementalPredictiveContactValidation
{
    [MenuItem("RTS/Diagnostics/Validate Incremental Predictive Contact Pipeline")]
    public static void ValidateLatestSnapshot()
    {
        IncrementalContactPipelineSnapshot snapshot =
            IncrementalContactPipelineDiagnosticsRuntime.Latest;
        IncrementalContactPipelineStatistics statistics = snapshot.Statistics;
        if (statistics.Timestep == 0)
        {
            Debug.LogWarning(
                "No incremental contact snapshot has been published. Enter Play Mode " +
                "and run at least one simulation timestep.");
            return;
        }

        string summary =
            $"Incremental Contact t={statistics.Timestep} | " +
            $"dirty={statistics.TopologyDirtyBodyCount}/{statistics.ProxyCount} " +
            $"neighbors={statistics.PersistentNeighborPairCount} " +
            $"swept={statistics.CurrentSweptContactCount} " +
            $"active={statistics.CurrentActiveConstraintCount} " +
            $"activated={statistics.UniqueActivatedPairCount} " +
            $"corrected={statistics.UniqueCorrectedPairCount} " +
            $"rebuild={statistics.FullRebuildCount} " +
            $"repair={statistics.IncrementalRepairCount} " +
            $"oracleMissing={statistics.OracleMissingPairCount}";

        if (statistics.OracleMismatch != 0 ||
            statistics.OracleMissingPairCount != 0)
        {
            Debug.LogError(summary +
                "\nIncremental contact validation failed: the O(N^2) oracle found " +
                "a swept contact missing from the active/scheduled pipeline.");
            return;
        }

        Debug.Log(summary +
            $"\nmode={snapshot.Mode}, schema=v{snapshot.SchemaVersion}; " +
            $"ratios: cleanProxy={snapshot.CleanProxyRatio:P1}, " +
            $"retainedPair={snapshot.RetainedNeighborPairRatio:P1}, " +
            $"neighbor->swept={snapshot.NeighborToSweptRatio:P1}, " +
            $"swept->active={snapshot.SweptToCurrentActiveRatio:P1}, " +
            $"activated->corrected={snapshot.ActivatedToCorrectedRatio:P1}");
    }
}
}
#endif
