#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Editor
{
public sealed class IncrementalContactPipelineDebuggerWindow : EditorWindow
{
    private enum PanelPage
    {
        Summary,
        Details,
        Recording
    }

    private PanelPage _page;
    private Vector2 _scroll;
    private bool _showLegacy;

    [MenuItem("RTS/Diagnostics/Incremental Contact Pipeline")]
    public static void Open()
    {
        GetWindow<IncrementalContactPipelineDebuggerWindow>("Contact Pipeline");
    }

    private void OnEnable()
    {
        EditorApplication.update += Repaint;
    }

    private void OnDisable()
    {
        EditorApplication.update -= Repaint;
    }

    private void OnGUI()
    {
        _page = (PanelPage)GUILayout.Toolbar((int)_page,
            new[] { "Summary", "Details", "Recording" });

        IncrementalContactPipelineSnapshot snapshot =
            IncrementalContactPipelineDiagnosticsRuntime.Latest;
        if (snapshot.Statistics.Timestep == 0)
        {
            EditorGUILayout.HelpBox(
                "No v3 contact-pipeline snapshot is available. Enter Play Mode and run at least one simulation timestep.",
                MessageType.Info);
            return;
        }

        _scroll = EditorGUILayout.BeginScrollView(_scroll);
        switch (_page)
        {
            case PanelPage.Summary:
                DrawSummary(snapshot);
                break;
            case PanelPage.Details:
                DrawDetails(snapshot);
                break;
            case PanelPage.Recording:
                DrawRecording();
                break;
        }
        EditorGUILayout.EndScrollView();
    }

    private static void DrawSummary(IncrementalContactPipelineSnapshot snapshot)
    {
        IncrementalContactPipelineStatistics statistics = snapshot.Statistics;
        var solver = snapshot.SolverStatistics;

        EditorGUILayout.LabelField("General", EditorStyles.boldLabel);
        Metric("Mode", snapshot.Mode.ToString());
        Metric("Timestep", statistics.Timestep.ToString());
        Metric("Units", snapshot.Configuration.UnitCount.ToString());
        Metric("Solver", NsToMs(solver.SolverNanoseconds));
        Metric("Soft avoidance", NsToMs(solver.SoftAvoidanceNanoseconds));
        Metric("XPBD iterations", NsToMs(solver.IterationNanoseconds));
        Metric("Average penetration", solver.AveragePenetration.ToString("G5"));
        Metric("Max contact correction", solver.MaxContactPositionCorrection.ToString("G5"));

        Space();
        EditorGUILayout.LabelField("Contact funnel", EditorStyles.boldLabel);
        FunnelRow("Persistent neighbors", statistics.PersistentNeighborPairCount, null);
        FunnelRow("Swept contacts", statistics.CurrentSweptContactCount,
            snapshot.NeighborToSweptRatio);
        FunnelRow("Current active", statistics.CurrentActiveConstraintCount,
            snapshot.SweptToCurrentActiveRatio);
        FunnelRow("Unique corrected", statistics.UniqueCorrectedPairCount,
            snapshot.ActivatedToCorrectedRatio);

        Space();
        EditorGUILayout.LabelField("Incremental health", EditorStyles.boldLabel);
        Metric("Topology dirty", $"{statistics.TopologyDirtyBodyCount}/{statistics.ProxyCount} ({snapshot.TopologyDirtyRatio:P1})");
        Metric("Motion dirty", statistics.MotionDirtyBodyCount.ToString());
        Metric("Escaped bodies", statistics.CorrectedEscapeBodyCount.ToString());
        Metric("Pair delta", $"+{statistics.NeighborPairAddedCount} / -{statistics.NeighborPairRemovedCount}");
        Metric("Rebuild / repair", $"{statistics.FullRebuildCount} / {statistics.IncrementalRepairCount}");

        if (statistics.OracleMissingPairCount != 0 || statistics.OracleMismatch != 0)
        {
            EditorGUILayout.HelpBox(
                $"Oracle mismatch: missing={statistics.OracleMissingPairCount}, extra={statistics.OracleExtraPairCount}. " +
                "The incremental cache will be invalidated and rebuilt.",
                MessageType.Error);
        }
        else
        {
            EditorGUILayout.HelpBox(
                $"Oracle OK · compared pairs={statistics.OraclePairCount} · extra conservative pairs={statistics.OracleExtraPairCount}",
                MessageType.None);
        }
    }

    private void DrawDetails(IncrementalContactPipelineSnapshot snapshot)
    {
        IncrementalContactPipelineStatistics statistics = snapshot.Statistics;
        var solver = snapshot.SolverStatistics;
        IncrementalContactPipelineConfiguration configuration = snapshot.Configuration;

        EditorGUILayout.LabelField("Effective configuration", EditorStyles.boldLabel);
        Metric("Schema", $"snapshot v{snapshot.SchemaVersion}, csv v{IncrementalContactPipelineCsvRecorderRuntime.CsvSchemaVersion}");
        Metric("Experiment", configuration.ExperimentId.ToString());
        Metric("Scenario", configuration.Scenario.ToString());
        Metric("Configuration", configuration.ConfigurationLabel.ToString());
        Metric("Units", configuration.UnitCount.ToString());
        Metric("dt / substeps / iterations",
            $"{configuration.DeltaTime:G5} / {configuration.SubstepCount} / {configuration.IterationCount}");
        Metric("Guard / predictive / contact margin",
            $"{configuration.GuardEnvelopeMargin:G5} / {configuration.PredictiveSkin:G5} / {configuration.TimestepContactMargin:G5}");
        Metric("Soft-avoidance shell", configuration.SoftAvoidanceShell.ToString("G5"));
        Metric("Timestep cache", Bool(configuration.TimestepCacheEnabled));
        Metric("Cross-frame topology", Bool(configuration.CrossFrameTopologyEnabled));
        Metric("Predictive contacts", Bool(configuration.PredictiveContactsEnabled));
        Metric("Diagnostics", Bool(configuration.DiagnosticsEnabled));

        Space();
        EditorGUILayout.LabelField("General solver", EditorStyles.boldLabel);
        Metric("Solver total", NsToMs(solver.SolverNanoseconds));
        Metric("Pair generation", NsToMs(solver.PairGenerationNanoseconds));
        Metric("Contact-set build", NsToMs(solver.TimestepContactSetBuildNanoseconds));
        Metric("Iterations total / average",
            $"{NsToMs(solver.IterationNanoseconds)} / {NsToMs(solver.AverageIterationNanoseconds)}");
        Metric("Soft avoidance total / average",
            $"{NsToMs(solver.SoftAvoidanceNanoseconds)} / {NsToMs(solver.AverageSoftAvoidanceNanoseconds)}");
        Metric("Contact / predictive / active",
            $"{solver.ContactPairCount} / {solver.PredictivePairCount} / {solver.ActiveConstraintCount}");
        Metric("Penetrating pairs", solver.PenetratingPairCount.ToString());
        Metric("Average penetration", solver.AveragePenetration.ToString("G6"));
        Metric("Contact correction total / max",
            $"{solver.TotalContactPositionCorrection:G6} / {solver.MaxContactPositionCorrection:G6}");
        Metric("Wall correction total / max",
            $"{solver.TotalWallPositionCorrection:G6} / {solver.MaxWallPositionCorrection:G6}");
        Metric("Speed before / after",
            $"{solver.AverageSpeedBeforeContact:G6} / {solver.AverageSpeedAfterContact:G6}");
        Metric("Velocity delta total / max",
            $"{solver.TotalVelocityChange:G6} / {solver.MaxVelocityChange:G6}");

        Space();
        EditorGUILayout.LabelField("Persistent topology", EditorStyles.boldLabel);
        Metric("Proxies", statistics.ProxyCount.ToString());
        Metric("Clean ratio", snapshot.CleanProxyRatio.ToString("P2"));
        Metric("Topology / motion dirty",
            $"{statistics.TopologyDirtyBodyCount} / {statistics.MotionDirtyBodyCount}");
        Metric("Corrected escapes", statistics.CorrectedEscapeBodyCount.ToString());
        Metric("Local proxy queries", statistics.LocalProxyQueryCount.ToString());
        Metric("Persistent neighbor pairs", statistics.PersistentNeighborPairCount.ToString());
        Metric("Added / removed / retained",
            $"{statistics.NeighborPairAddedCount} / {statistics.NeighborPairRemovedCount} / {statistics.NeighborPairRetainedCount}");
        Metric("Retained ratio", snapshot.RetainedNeighborPairRatio.ToString("P2"));
        Metric("Full rebuilds / repairs",
            $"{statistics.FullRebuildCount} / {statistics.IncrementalRepairCount}");

        Space();
        EditorGUILayout.LabelField("Predictive lifecycle", EditorStyles.boldLabel);
        Metric("Reclassified evaluations", statistics.ReclassifiedPairEvaluationCount.ToString());
        Metric("Swept evaluations", statistics.SweptClassificationEvaluationCount.ToString());
        Metric("Constraint evaluations", statistics.ActiveConstraintEvaluationCount.ToString());
        Metric("Current swept", statistics.CurrentSweptContactCount.ToString());
        Metric("Dormant / approaching",
            $"{statistics.CurrentDormantPairCount} / {statistics.CurrentApproachingPairCount}");
        Metric("Predictive / actual",
            $"{statistics.CurrentPredictivePairCount} / {statistics.CurrentActualPairCount}");
        Metric("Current / peak active",
            $"{statistics.CurrentActiveConstraintCount} / {statistics.PeakActiveConstraintCount}");
        Metric("Scheduled wakeups", statistics.ScheduledWakeupCount.ToString());
        Metric("Unique activated / corrected",
            $"{statistics.UniqueActivatedPairCount} / {statistics.UniqueCorrectedPairCount}");
        Metric("Expired", statistics.ExpiredPairCount.ToString());
        Metric("Neighbor → swept", snapshot.NeighborToSweptRatio.ToString("P2"));
        Metric("Swept → current active", snapshot.SweptToCurrentActiveRatio.ToString("P2"));
        Metric("Activated → corrected", snapshot.ActivatedToCorrectedRatio.ToString("P2"));

        Space();
        EditorGUILayout.LabelField("Incremental timings", EditorStyles.boldLabel);
        Metric("Proxy validation", NsToMs(statistics.ProxyValidationNanoseconds));
        Metric("Local broadphase", NsToMs(statistics.LocalBroadPhaseNanoseconds));
        Metric("Pair diff / merge", NsToMs(statistics.PairDiffNanoseconds));
        Metric("Swept classification", NsToMs(statistics.SweptClassificationNanoseconds));
        Metric("Contact activation", NsToMs(statistics.ContactActivationNanoseconds));
        Metric("Fallback", NsToMs(statistics.FallbackNanoseconds));

        Space();
        EditorGUILayout.LabelField("Correctness", EditorStyles.boldLabel);
        Metric("Oracle health", snapshot.OracleHealthy != 0 ? "OK" : "MISMATCH");
        Metric("Oracle pair / missing / extra",
            $"{statistics.OraclePairCount} / {statistics.OracleMissingPairCount} / {statistics.OracleExtraPairCount}");

        _showLegacy = EditorGUILayout.Foldout(_showLegacy,
            "Legacy Fat/Adaptive compatibility counters", true);
        if (_showLegacy)
        {
            var legacy = snapshot.LegacyBroadPhaseStatistics;
            EditorGUI.indentLevel++;
            Metric("Cache use / reuse / rebuild",
                $"{legacy.CacheUseCount} / {legacy.CacheReuseCount} / {legacy.CacheRebuildCount}");
            Metric("Full broadphase fallback", legacy.FullBroadPhaseFallbackCount.ToString());
            EditorGUILayout.HelpBox(
                "These fields remain for serialized-scene and historical CSV compatibility. " +
                "They no longer describe the primary incremental execution path.",
                MessageType.Info);
            EditorGUI.indentLevel--;
        }
    }

    private static void DrawRecording()
    {
        EditorGUILayout.LabelField("CSV v3 recorder", EditorStyles.boldLabel);
        Metric("State", IncrementalContactPipelineCsvRecorderRuntime.IsRecording ? "Recording" : "Idle");
        Metric("Progress",
            $"{IncrementalContactPipelineCsvRecorderRuntime.RecordedFrames}/{IncrementalContactPipelineCsvRecorderRuntime.TargetFrames}");
        Metric("Raw CSV", IncrementalContactPipelineCsvRecorderRuntime.RawPath ?? string.Empty);
        Metric("Summary CSV", IncrementalContactPipelineCsvRecorderRuntime.SummaryPath ?? string.Empty);
        EditorGUILayout.Space();
        if (GUILayout.Button("Open benchmark tuner"))
            IncrementalContactPipelineBenchmarkWindow.Open();
    }

    private static void FunnelRow(string label, int count, float? ratio)
    {
        Metric(label, ratio.HasValue ? $"{count} ({ratio.Value:P1})" : count.ToString());
    }

    private static void Metric(string label, string value)
    {
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField(label, GUILayout.Width(220f));
        EditorGUILayout.SelectableLabel(value ?? string.Empty,
            EditorStyles.label, GUILayout.Height(EditorGUIUtility.singleLineHeight));
        EditorGUILayout.EndHorizontal();
    }

    private static string Bool(byte value) => value != 0 ? "Enabled" : "Disabled";

    private static string NsToMs(long nanoseconds)
    {
        return (nanoseconds / 1_000_000d).ToString("F4") + " ms";
    }

    private static void Space()
    {
        EditorGUILayout.Space(8f);
    }
}
}
#endif
