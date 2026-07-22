using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Versioned per-timestep CSV recorder for the migrated contact pipeline.
/// It records raw samples and writes a separate percentile summary on stop.
/// Recording is opt-in and diagnostics-only.
/// </summary>
public static class IncrementalContactPipelineCsvRecorderRuntime
{
    public const int CsvSchemaVersion = 5;

    private static readonly List<IncrementalContactPipelineSnapshot> Samples =
        new List<IncrementalContactPipelineSnapshot>(1024);

    private static StreamWriter _writer;
    private static string _rawPath;
    private static string _summaryPath;
    private static int _warmupFrames;
    private static int _measuredFrames;
    private static int _observedFrames;
    private static int _recordedFrames;
    private static uint _lastTimestep;

    public static bool IsRecording { get; private set; }
    public static int RecordedFrames => _recordedFrames;
    public static int TargetFrames => _measuredFrames;
    public static string RawPath => _rawPath;
    public static string SummaryPath => _summaryPath;

    public static event Action<string, string> SessionCompleted;

    public static void Start(string rawPath, int warmupFrames, int measuredFrames)
    {
        Stop(writeSummary: false);

        if (string.IsNullOrWhiteSpace(rawPath))
            throw new ArgumentException("CSV path must not be empty.", nameof(rawPath));

        _rawPath = Path.GetFullPath(rawPath);
        _summaryPath = BuildSummaryPath(_rawPath);
        _warmupFrames = Math.Max(0, warmupFrames);
        _measuredFrames = Math.Max(1, measuredFrames);
        _observedFrames = 0;
        _recordedFrames = 0;
        _lastTimestep = 0;
        Samples.Clear();

        string directory = Path.GetDirectoryName(_rawPath);
        if (!string.IsNullOrEmpty(directory))
            Directory.CreateDirectory(directory);

        _writer = new StreamWriter(_rawPath, false, new UTF8Encoding(false));
        _writer.WriteLine(BuildHeader());
        _writer.Flush();
        IsRecording = true;
    }

    public static void Stop(bool writeSummary = true)
    {
        if (!IsRecording && _writer == null)
            return;

        IsRecording = false;
        _writer?.Flush();
        _writer?.Dispose();
        _writer = null;

        if (writeSummary && Samples.Count > 0 && !string.IsNullOrEmpty(_summaryPath))
            WriteSummary(_summaryPath, Samples);

        string raw = _rawPath;
        string summary = _summaryPath;
        SessionCompleted?.Invoke(raw, summary);
    }

    public static void TryRecord(IncrementalContactPipelineSnapshot snapshot)
    {
        if (!IsRecording || snapshot.Statistics.Timestep == 0)
            return;
        if (snapshot.Statistics.Timestep == _lastTimestep)
            return;

        _lastTimestep = snapshot.Statistics.Timestep;
        _observedFrames++;
        if (_observedFrames <= _warmupFrames)
            return;

        Samples.Add(snapshot);
        _writer.WriteLine(BuildRow(snapshot, _recordedFrames));
        _recordedFrames++;

        if ((_recordedFrames % 60) == 0)
            _writer.Flush();
        if (_recordedFrames >= _measuredFrames)
            Stop(writeSummary: true);
    }

    private static string BuildHeader()
    {
        return string.Join(",", new[]
        {
            "CsvSchemaVersion", "SnapshotSchemaVersion", "UtcTimestamp", "SampleIndex",
            "ExperimentId", "Scenario", "ConfigurationLabel", "PipelineMode", "Timestep",
            "UnitCount", "DeltaTime", "SubstepCount", "IterationCount",
            "TimestepCacheEnabled", "CrossFrameTopologyEnabled", "PredictiveContactsEnabled", "DiagnosticsEnabled",
            "GuardEnvelopeMargin", "PredictiveSkin", "TimestepContactMargin", "SoftAvoidanceShell",

            "SolverNanoseconds", "PairGenerationNanoseconds", "TimestepContactSetBuildNanoseconds",
            "IterationNanoseconds", "AverageIterationNanoseconds", "SoftAvoidanceNanoseconds",
            "AverageSoftAvoidanceNanoseconds", "ContactPairCount", "PredictivePairCount",
            "ActiveConstraintCount", "PenetratingPairCount", "AveragePenetration",
            "TotalContactPositionCorrection", "MaxContactPositionCorrection",
            "TotalWallPositionCorrection", "MaxWallPositionCorrection",
            "AverageSpeedBeforeContact", "AverageSpeedAfterContact",
            "TotalVelocityChange", "MaxVelocityChange",

            "ProxyCount", "TopologyDirtyBodyCount", "MotionDirtyBodyCount",
            "CorrectedEscapeBodyCount", "LocalProxyQueryCount", "PersistentNeighborPairCount",
            "NeighborPairAddedCount", "NeighborPairRemovedCount", "NeighborPairRetainedCount",
            "FullRebuildCount", "IncrementalRepairCount", "UsedIncrementalTopology", "UsedFullRebuild",

            "ReclassifiedPairEvaluationCount", "ClassificationReuseCount", "ClassificationSkippedCount",
            "SweptClassificationEvaluationCount", "SoftAvoidancePairEvaluationCount",
            "ActiveConstraintEvaluationCount", "CurrentInteractionPairCount", "CurrentSoftAvoidancePairCount",
            "CurrentSweptContactCount", "CurrentDormantPairCount",
            "CurrentApproachingPairCount", "CurrentPredictivePairCount", "CurrentActualPairCount",
            "CurrentActiveConstraintCount", "PeakActiveConstraintCount", "ScheduledWakeupCount",
            "UniqueActivatedPairCount", "UniqueCorrectedPairCount", "ExpiredPairCount",

            "TopologyDirtyRatio", "CleanProxyRatio", "RetainedNeighborPairRatio",
            "NeighborToSweptRatio", "SweptToCurrentActiveRatio", "ActivatedToCorrectedRatio",

            "FullSweepSourceNanoseconds", "PersistentPairMappingNanoseconds",
            "ProxyValidationNanoseconds", "LocalBroadPhaseNanoseconds", "PairDiffNanoseconds",
            "SweptClassificationNanoseconds", "ContactActivationNanoseconds", "FallbackNanoseconds",

            "PersistentViewReuseCount", "PersistentViewRebuildCount", "InteractionEnvelopeEscapeCount",
            "OraclePairCount", "OracleMissingPairCount", "OracleExtraPairCount", "OracleMismatch",
            "SoftAvoidanceOraclePairCount", "SoftAvoidanceOracleMissingPairCount",
            "LegacyCacheUseCount", "LegacyCacheReuseCount", "LegacyCacheRebuildCount",
            "LegacyFullBroadPhaseFallbackCount"
        });
    }

    private static string BuildRow(IncrementalContactPipelineSnapshot snapshot, int sampleIndex)
    {
        IncrementalContactPipelineConfiguration configuration = snapshot.Configuration;
        var solver = snapshot.SolverStatistics;
        var statistics = snapshot.Statistics;
        var legacy = snapshot.LegacyBroadPhaseStatistics;
        var values = new List<string>(100)
        {
            CsvSchemaVersion.ToString(CultureInfo.InvariantCulture),
            snapshot.SchemaVersion.ToString(CultureInfo.InvariantCulture),
            DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            sampleIndex.ToString(CultureInfo.InvariantCulture),
            Escape(configuration.ExperimentId.ToString()),
            Escape(configuration.Scenario.ToString()),
            Escape(configuration.ConfigurationLabel.ToString()),
            snapshot.Mode.ToString(),
            statistics.Timestep.ToString(CultureInfo.InvariantCulture),
            configuration.UnitCount.ToString(CultureInfo.InvariantCulture),
            F(configuration.DeltaTime),
            configuration.SubstepCount.ToString(CultureInfo.InvariantCulture),
            configuration.IterationCount.ToString(CultureInfo.InvariantCulture),
            configuration.TimestepCacheEnabled.ToString(CultureInfo.InvariantCulture),
            configuration.CrossFrameTopologyEnabled.ToString(CultureInfo.InvariantCulture),
            configuration.PredictiveContactsEnabled.ToString(CultureInfo.InvariantCulture),
            configuration.DiagnosticsEnabled.ToString(CultureInfo.InvariantCulture),
            F(configuration.GuardEnvelopeMargin),
            F(configuration.PredictiveSkin),
            F(configuration.TimestepContactMargin),
            F(configuration.SoftAvoidanceShell),

            solver.SolverNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.PairGenerationNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.TimestepContactSetBuildNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.IterationNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.AverageIterationNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.SoftAvoidanceNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.AverageSoftAvoidanceNanoseconds.ToString(CultureInfo.InvariantCulture),
            solver.ContactPairCount.ToString(CultureInfo.InvariantCulture),
            solver.PredictivePairCount.ToString(CultureInfo.InvariantCulture),
            solver.ActiveConstraintCount.ToString(CultureInfo.InvariantCulture),
            solver.PenetratingPairCount.ToString(CultureInfo.InvariantCulture),
            F(solver.AveragePenetration),
            F(solver.TotalContactPositionCorrection),
            F(solver.MaxContactPositionCorrection),
            F(solver.TotalWallPositionCorrection),
            F(solver.MaxWallPositionCorrection),
            F(solver.AverageSpeedBeforeContact),
            F(solver.AverageSpeedAfterContact),
            F(solver.TotalVelocityChange),
            F(solver.MaxVelocityChange),

            statistics.ProxyCount.ToString(CultureInfo.InvariantCulture),
            statistics.TopologyDirtyBodyCount.ToString(CultureInfo.InvariantCulture),
            statistics.MotionDirtyBodyCount.ToString(CultureInfo.InvariantCulture),
            statistics.CorrectedEscapeBodyCount.ToString(CultureInfo.InvariantCulture),
            statistics.LocalProxyQueryCount.ToString(CultureInfo.InvariantCulture),
            statistics.PersistentNeighborPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.NeighborPairAddedCount.ToString(CultureInfo.InvariantCulture),
            statistics.NeighborPairRemovedCount.ToString(CultureInfo.InvariantCulture),
            statistics.NeighborPairRetainedCount.ToString(CultureInfo.InvariantCulture),
            statistics.FullRebuildCount.ToString(CultureInfo.InvariantCulture),
            statistics.IncrementalRepairCount.ToString(CultureInfo.InvariantCulture),
            statistics.UsedIncrementalTopology.ToString(CultureInfo.InvariantCulture),
            statistics.UsedFullRebuild.ToString(CultureInfo.InvariantCulture),

            statistics.ReclassifiedPairEvaluationCount.ToString(CultureInfo.InvariantCulture),
            statistics.ClassificationReuseCount.ToString(CultureInfo.InvariantCulture),
            statistics.ClassificationSkippedCount.ToString(CultureInfo.InvariantCulture),
            statistics.SweptClassificationEvaluationCount.ToString(CultureInfo.InvariantCulture),
            statistics.SoftAvoidancePairEvaluationCount.ToString(CultureInfo.InvariantCulture),
            statistics.ActiveConstraintEvaluationCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentInteractionPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentSoftAvoidancePairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentSweptContactCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentDormantPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentApproachingPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentPredictivePairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentActualPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.CurrentActiveConstraintCount.ToString(CultureInfo.InvariantCulture),
            statistics.PeakActiveConstraintCount.ToString(CultureInfo.InvariantCulture),
            statistics.ScheduledWakeupCount.ToString(CultureInfo.InvariantCulture),
            statistics.UniqueActivatedPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.UniqueCorrectedPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.ExpiredPairCount.ToString(CultureInfo.InvariantCulture),

            F(snapshot.TopologyDirtyRatio), F(snapshot.CleanProxyRatio),
            F(snapshot.RetainedNeighborPairRatio), F(snapshot.NeighborToSweptRatio),
            F(snapshot.SweptToCurrentActiveRatio), F(snapshot.ActivatedToCorrectedRatio),

            statistics.FullSweepSourceNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.PersistentPairMappingNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.ProxyValidationNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.LocalBroadPhaseNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.PairDiffNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.SweptClassificationNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.ContactActivationNanoseconds.ToString(CultureInfo.InvariantCulture),
            statistics.FallbackNanoseconds.ToString(CultureInfo.InvariantCulture),

            statistics.PersistentViewReuseCount.ToString(CultureInfo.InvariantCulture),
            statistics.PersistentViewRebuildCount.ToString(CultureInfo.InvariantCulture),
            statistics.InteractionEnvelopeEscapeCount.ToString(CultureInfo.InvariantCulture),
            statistics.OraclePairCount.ToString(CultureInfo.InvariantCulture),
            statistics.OracleMissingPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.OracleExtraPairCount.ToString(CultureInfo.InvariantCulture),
            statistics.OracleMismatch.ToString(CultureInfo.InvariantCulture),
            statistics.SoftAvoidanceOraclePairCount.ToString(CultureInfo.InvariantCulture),
            statistics.SoftAvoidanceOracleMissingPairCount.ToString(CultureInfo.InvariantCulture),
            legacy.CacheUseCount.ToString(CultureInfo.InvariantCulture),
            legacy.CacheReuseCount.ToString(CultureInfo.InvariantCulture),
            legacy.CacheRebuildCount.ToString(CultureInfo.InvariantCulture),
            legacy.FullBroadPhaseFallbackCount.ToString(CultureInfo.InvariantCulture)
        };
        return string.Join(",", values);
    }

    private static void WriteSummary(
        string path,
        IReadOnlyList<IncrementalContactPipelineSnapshot> samples)
    {
        IncrementalContactPipelineSnapshot last = samples[samples.Count - 1];
        var solverNs = samples.Select(s => s.SolverStatistics.SolverNanoseconds).ToArray();
        var softNs = samples.Select(s => s.SolverStatistics.SoftAvoidanceNanoseconds).ToArray();
        var topologyNs = samples.Select(s =>
            s.Statistics.ProxyValidationNanoseconds +
            s.Statistics.PersistentPairMappingNanoseconds +
            s.Statistics.LocalBroadPhaseNanoseconds +
            s.Statistics.PairDiffNanoseconds +
            s.Statistics.FallbackNanoseconds).ToArray();
        var predictiveNs = samples.Select(s =>
            s.Statistics.SweptClassificationNanoseconds +
            s.Statistics.ContactActivationNanoseconds).ToArray();

        using (var writer = new StreamWriter(path, false, new UTF8Encoding(false)))
        {
            writer.WriteLine(
                "CsvSchemaVersion,ExperimentId,Scenario,ConfigurationLabel,SampleCount," +
                "Metric,Average,P50,P95,P99,Maximum,Unit");
            WriteSummaryMetric(writer, last, samples.Count, "Solver", solverNs, "ns");
            WriteSummaryMetric(writer, last, samples.Count, "SoftAvoidance", softNs, "ns");
            WriteSummaryMetric(writer, last, samples.Count, "A0FullSweepSource",
                samples.Select(s => s.Statistics.FullSweepSourceNanoseconds).ToArray(), "ns");
            WriteSummaryMetric(writer, last, samples.Count, "TopologyUpdate", topologyNs, "ns");
            WriteSummaryMetric(writer, last, samples.Count, "PredictiveContact", predictiveNs, "ns");
            WriteSummaryMetric(writer, last, samples.Count, "PersistentNeighborPairs",
                samples.Select(s => (long)s.Statistics.PersistentNeighborPairCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "CurrentActiveConstraints",
                samples.Select(s => (long)s.Statistics.CurrentActiveConstraintCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "TimestepInteractionPairs",
                samples.Select(s => (long)s.Statistics.CurrentInteractionPairCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "SoftAvoidancePairs",
                samples.Select(s => (long)s.Statistics.CurrentSoftAvoidancePairCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "ClassificationEvaluations",
                samples.Select(s => (long)s.Statistics.SweptClassificationEvaluationCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "ClassificationSkipped",
                samples.Select(s => (long)s.Statistics.ClassificationSkippedCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "PersistentViewReuse",
                samples.Select(s => (long)s.Statistics.PersistentViewReuseCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "SoftCandidateEvaluations",
                samples.Select(s => (long)s.SolverStatistics.SoftAvoidanceCandidatePairCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "ConstraintEvaluations",
                samples.Select(s => (long)s.Statistics.ActiveConstraintEvaluationCount).ToArray(), "count");
            WriteSummaryMetric(writer, last, samples.Count, "TopologyDirtyRatioPpm",
                samples.Select(s => (long)Math.Round(s.TopologyDirtyRatio * 1_000_000d)).ToArray(), "ppm");
            WriteSummaryMetric(writer, last, samples.Count, "OracleMissingPairs",
                samples.Select(s => (long)s.Statistics.OracleMissingPairCount).ToArray(), "count");
        }
    }

    private static void WriteSummaryMetric(
        StreamWriter writer,
        IncrementalContactPipelineSnapshot last,
        int sampleCount,
        string metric,
        long[] values,
        string unit)
    {
        Array.Sort(values);
        double average = values.Length == 0 ? 0d : values.Average(v => (double)v);
        writer.WriteLine(string.Join(",", new[]
        {
            CsvSchemaVersion.ToString(CultureInfo.InvariantCulture),
            Escape(last.Configuration.ExperimentId.ToString()),
            Escape(last.Configuration.Scenario.ToString()),
            Escape(last.Configuration.ConfigurationLabel.ToString()),
            sampleCount.ToString(CultureInfo.InvariantCulture),
            metric,
            average.ToString("R", CultureInfo.InvariantCulture),
            Percentile(values, 0.50).ToString(CultureInfo.InvariantCulture),
            Percentile(values, 0.95).ToString(CultureInfo.InvariantCulture),
            Percentile(values, 0.99).ToString(CultureInfo.InvariantCulture),
            (values.Length == 0 ? 0 : values[values.Length - 1]).ToString(CultureInfo.InvariantCulture),
            unit
        }));
    }

    private static long Percentile(long[] sortedValues, double percentile)
    {
        if (sortedValues.Length == 0)
            return 0;
        int index = (int)Math.Ceiling(percentile * sortedValues.Length) - 1;
        index = Math.Max(0, Math.Min(index, sortedValues.Length - 1));
        return sortedValues[index];
    }

    private static string BuildSummaryPath(string rawPath)
    {
        string directory = Path.GetDirectoryName(rawPath) ?? string.Empty;
        string fileName = Path.GetFileNameWithoutExtension(rawPath);
        return Path.Combine(directory, fileName + "_summary.csv");
    }

    private static string F(float value)
    {
        return value.ToString("R", CultureInfo.InvariantCulture);
    }

    private static string Escape(string value)
    {
        if (string.IsNullOrEmpty(value))
            return string.Empty;
        if (value.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0)
            return value;
        return '"' + value.Replace("\"", "\"\"") + '"';
    }
}
}
