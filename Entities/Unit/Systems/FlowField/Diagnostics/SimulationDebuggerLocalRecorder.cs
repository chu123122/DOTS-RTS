using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// 轻量本地记录器：只采样四面板的摘要指标，不保存热力图、代理或 Pair 明细。
/// F6 手动开始/停止；F7 记录固定 10 秒。每个运行创建四个 CSV 文件。
/// </summary>
[DefaultExecutionOrder(1000)]
public sealed class SimulationDebuggerLocalRecorder : MonoBehaviour
{
    private const float DefaultSampleIntervalSeconds = 0.5f;
    private const float AutomaticDurationSeconds = 10f;
    private const int DefaultMaxSamplesPerManualRun = 3600;
    private const int FlushEverySamples = 16;

    public static SimulationDebuggerLocalRecorder Instance { get; private set; }

    [Header("本地记录")]
    [Min(0.1f)] public float SampleIntervalSeconds = DefaultSampleIntervalSeconds;
    [Min(1)] public int MaxSamplesPerManualRun = DefaultMaxSamplesPerManualRun;

    public bool IsRecording => _isRecording;
    public bool IsAutomaticRun => _automaticRun;
    public int SampleCount => _sampleCount;
    public string OutputDirectory => _outputDirectory ?? string.Empty;
    public float ElapsedSeconds => _isRecording
        ? Mathf.Max(0f, Time.unscaledTime - _startedAtUnscaledTime)
        : 0f;

    private StreamWriter _overviewWriter;
    private StreamWriter _topologyWriter;
    private StreamWriter _contactWriter;
    private StreamWriter _settingsWriter;
    private bool _isRecording;
    private bool _automaticRun;
    private int _sampleCount;
    private float _startedAtUnscaledTime;
    private float _nextSampleAtUnscaledTime;
    private ulong _lastRecordedFrameId;
    private string _outputDirectory;
    private string _lastSettingsKey;

    private void OnEnable()
    {
        Instance = this;
    }

    private void OnDisable()
    {
        if (_isRecording)
            StopRecording("组件已禁用，已安全写入");
        if (Instance == this)
            Instance = null;
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F6))
        {
            if (_isRecording)
                StopRecording("F6 手动停止");
            else
                StartRecording(false);
        }

        if (Input.GetKeyDown(KeyCode.F7) && !_isRecording)
            StartRecording(true);

        if (!_isRecording)
            return;

        if (_automaticRun && ElapsedSeconds >= AutomaticDurationSeconds)
        {
            StopRecording("F7 固定 10 秒记录完成");
            return;
        }

        if (Time.unscaledTime < _nextSampleAtUnscaledTime ||
            !SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot) ||
            snapshot.FrameId == _lastRecordedFrameId)
            return;

        WriteSample(snapshot);
        _lastRecordedFrameId = snapshot.FrameId;
        _nextSampleAtUnscaledTime = Time.unscaledTime + Mathf.Max(0.1f, SampleIntervalSeconds);

        if (!_automaticRun && _sampleCount >= Mathf.Max(1, MaxSamplesPerManualRun))
            StopRecording($"达到手动记录上限 {MaxSamplesPerManualRun} 条");
    }

    private void StartRecording(bool automatic)
    {
        string runId = $"{DateTime.Now:yyyyMMdd-HHmmss-fff}-{(automatic ? "auto10s" : "manual")}";
        _outputDirectory = Path.Combine(
            Application.persistentDataPath,
            "RTSLocalDiagnostics",
            runId);
        Directory.CreateDirectory(_outputDirectory);

        try
        {
            _overviewWriter = CreateWriter("01_overview.csv",
                "elapsed_s,frame,units,solver_us,soft_avoidance_us,pair_generation_us,iteration_us,other_us,candidate_pairs,contact_pairs,max_contact_correction,max_wall_correction,max_velocity_change,substeps,iterations");
            _topologyWriter = CreateWriter("02_incremental_topology.csv",
                "elapsed_s,frame,timestep,mode,proxy_count,topology_dirty_bodies,motion_dirty_bodies,escaped_bodies,local_proxy_queries,persistent_neighbor_pairs,pairs_added,pairs_removed,pairs_retained,full_rebuilds,incremental_repairs,clean_proxy_ratio,retained_pair_ratio,proxy_validation_us,local_broadphase_us,pair_diff_us,oracle_missing_pairs,oracle_extra_pairs");
            _contactWriter = CreateWriter("03_timestep_contact_set.csv",
                "elapsed_s,frame,cache_enabled,builds,contact_set_size,active_contacts,inactive_contacts,actual_contacts,predictive_contacts,predictive_activated,fallbacks,activation_ratio,predictive_activation_ratio,substeps");
            _settingsWriter = CreateWriter("04_settings.csv",
                "elapsed_s,frame,configuration_id,adaptive_hotspot_enabled,legacy_fat_aabb_enabled,timestep_contact_set_enabled,predictive_skin,fat_margin,substeps,iterations,soft_avoidance_solver,soft_avoidance_rate,soft_avoidance_shell,rvo_horizon");
        }
        catch (Exception exception)
        {
            CloseWriters();
            Debug.LogError($"[RTS Local Recorder] 无法创建记录文件：{exception.Message}");
            return;
        }

        _isRecording = true;
        _automaticRun = automatic;
        _sampleCount = 0;
        _lastRecordedFrameId = 0;
        _lastSettingsKey = null;
        _startedAtUnscaledTime = Time.unscaledTime;
        _nextSampleAtUnscaledTime = _startedAtUnscaledTime;
        SimulationDebuggerRuntime.SetLocalRecordingCapture(true);
        Debug.Log($"[RTS Local Recorder] {(automatic ? "F7 自动 10 秒" : "F6 手动")}记录开始：{_outputDirectory}");
    }

    private void StopRecording(string reason)
    {
        if (!_isRecording)
            return;

        _isRecording = false;
        SimulationDebuggerRuntime.SetLocalRecordingCapture(false);
        CloseWriters();
        Debug.Log($"[RTS Local Recorder] {reason}；已写入 {_sampleCount} 条采样：{_outputDirectory}");
    }

    private StreamWriter CreateWriter(string fileName, string header)
    {
        var writer = new StreamWriter(
            Path.Combine(_outputDirectory, fileName),
            false,
            new UTF8Encoding(false));
        writer.WriteLine(header);
        return writer;
    }

    private void WriteSample(SimulationDebuggerFrameSnapshot snapshot)
    {
        float elapsed = ElapsedSeconds;
        SimulationOverviewMetrics overview = snapshot.Overview;
        IncrementalContactPipelineSnapshot incremental =
            IncrementalContactPipelineDiagnosticsRuntime.Latest;
        TimestepContactSetMetrics contactSet = snapshot.ContactSet;
        SimulationDebuggerEffectiveSettings settings = snapshot.EffectiveSettings;

        long known = overview.SoftAvoidanceNanoseconds +
                     overview.PairGenerationNanoseconds +
                     overview.IterationNanoseconds;
        long other = Math.Max(0, overview.SolverNanoseconds - known);

        _overviewWriter.WriteLine(string.Join(",",
            Number(elapsed), snapshot.FrameId, overview.UnitCount,
            Microseconds(overview.SolverNanoseconds), Microseconds(overview.SoftAvoidanceNanoseconds),
            Microseconds(overview.PairGenerationNanoseconds), Microseconds(overview.IterationNanoseconds),
            Microseconds(other), overview.CandidatePairCount, overview.ContactPairCount,
            Number(overview.MaxContactCorrection), Number(overview.MaxWallCorrection),
            Number(overview.MaxVelocityChange), snapshot.SubstepCount, snapshot.IterationCount));

        IncrementalContactPipelineStatistics topology = incremental.Statistics;
        _topologyWriter.WriteLine(string.Join(",",
            Number(elapsed), snapshot.FrameId, topology.Timestep, incremental.Mode,
            topology.ProxyCount, topology.TopologyDirtyBodyCount, topology.MotionDirtyBodyCount,
            topology.CorrectedEscapeBodyCount, topology.LocalProxyQueryCount,
            topology.PersistentNeighborPairCount, topology.NeighborPairAddedCount,
            topology.NeighborPairRemovedCount, topology.NeighborPairRetainedCount,
            topology.FullRebuildCount, topology.IncrementalRepairCount,
            Number(incremental.CleanProxyRatio), Number(incremental.RetainedNeighborPairRatio),
            Microseconds(topology.ProxyValidationNanoseconds),
            Microseconds(topology.LocalBroadPhaseNanoseconds),
            Microseconds(topology.PairDiffNanoseconds), topology.OracleMissingPairCount,
            topology.OracleExtraPairCount));

        _contactWriter.WriteLine(string.Join(",",
            Number(elapsed), snapshot.FrameId, contactSet.CacheEnabled,
            contactSet.ContactGenerationCount, contactSet.ContactSetSize,
            contactSet.ActiveContactCount, contactSet.InactiveContactCount,
            contactSet.ActualContactCount, contactSet.PredictiveContactCount,
            contactSet.PredictiveActivatedCount, contactSet.SupplementOrFallbackCount,
            Number(contactSet.ActivationRatio), Number(contactSet.PredictiveActivationRatio),
            contactSet.SubstepCount));

        string settingsKey = string.Join("|",
            snapshot.Experiment.ConfigurationId, settings.EnableAdaptiveFatAabb,
            settings.EnableFatAabbCache, settings.EnableTimestepContactSetCache,
            settings.PredictiveSkin, settings.FatAabbCacheMargin, settings.SubstepCount,
            settings.IterationCount, settings.SoftAvoidanceVelocitySolver,
            settings.SoftAvoidanceResponseRate, settings.SoftAvoidanceShell,
            settings.RvoTimeHorizon);
        if (_lastSettingsKey != settingsKey)
        {
            _settingsWriter.WriteLine(string.Join(",",
                Number(elapsed), snapshot.FrameId, snapshot.Experiment.ConfigurationId,
                settings.EnableAdaptiveFatAabb, settings.EnableFatAabbCache,
                settings.EnableTimestepContactSetCache, Number(settings.PredictiveSkin),
                Number(settings.FatAabbCacheMargin), settings.SubstepCount, settings.IterationCount,
                settings.SoftAvoidanceVelocitySolver, Number(settings.SoftAvoidanceResponseRate),
                Number(settings.SoftAvoidanceShell), Number(settings.RvoTimeHorizon)));
            _lastSettingsKey = settingsKey;
        }

        _sampleCount++;
        if (_sampleCount % FlushEverySamples == 0)
            FlushWriters();
    }

    private void CloseWriters()
    {
        FlushWriters();
        _overviewWriter?.Dispose();
        _topologyWriter?.Dispose();
        _contactWriter?.Dispose();
        _settingsWriter?.Dispose();
        _overviewWriter = null;
        _topologyWriter = null;
        _contactWriter = null;
        _settingsWriter = null;
    }

    private void FlushWriters()
    {
        _overviewWriter?.Flush();
        _topologyWriter?.Flush();
        _contactWriter?.Flush();
        _settingsWriter?.Flush();
    }

    private static string Number(float value) => value.ToString("0.######", CultureInfo.InvariantCulture);
    private static string Microseconds(long nanoseconds) =>
        (nanoseconds / 1000f).ToString("0.###", CultureInfo.InvariantCulture);
}
}
