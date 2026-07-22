using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.SceneManagement;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// 全自动自适应 Fat AABB 参数调优器。
///
/// 工作流程：
///   1. 等 RtsLocalPrefabs 加载完成
///   2. 在聚集点批量生成 UnitCount 个单位
///   3. 等 WarmupFrames 帧让求解收敛
///   4. 按 TrialList 依次切换参数，每组跑 TrialFrames 帧
///   5. 采集末尾 StatisticsFrames 帧的求解统计
///   6. 退出 Play Mode 前输出 CSV 到项目根目录
///
/// 不需要任何手动操作——场景进入 Play Mode 后全自动完成。
/// </summary>
public sealed class AdaptiveParameterTuner : MonoBehaviour
{
    [Header("Spawn")]
    [Min(1)] public int UnitCount = 200;
    public float3 ClusterCenter = new(0f, 0f, 0f);
    public float SpawnSpread = 2f;

    [Header("Timing")]
    [Min(0)] public float PostSpawnDelaySeconds = 5f;
    [Min(1)] public int WarmupFrames = 360;
    [Min(1)] public int TrialFrames = 600;
    [Min(1)] public int StatisticsFrames = 300;

    [Header("Scenario")]
    [Tooltip("默认关闭：本轮用于静止密集单位测试。开启后才会在预热前下发随机移动命令。")]
    public bool IssueMoveBeforeTrials;
    public float MoveTargetSpread = 10f;

    [Header("Trials — 跨帧缓存 A/B")]
    public List<ParameterTrial> TrialList = new()
    {
        // 保持跨子步接触集(B)一致，只切换跨帧持久邻居拓扑(A)。
        // 详细诊断含 O(N²) Oracle，性能测量必须关闭。
        new()
        {
            Label = "baseline_timestep_swept_A0_B1",
            EnableFatAabbCache = 0,
            EnableTimestepContactSetCache = 1,
            EnableAdaptiveFatAabb = 0,
            EnableDiagnostics = 0,
            FatAabbCacheMargin = 0.5f,
            SubstepCount = 4,
            IterationCount = 4
        },
        new()
        {
            Label = "incremental_persistent_A1_B1",
            EnableFatAabbCache = 1,
            EnableTimestepContactSetCache = 1,
            EnableAdaptiveFatAabb = 0,
            EnableDiagnostics = 0,
            FatAabbCacheMargin = 0.5f,
            SubstepCount = 4,
            IterationCount = 4
        }
    };

    private enum Phase { WaitingForScene, WaitingForButton, Spawning, PostSpawnWait, IssueMove, Warmup, Trial, Done }

    private Phase _phase;
    private int _phaseStartFrame;
    private float _phaseStartTime;
    private int _trialIndex;
    private int _frameInTrial;
    private int _spawnCooldown;
    private SimulationDebuggerEffectiveSettings _baseline;
    private bool _hasBaseline;
    private bool _hasWrittenCsv;
    private UnityEngine.UI.Button _spawnButton;
    private int _targetUnitCount;
    private EntityManager _entityManager;
    private readonly List<TrialResult> _results = new();
    private readonly Accumulator _accumulator = new();

    private void Awake()
    {
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        if (SceneManager.GetActiveScene().buildIndex == 0)
        {
            Debug.Log("[Tuner] 检测到连接场景，自动进入游戏场景...");
            _phase = Phase.WaitingForScene;
            StartCoroutine(AutoEnterGameScene());
            return;
        }

        BeginAfterSceneLoad();
    }

    private IEnumerator AutoEnterGameScene()
    {
        yield return null;

        World localWorld = World.DefaultGameObjectInjectionWorld;
        if (localWorld == null || !localWorld.IsCreated)
        {
            Debug.LogError("[Tuner] Local World 不可用，无法进入游戏。");
            enabled = false;
            yield break;
        }

        SceneManager.LoadScene(1);
        SceneManager.LoadSceneAsync("SubScene 1", LoadSceneMode.Additive);

        yield return new WaitForSeconds(1f);
        BeginAfterSceneLoad();
    }

    private void BeginAfterSceneLoad()
    {
        _phase = Phase.WaitingForButton;
    }

    private void Update()
    {
        switch (_phase)
        {
            case Phase.WaitingForScene:
                break;
            case Phase.WaitingForButton:
                WaitForButton();
                break;
            case Phase.Spawning:
                SpawnViaButton();
                break;
            case Phase.PostSpawnWait:
                if (Time.time - _phaseStartTime >= PostSpawnDelaySeconds)
                {
                    if (IssueMoveBeforeTrials)
                        IssueRandomMoveCommands();
                    else
                        BeginWarmupForStaticHold();
                }
                break;
            case Phase.IssueMove:
                _phaseStartTime = Time.time;
                _phase = Phase.Warmup;
                _phaseStartFrame = Time.frameCount;
                Debug.Log($"[Tuner] 移动命令已下发，预热 {WarmupFrames} 帧");
                break;
            case Phase.Warmup:
                if (Time.frameCount - _phaseStartFrame >= WarmupFrames)
                    StartNextTrial();
                break;
            case Phase.Trial:
                RunTrial();
                break;
            case Phase.Done:
                break;
        }
    }

    private void OnDestroy()
    {
        WriteCsv();
    }

    // ── 按钮查找 ────────────────────────────────────────

    private void WaitForButton()
    {
        // BasicBuildUIController 在 Test 命名空间，挂的场景里
        var controller = FindFirstObjectByType<Test.BasicBuildUIController>();
        if (controller == null)
            return;

        _spawnButton = controller.create50UnitButton;
        if (_spawnButton == null)
        {
            Debug.LogError("[Tuner] create50UnitButton 未绑定。");
            enabled = false;
            return;
        }

        _targetUnitCount = UnitCount;
        _spawnCooldown = 0;
        _phase = Phase.Spawning;
        _phaseStartFrame = Time.frameCount;
        Debug.Log($"[Tuner] 找到生成按钮，目标 {_targetUnitCount} 个单位，每 5 帧点击一次");
    }

    // ── 单位生成（按钮点击） ─────────────────────────────

    private void SpawnViaButton()
    {
        int currentCount = GetCurrentUnitCount();

        if (currentCount >= _targetUnitCount)
        {
            _phase = Phase.PostSpawnWait;
            _phaseStartTime = Time.time;
            Debug.Log($"[Tuner] 已有 {currentCount} 个单位，等待 {PostSpawnDelaySeconds}s 后进入{(IssueMoveBeforeTrials ? "移动" : "静止")}预热");
            return;
        }

        if (_spawnCooldown > 0)
        {
            _spawnCooldown--;
            return;
        }

        _spawnButton.onClick.Invoke();
        _spawnCooldown = 5;
    }

    private int GetCurrentUnitCount()
    {
        // 不用 TryGetLatest —— 上一轮 Play Mode 的静态 snapshot 可能未清空。
        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
            return 0;
        using var query = world.EntityManager.CreateEntityQuery(typeof(UnitMoveDestination));
        return query.CalculateEntityCount();
    }

    private void BeginWarmupForStaticHold()
    {
        EnsureFlowFieldRuntimeActive();
        _phaseStartTime = Time.time;
        _phaseStartFrame = Time.frameCount;
        _phase = Phase.Warmup;
        Debug.Log($"[Tuner] 未下发移动命令，静止预热 {WarmupFrames} 帧");
    }

    // ── 随机移动命令 ────────────────────────────────────

    private void IssueRandomMoveCommands()
    {
        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
            return;

        _entityManager = world.EntityManager;
        var random = new Unity.Mathematics.Random((uint)(Time.frameCount + 1));

        // 获取流场中心作为参考点
        float3 gridOrigin = float3.zero;
        using (var gridQuery = _entityManager.CreateEntityQuery(typeof(FlowFieldGrid)))
        {
            if (!gridQuery.IsEmptyIgnoreFilter)
                gridOrigin = gridQuery.GetSingleton<FlowFieldGrid>().GridOrigin;
        }

        // 给所有单位设随机目标
        using var query = _entityManager.CreateEntityQuery(
            typeof(UnitMoveDestination));
        var entities = query.ToEntityArray(Allocator.Temp);
        foreach (var entity in entities)
        {
            float2 offset = random.NextFloat2Direction() * random.NextFloat(0.5f, MoveTargetSpread);
            float3 target = gridOrigin + new float3(offset.x, 0f, offset.y);

            _entityManager.SetComponentData(entity, new UnitMoveDestination
            {
                Position = target,
                ArrivalRadius = 1f,
                IsActive = 1,
                OrderVersion = 1
            });
        }

        EnsureFlowFieldRuntimeActive();

        Debug.Log($"[Tuner] 已为 {entities.Length} 个单位下发随机移动目标");
        _phase = Phase.IssueMove;
    }

    private void EnsureFlowFieldRuntimeActive()
    {
        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
            return;

        EntityManager entityManager = world.EntityManager;
        using var stateQuery = entityManager.CreateEntityQuery(typeof(FlowFieldRuntimeState));
        if (stateQuery.IsEmptyIgnoreFilter)
            return;

        FlowFieldRuntimeState runtimeState = stateQuery.GetSingleton<FlowFieldRuntimeState>();
        if (runtimeState.ActiveVersion != 0)
            return;

        runtimeState.ActiveVersion = 1;
        stateQuery.SetSingleton(runtimeState);
    }

    // ── Trial 管理 ───────────────────────────────────────

    private void StartNextTrial()
    {
        if (_trialIndex >= TrialList.Count)
        {
            _phase = Phase.Done;
            WriteCsv();
            Debug.Log("[Tuner] 全部 trial 完成，CSV 已输出。");
            return;
        }

        ParameterTrial trial = TrialList[_trialIndex];

        if (!_hasBaseline && SimulationDebuggerRuntime.TryGetBaselineSettings(out var baseline))
        {
            _baseline = baseline;
            _hasBaseline = true;
        }

        SimulationDebuggerEffectiveSettings settings = _hasBaseline ? _baseline : default;
        trial.ApplyTo(ref settings);
        SimulationDebuggerRuntime.SubmitSettings(settings);

        _phase = Phase.Trial;
        _phaseStartFrame = Time.frameCount;
        _frameInTrial = 0;
        _accumulator.Reset();

        Debug.Log($"[Tuner] Trial {_trialIndex + 1}/{TrialList.Count}: {trial.Label}");
    }

    private void RunTrial()
    {
        _frameInTrial++;
        int settleEnd = TrialFrames - StatisticsFrames;

        if (_frameInTrial > settleEnd &&
            SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot s))
        {
            _accumulator.Accumulate(s);
        }

        if (_frameInTrial >= TrialFrames)
        {
            TrialResult result = _accumulator.Finalize(TrialList[_trialIndex].Label, _trialIndex);
            _results.Add(result);

            Debug.Log(
                $"[Tuner] {result.Label}: " +
                $"dirty={result.AverageTopologyDirtyBodies:F1} " +
                $"neighbors={result.AveragePersistentNeighborPairs:F0} " +
                $"rebuild={result.AverageFullRebuilds:F2} " +
                $"solver={result.AverageSolverNs / 1000f:F1}us");

            _trialIndex++;
            StartNextTrial();
        }
    }

    // ── CSV 输出 ─────────────────────────────────────────

    private void WriteCsv()
    {
        if (_hasWrittenCsv || _results.Count == 0)
            return;

        _hasWrittenCsv = true;
        string path = Path.Combine(Application.dataPath, "..", "adaptive_tuning_result.csv");
        using var writer = new StreamWriter(path);
        writer.WriteLine(
            "Label,UnitCount,FrameCount," +
            "AvgSolverNs,AvgIterationNs,AvgSoftAvoidNs," +
            "AvgProxyValidationNs,AvgLocalBroadPhaseNs,AvgPairDiffNs,AvgClassificationNs," +
            "AvgTopologyDirtyBodies,AvgPersistentNeighborPairs,AvgFullRebuilds,AvgIncrementalRepairs," +
            "ContactPairs,ActivePairs,PredictivePairs," +
            "CrossFrameTopologyEnabled,EnableAdaptive,CrossSubstepCacheEnabled,DiagnosticsEnabled," +
            "PredictiveSkin,GuardMargin,Substeps,Iterations,AdpCellSpan,AdpMinPerCell,AdpEnableScore");

        foreach (TrialResult r in _results)
        {
            writer.WriteLine(
                $"{r.Label},{r.UnitCount},{r.FrameCount}," +
                $"{r.AverageSolverNs:F0},{r.AverageIterationNs:F0},{r.AverageSoftAvoidanceNs:F0}," +
                $"{r.AverageProxyValidationNs:F0},{r.AverageLocalBroadPhaseNs:F0},{r.AveragePairDiffNs:F0},{r.AverageClassificationNs:F0}," +
                $"{r.AverageTopologyDirtyBodies:F2},{r.AveragePersistentNeighborPairs:F2},{r.AverageFullRebuilds:F3},{r.AverageIncrementalRepairs:F3}," +
                $"{r.AverageContactPairs:F0},{r.AverageActivePairs:F0},{r.AveragePredictivePairs:F0}," +
                $"{r.EnableFatAabbCache},{r.EnableAdaptiveFatAabb},{r.EnableTimestepContactSetCache},{r.EnableDiagnostics}," +
                $"{r.PredictiveSkin:F3},{r.FatAabbCacheMargin:F3},{r.SubstepCount},{r.IterationCount},{r.AdaptiveDetectionCellSpan},{r.AdaptiveMinimumUnitsPerCell},{r.AdaptiveEnableScore:F2}");
        }

        Debug.Log($"[Tuner] CSV 已输出: {path}");
    }

    // ── 辅助类型 ─────────────────────────────────────────

    private sealed class Accumulator
    {
        private int _frames;
        private long _solverTotal;
        private long _iterTotal;
        private long _softAvoidTotal;
        private long _contactPairTotal;
        private long _activePairTotal;
        private long _predictivePairTotal;
        private int _topologyDirtyBodies;
        private int _persistentNeighborPairs;
        private int _fullRebuilds;
        private int _incrementalRepairs;
        private long _proxyValidationTotal;
        private long _localBroadPhaseTotal;
        private long _pairDiffTotal;
        private long _classificationTotal;
        private int _unitCount;

        public void Reset()
        {
            _frames = 0;
            _solverTotal = 0;
            _iterTotal = 0;
            _softAvoidTotal = 0;
            _contactPairTotal = 0;
            _activePairTotal = 0;
            _predictivePairTotal = 0;
            _topologyDirtyBodies = 0;
            _persistentNeighborPairs = 0;
            _fullRebuilds = 0;
            _incrementalRepairs = 0;
            _proxyValidationTotal = 0;
            _localBroadPhaseTotal = 0;
            _pairDiffTotal = 0;
            _classificationTotal = 0;
        }

        public void Accumulate(SimulationDebuggerFrameSnapshot s)
        {
            _frames++;
            _unitCount = s.Overview.UnitCount;
            _solverTotal += s.Overview.SolverNanoseconds;
            _iterTotal += s.Overview.IterationNanoseconds;
            _softAvoidTotal += s.Overview.SoftAvoidanceNanoseconds;
            _contactPairTotal += s.ContactSet.ContactSetSize;
            _activePairTotal += s.ContactSet.ActiveContactCount;
            _predictivePairTotal += s.ContactSet.PredictiveContactCount;
            IncrementalContactPipelineStatistics pipeline =
                IncrementalContactPipelineDiagnosticsRuntime.Latest.Statistics;
            _topologyDirtyBodies += pipeline.TopologyDirtyBodyCount;
            _persistentNeighborPairs += pipeline.PersistentNeighborPairCount;
            _fullRebuilds += pipeline.FullRebuildCount;
            _incrementalRepairs += pipeline.IncrementalRepairCount;
            _proxyValidationTotal += pipeline.ProxyValidationNanoseconds;
            _localBroadPhaseTotal += pipeline.LocalBroadPhaseNanoseconds;
            _pairDiffTotal += pipeline.PairDiffNanoseconds;
            _classificationTotal += pipeline.SweptClassificationNanoseconds;
        }

        public TrialResult Finalize(string label, int trialIndex)
        {
            SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot last);
            SimulationDebuggerEffectiveSettings settings = last?.EffectiveSettings ?? default;

            float inv = 1f / math.max(1, _frames);
            return new TrialResult
            {
                Label = label,
                TrialIndex = trialIndex,
                UnitCount = _unitCount,
                FrameCount = _frames,
                AverageTopologyDirtyBodies = _topologyDirtyBodies * inv,
                AveragePersistentNeighborPairs = _persistentNeighborPairs * inv,
                AverageFullRebuilds = _fullRebuilds * inv,
                AverageIncrementalRepairs = _incrementalRepairs * inv,
                AverageProxyValidationNs = _proxyValidationTotal * inv,
                AverageLocalBroadPhaseNs = _localBroadPhaseTotal * inv,
                AveragePairDiffNs = _pairDiffTotal * inv,
                AverageClassificationNs = _classificationTotal * inv,
                AverageSolverNs = (long)(_solverTotal * inv),
                AverageIterationNs = (long)(_iterTotal * inv),
                AverageSoftAvoidanceNs = (long)(_softAvoidTotal * inv),
                AverageContactPairs = _contactPairTotal * inv,
                AverageActivePairs = _activePairTotal * inv,
                AveragePredictivePairs = _predictivePairTotal * inv,
                EnableFatAabbCache = settings.EnableFatAabbCache,
                EnableAdaptiveFatAabb = settings.EnableAdaptiveFatAabb,
                EnableTimestepContactSetCache = settings.EnableTimestepContactSetCache,
                EnableDiagnostics = settings.EnableDiagnostics,
                PredictiveSkin = settings.PredictiveSkin,
                FatAabbCacheMargin = settings.FatAabbCacheMargin,
                SubstepCount = settings.SubstepCount,
                IterationCount = settings.IterationCount,
                AdaptiveDetectionCellSpan = settings.AdaptiveDetectionCellSpan,
                AdaptiveMinimumUnitsPerCell = settings.AdaptiveMinimumUnitsPerCell,
                AdaptiveEnableScore = settings.AdaptiveEnableScore,
            };
        }
    }

    [Serializable]
    public sealed class ParameterTrial
    {
        public string Label;
        public byte EnablePredictivePairGeneration = 1;
        public byte EnableFatAabbCache = 1;
        public byte EnableAdaptiveFatAabb = 1;
        public byte EnableTimestepContactSetCache = 1;
        public byte EnableDiagnostics = 1;
        public float PredictiveSkin;
        public float FatAabbCacheMargin;
        public int SubstepCount;
        public int IterationCount;
        public int AdaptiveDetectionCellSpan;
        public int AdaptiveMinimumUnitsPerCell;
        public int AdaptiveMinimumUnitsPerRegion;
        public float AdaptiveEnableScore;

        public void ApplyTo(ref SimulationDebuggerEffectiveSettings s)
        {
            s.EnablePredictivePairGeneration = EnablePredictivePairGeneration;
            s.EnableFatAabbCache = EnableFatAabbCache;
            s.EnableAdaptiveFatAabb = EnableAdaptiveFatAabb;
            s.EnableTimestepContactSetCache = EnableTimestepContactSetCache;
            s.EnableDiagnostics = EnableDiagnostics;
            s.PredictiveSkin = PredictiveSkin;
            if (FatAabbCacheMargin > 0f)
                s.FatAabbCacheMargin = FatAabbCacheMargin;
            if (SubstepCount > 0)
                s.SubstepCount = SubstepCount;
            if (IterationCount > 0)
                s.IterationCount = IterationCount;
            if (AdaptiveDetectionCellSpan > 0)
                s.AdaptiveDetectionCellSpan = AdaptiveDetectionCellSpan;
            if (AdaptiveMinimumUnitsPerCell > 0)
                s.AdaptiveMinimumUnitsPerCell = AdaptiveMinimumUnitsPerCell;
            if (AdaptiveMinimumUnitsPerRegion > 0)
                s.AdaptiveMinimumUnitsPerRegion = AdaptiveMinimumUnitsPerRegion;
            if (AdaptiveEnableScore > 0f)
                s.AdaptiveEnableScore = AdaptiveEnableScore;
        }
    }

    private sealed class TrialResult
    {
        public string Label;
        public int TrialIndex;
        public int UnitCount;
        public int FrameCount;
        public float AverageTopologyDirtyBodies;
        public float AveragePersistentNeighborPairs;
        public float AverageFullRebuilds;
        public float AverageIncrementalRepairs;
        public float AverageProxyValidationNs;
        public float AverageLocalBroadPhaseNs;
        public float AveragePairDiffNs;
        public float AverageClassificationNs;
        public long AverageSolverNs;
        public long AverageIterationNs;
        public long AverageSoftAvoidanceNs;
        public float AverageContactPairs;
        public float AverageActivePairs;
        public float AveragePredictivePairs;
        public byte EnableFatAabbCache;
        public byte EnableAdaptiveFatAabb;
        public byte EnableTimestepContactSetCache;
        public byte EnableDiagnostics;
        public float PredictiveSkin;
        public float FatAabbCacheMargin;
        public int SubstepCount;
        public int IterationCount;
        public int AdaptiveDetectionCellSpan;
        public int AdaptiveMinimumUnitsPerCell;
        public float AdaptiveEnableScore;
    }
}
}
