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
    [Min(1)] public int WarmupFrames = 180;
    [Min(1)] public int TrialFrames = 120;
    [Min(1)] public int StatisticsFrames = 60;

    [Header("移动")]
    public float2 MoveTargetSpread = 10f;

    [Header("Trials")]
    public List<ParameterTrial> TrialList = new()
    {
        new() { Label = "baseline" },
        new() { Label = "adaptive_on",  EnableAdaptiveFatAabb = 1 },
        new() { Label = "adaptive_off", EnableAdaptiveFatAabb = 0 },
        new() { Label = "fatcache_off", EnableFatAabbCache = 0 },
        new() { Label = "skin_0.1",     PredictiveSkin = 0.1f },
        new() { Label = "skin_0.2",     PredictiveSkin = 0.2f },
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
                    IssueRandomMoveCommands();
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
        if (_results.Count > 0)
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
            Debug.Log($"[Tuner] 已有 {currentCount} 个单位，等待 {PostSpawnDelaySeconds}s 后下发移动命令");
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
        if (SimulationDebuggerRuntime.TryGetLatest(out var s))
            return s.Overview.UnitCount;
        return 0;
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

        // 激活流场求解
        using var stateQuery = _entityManager.CreateEntityQuery(typeof(FlowFieldRuntimeState));
        if (!stateQuery.IsEmptyIgnoreFilter)
        {
            var runtimeState = stateQuery.GetSingleton<FlowFieldRuntimeState>();
            if (runtimeState.ActiveVersion == 0)
            {
                runtimeState.ActiveVersion = 1;
                stateQuery.SetSingleton(runtimeState);
            }
        }

        Debug.Log($"[Tuner] 已为 {entities.Length} 个单位下发随机移动目标");
        _phase = Phase.IssueMove;
    }

    // ── Trial 管理 ───────────────────────────────────────

    private void StartNextTrial()
    {
        if (_trialIndex >= TrialList.Count)
        {
            _phase = Phase.Done;
            Debug.Log("[Tuner] 全部 trial 完成，退出 Play Mode 时输出 CSV");
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

            int cacheAttempts = result.CacheReuseCount + result.CacheRebuildCount;
            float hitRate = cacheAttempts > 0
                ? (float)result.CacheReuseCount / cacheAttempts * 100f
                : 0f;
            Debug.Log(
                $"[Tuner] {result.Label}: " +
                $"hit={hitRate:F0}% " +
                $"rebuild={result.CacheRebuildCount} " +
                $"fallback={result.FallbackCount} " +
                $"solver={result.AverageSolverNs / 1000f:F1}us");

            _trialIndex++;
            StartNextTrial();
        }
    }

    // ── CSV 输出 ─────────────────────────────────────────

    private void WriteCsv()
    {
        string path = Path.Combine(Application.dataPath, "..", "adaptive_tuning_result.csv");
        using var writer = new StreamWriter(path);
        writer.WriteLine(
            "Label,UnitCount,FrameCount," +
            "CacheReuse,CacheRebuild,CacheHitRate," +
            "FallbackCount,InvalidationCount," +
            "AvgSolverNs,AvgIterationNs,AvgSoftAvoidNs," +
            "EnableFatAabb,EnableAdaptive,EnableTimestepCache,PredictiveSkin");

        foreach (TrialResult r in _results)
        {
            int attempts = r.CacheReuseCount + r.CacheRebuildCount;
            float hitRate = attempts > 0 ? (float)r.CacheReuseCount / attempts : 0f;
            writer.WriteLine(
                $"{r.Label},{r.UnitCount},{r.FrameCount}," +
                $"{r.CacheReuseCount},{r.CacheRebuildCount},{hitRate:F4}," +
                $"{r.FallbackCount},{r.InvalidationCount}," +
                $"{r.AverageSolverNs:F0},{r.AverageIterationNs:F0},{r.AverageSoftAvoidanceNs:F0}," +
                $"{r.EnableFatAabbCache},{r.EnableAdaptiveFatAabb},{r.EnableTimestepContactSetCache},{r.PredictiveSkin:F3}");
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
        private int _cacheReuse;
        private int _cacheRebuild;
        private int _fallback;
        private int _invalidation;
        private int _unitCount;

        public void Reset()
        {
            _frames = 0;
            _solverTotal = 0;
            _iterTotal = 0;
            _softAvoidTotal = 0;
            _cacheReuse = 0;
            _cacheRebuild = 0;
            _fallback = 0;
            _invalidation = 0;
        }

        public void Accumulate(SimulationDebuggerFrameSnapshot s)
        {
            _frames++;
            _unitCount = s.Overview.UnitCount;
            _solverTotal += s.Overview.SolverNanoseconds;
            _iterTotal += s.Overview.IterationNanoseconds;
            _softAvoidTotal += s.Overview.SoftAvoidanceNanoseconds;
            _cacheReuse += s.BroadPhase.ReuseCount;
            _cacheRebuild += s.BroadPhase.RebuildCount;
            _fallback += s.BroadPhase.FallbackCount;
            _invalidation += s.BroadPhase.InvalidationCount;
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
                CacheReuseCount = _cacheReuse,
                CacheRebuildCount = _cacheRebuild,
                FallbackCount = _fallback,
                InvalidationCount = _invalidation,
                AverageSolverNs = (long)(_solverTotal * inv),
                AverageIterationNs = (long)(_iterTotal * inv),
                AverageSoftAvoidanceNs = (long)(_softAvoidTotal * inv),
                EnableFatAabbCache = settings.EnableFatAabbCache,
                EnableAdaptiveFatAabb = settings.EnableAdaptiveFatAabb,
                EnableTimestepContactSetCache = settings.EnableTimestepContactSetCache,
                PredictiveSkin = settings.PredictiveSkin,
            };
        }
    }

    [Serializable]
    public sealed class ParameterTrial
    {
        public string Label;
        public byte EnableFatAabbCache = 1;
        public byte EnableAdaptiveFatAabb = 1;
        public byte EnableTimestepContactSetCache = 1;
        public float PredictiveSkin;

        public void ApplyTo(ref SimulationDebuggerEffectiveSettings s)
        {
            s.EnableFatAabbCache = EnableFatAabbCache;
            s.EnableAdaptiveFatAabb = EnableAdaptiveFatAabb;
            s.EnableTimestepContactSetCache = EnableTimestepContactSetCache;
            s.PredictiveSkin = PredictiveSkin;
        }
    }

    private sealed class TrialResult
    {
        public string Label;
        public int TrialIndex;
        public int UnitCount;
        public int FrameCount;
        public int CacheReuseCount;
        public int CacheRebuildCount;
        public int FallbackCount;
        public int InvalidationCount;
        public long AverageSolverNs;
        public long AverageIterationNs;
        public long AverageSoftAvoidanceNs;
        public byte EnableFatAabbCache;
        public byte EnableAdaptiveFatAabb;
        public byte EnableTimestepContactSetCache;
        public float PredictiveSkin;
    }
}
}
