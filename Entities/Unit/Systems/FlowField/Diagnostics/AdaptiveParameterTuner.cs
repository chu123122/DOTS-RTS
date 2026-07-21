using System;
using System.Collections.Generic;
using System.IO;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using _RePlaySystem.Base;
using 通用;

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
    [Min(1)] public int WarmupFrames = 180;
    [Min(1)] public int TrialFrames = 120;
    [Min(1)] public int StatisticsFrames = 60;

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

    private enum Phase { WaitingForScene, WaitingForWorld, WaitingForPrefabs, Spawning, Warmup, Trial, Done }

    private Phase _phase;
    private int _phaseStartFrame;
    private int _trialIndex;
    private int _frameInTrial;
    private SimulationDebuggerEffectiveSettings _baseline;
    private bool _hasBaseline;
    private EntityManager _entityManager;
    private readonly List<TrialResult> _results = new();
    private readonly Accumulator _accumulator = new();

    private void Awake()
    {
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        // 如果还在连接场景，自动进入游戏场景
        if (UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex == 0)
        {
            Debug.Log("[Tuner] 检测到连接场景，自动进入游戏场景...");
            _phase = Phase.WaitingForScene;
            StartCoroutine(AutoEnterGameScene());
            return;
        }

        BeginGameLoop();
    }

    private System.Collections.IEnumerator AutoEnterGameScene()
    {
        // 等一帧确保场景初始化完成
        yield return null;

        World localWorld = World.DefaultGameObjectInjectionWorld;
        if (localWorld == null || !localWorld.IsCreated)
        {
            Debug.LogError("[Tuner] Local World 不可用，无法进入游戏。");
            enabled = false;
            yield break;
        }

        UnityEngine.SceneManagement.SceneManager.LoadScene(1);
        UnityEngine.SceneManagement.SceneManager.LoadSceneAsync(
            "SubScene 1",
            UnityEngine.SceneManagement.LoadSceneMode.Additive);

        // 等场景加载完成
        yield return new WaitForSeconds(1f);
        BeginGameLoop();
    }

    private void BeginGameLoop()
    {
        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
        {
            Debug.LogError("[Tuner] 游戏 World 不可用。");
            enabled = false;
            return;
        }
        _entityManager = world.EntityManager;
        _phase = Phase.WaitingForPrefabs;
    }

    private void Update()
    {
        switch (_phase)
        {
            case Phase.WaitingForScene:
            case Phase.WaitingForWorld:
                // 由协程或 BeginGameLoop 推进
                break;
            case Phase.WaitingForPrefabs:
                WaitForPrefabs();
                break;
            case Phase.Spawning:
                _phase = Phase.Warmup;
                _phaseStartFrame = Time.frameCount;
                Debug.Log($"[Tuner] 已生成 {UnitCount} 个单位，预热 {WarmupFrames} 帧");
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

    // ── prefab 等待 ──────────────────────────────────────

    private void WaitForPrefabs()
    {
        if (_entityManager.World == null || !_entityManager.World.IsCreated)
            return;

        using var query = _entityManager.CreateEntityQuery(typeof(RtsLocalPrefabs));
        if (query.IsEmptyIgnoreFilter)
            return;

        Entity prefab = query.GetSingleton<RtsLocalPrefabs>().Entity;
        SpawnUnits(prefab);
    }

    // ── 单位生成 ─────────────────────────────────────────

    private void SpawnUnits(Entity prefab)
    {
        var random = new Unity.Mathematics.Random((uint)(Time.frameCount + 1));

        for (int i = 0; i < UnitCount; i++)
        {
            float2 offset = random.NextFloat2Direction() * random.NextFloat(SpawnSpread);
            float3 pos = ClusterCenter + new float3(offset.x, 0f, offset.y);

            Entity unit = _entityManager.Instantiate(prefab);
            _entityManager.SetComponentData(unit, LocalTransform.FromPosition(pos));

            if (_entityManager.HasComponent<LocalInstance>(unit))
                _entityManager.SetComponentData(unit, new LocalInstance { Id = i + 1000 });
        }

        _phase = Phase.Spawning;
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
