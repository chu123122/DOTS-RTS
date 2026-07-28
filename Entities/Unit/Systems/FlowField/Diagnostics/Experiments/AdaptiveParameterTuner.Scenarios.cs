using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Diagnostics
{
public sealed partial class AdaptiveParameterTuner
{
    [Header("场景基准测试 (Inspector)")]
    [Tooltip("启用后使用下方三类可复现场景；关闭则保留旧版自动调优流程。")]
    public bool UseScenarioBenchmark = true;
    [Tooltip("每个（场景×缓存配置）组合重复测量的次数。多次取中位数以消抖动，3 次是性价比之选。")]
    [Min(1)] public int BenchmarkRepetitions = 3;
    [Tooltip("正式测量前的缓存预热帧数。让持久邻居拓扑/接触集缓存先填充，避免冷启动计入统计。")]
    [Min(0)] public int BenchmarkCachePrimingFrames = 60;
    [Tooltip("静止密集场景（MoveThenHold）的正式测量帧数。单位到位后持续测这么多帧。")]
    [Min(1)] public int BenchmarkHoldFrames = 600;
    [Tooltip("往返场景（Open/Obstacle PingPong）单程测量帧数。一个往返 = 2 段。")]
    [Min(1)] public int BenchmarkFramesPerLeg = 300;
    [Tooltip("原始逐帧采样的间隔帧数。10 = 每 10 帧写一行 raw CSV，控制 raw 文件体积。")]
    [Min(1)] public int BenchmarkRawSampleInterval = 10;
    [Tooltip("判定单位“已稳定”需连续满足的帧数。用于捕获基线快照前确认队形收敛。")]
    [Min(1)] public int BenchmarkSettledFrames = 30;
    [Tooltip("等待单位稳定的最大帧数。超时则判失败，避免无限等待。")]
    [Min(1)] public int BenchmarkMaxPreparationFrames = 3600;
    [Tooltip("单位速度低于此阈值视为“已稳定”（配合 SettledUnitRatio 一起判定）。")]
    [Min(0f)] public float BenchmarkSettledSpeedThreshold = 0.25f;
    [Tooltip("“已稳定”所需达到的单位比例。0.99 = 99% 单位速度低于阈值才算稳定。")]
    [Range(0.8f, 1f)] public float BenchmarkSettledUnitRatio = 0.99f;

    [Tooltip("三种模式均从 PointA 稳定后的同一 ECS 快照开始。ObstaclePingPong 直接使用场景障碍物。")]
    public List<BenchmarkScenario> BenchmarkScenarios = new()
    {
        new BenchmarkScenario
        {
            Enabled = true, Label = "静止密集", Mode = BenchmarkScenarioMode.MoveThenHold,
            PointA = new float3(2.584251f, 3.814697E-06f, -4.022369f), RoundTripCount = 1
        },
        new BenchmarkScenario
        {
            Enabled = true, Label = "开阔往返", Mode = BenchmarkScenarioMode.OpenPingPong,
            PointA = new float3(2.584251f, 3.814697E-06f, -4.022369f),
            PointB = new float3(27.87032f, 0f, -2.884846f), RoundTripCount = 2
        },
        new BenchmarkScenario
        {
            Enabled = true, Label = "障碍往返", Mode = BenchmarkScenarioMode.ObstaclePingPong,
            PointA = new float3(2.584251f, 3.814697E-06f, -4.022369f),
            PointB = new float3(11.11795f, 0f, 25.84306f), RoundTripCount = 2
        }
    };

    private enum BenchmarkPhase { WaitingForScene, WaitingForButton, Spawning, Delay, Preparing, Priming, Measuring, Done, Failed }
    private BenchmarkPhase _benchmarkPhase;
    private UnityEngine.UI.Button _benchmarkSpawnButton;
    private int _benchmarkSpawnCooldown;
    private float _benchmarkDelayStart;
    private int _benchmarkScenarioIndex = -1;
    private int _benchmarkRunIndex;
    private int _benchmarkSettledCount;
    private int _benchmarkPreparationFrames;
    private int _benchmarkNoSnapshotFrames;
    private int _benchmarkPrimingFrames;
    private int _benchmarkMeasuredFrames;
    private int _benchmarkLegIndex;
    private uint _benchmarkRestoreRequestVersion;
    private ulong _benchmarkLastSnapshotVersion;
    private EntityManager _benchmarkEntityManager;
    private ScenarioSnapshot _benchmarkSnapshot;
    private SimulationDebuggerEffectiveSettings _benchmarkBaselineSettings;
    private bool _benchmarkHasBaselineSettings;
    private readonly List<BenchmarkRun> _benchmarkRuns = new();
    private readonly List<BenchmarkResult> _benchmarkResults = new();
    private readonly BenchmarkAccumulator _benchmarkAccumulator = new();
    private StreamWriter _benchmarkRawWriter;
    private string _benchmarkDirectory;
    private bool _benchmarkFinalized;

    private BenchmarkScenario CurrentBenchmarkScenario => BenchmarkScenarios[_benchmarkScenarioIndex];
    private BenchmarkRun CurrentBenchmarkRun => _benchmarkRuns[_benchmarkRunIndex];

    private void UpdateScenarioBenchmark()
    {
        switch (_benchmarkPhase)
        {
            case BenchmarkPhase.WaitingForScene:
                if (SceneManager.GetActiveScene().buildIndex != 0)
                    _benchmarkPhase = BenchmarkPhase.WaitingForButton;
                break;
            case BenchmarkPhase.WaitingForButton:
                FindBenchmarkSpawnButton();
                break;
            case BenchmarkPhase.Spawning:
                SpawnBenchmarkUnits();
                break;
            case BenchmarkPhase.Delay:
                if (Time.time - _benchmarkDelayStart >= PostSpawnDelaySeconds)
                    StartNextBenchmarkScenario();
                break;
            case BenchmarkPhase.Preparing:
                UpdateBenchmarkPreparation();
                break;
            case BenchmarkPhase.Priming:
                UpdateBenchmarkPriming();
                break;
            case BenchmarkPhase.Measuring:
                UpdateBenchmarkMeasurement();
                break;
        }
    }

    private void FindBenchmarkSpawnButton()
    {
        var controller = FindFirstObjectByType<Test.BasicBuildUIController>();
        if (controller == null)
            return;
        _benchmarkSpawnButton = controller.create50UnitButton;
        if (_benchmarkSpawnButton == null)
        {
            FailBenchmark("create50UnitButton 未绑定。");
            return;
        }
        _benchmarkPhase = BenchmarkPhase.Spawning;
        Debug.Log($"[Benchmark] 使用 Inspector 场景测试，目标 {UnitCount} 个单位。");
    }

    private void SpawnBenchmarkUnits()
    {
        if (GetCurrentUnitCount() >= UnitCount)
        {
            _benchmarkDelayStart = Time.time;
            _benchmarkPhase = BenchmarkPhase.Delay;
            return;
        }
        if (_benchmarkSpawnCooldown-- > 0)
            return;
        _benchmarkSpawnButton.onClick.Invoke();
        _benchmarkSpawnCooldown = 5;
    }

    private void StartNextBenchmarkScenario()
    {
        _benchmarkScenarioIndex = FindEnabledBenchmarkScenario(_benchmarkScenarioIndex + 1);
        if (_benchmarkScenarioIndex < 0)
        {
            _benchmarkPhase = BenchmarkPhase.Done;
            FinalizeScenarioBenchmark();
            Debug.Log("[Benchmark] 所有场景已完成。");
            return;
        }
        if (!TryGetBenchmarkEntityManager(out _benchmarkEntityManager))
        {
            Debug.LogWarning("[Benchmark] TryGetBenchmarkEntityManager 失败，无法开始下一个场景。");
            return;
        }
        Debug.Log($"[Benchmark] 开始场景 {_benchmarkScenarioIndex}: {CurrentBenchmarkScenario.Label}，共 {BenchmarkScenarios.Count} 个场景。");
        if (CurrentBenchmarkScenario.Mode != BenchmarkScenarioMode.MoveThenHold && CurrentBenchmarkScenario.RoundTripCount < 1)
        {
            FailBenchmark($"{CurrentBenchmarkScenario.Label}: RoundTripCount 必须 >= 1。");
            return;
        }
        EnsureBenchmarkFlowFieldActive();
        IssueBenchmarkMoveOrder(CurrentBenchmarkScenario.PointA);
        _benchmarkPreparationFrames = 0;
        _benchmarkSettledCount = 0;
        _benchmarkPhase = BenchmarkPhase.Preparing;
        Debug.Log($"[Benchmark] 准备 {CurrentBenchmarkScenario.Label}，移动到 A={FormatBenchmark(CurrentBenchmarkScenario.PointA)}。");
    }

    private int FindEnabledBenchmarkScenario(int start)
    {
        for (int i = start; i < BenchmarkScenarios.Count; i++)
            if (BenchmarkScenarios[i] != null && BenchmarkScenarios[i].Enabled)
                return i;
        return -1;
    }

    private void UpdateBenchmarkPreparation()
    {
        // 每 300 帧输出一次稳定诊断，便于定位未稳定根因。
        if (_benchmarkPreparationFrames > 0 && _benchmarkPreparationFrames % 300 == 0)
            LogSettlementDiagnostics();
        if (++_benchmarkPreparationFrames > BenchmarkMaxPreparationFrames)
        {
            LogSettlementDiagnostics();
            FailBenchmark($"{CurrentBenchmarkScenario.Label} 在 {BenchmarkMaxPreparationFrames} 帧内未稳定。");
            return;
        }
        if (!AreBenchmarkUnitsSettled())
        {
            _benchmarkSettledCount = 0;
            return;
        }
        if (++_benchmarkSettledCount < BenchmarkSettledFrames)
            return;

        try { _benchmarkSnapshot = CaptureBenchmarkSnapshot(); }
        catch (Exception exception) { FailBenchmark("捕获基线失败: " + exception.Message); return; }
        BuildBenchmarkRunSchedule();
        if (_benchmarkRuns.Count == 0)
        {
            FailBenchmark("TrialList 为空。");
            return;
        }
        _benchmarkRunIndex = 0;
        Debug.Log($"[Benchmark] {CurrentBenchmarkScenario.Label} 基线已稳定，hash={_benchmarkSnapshot.Hash:X16}。");
        StartBenchmarkRun();
    }

    private void BuildBenchmarkRunSchedule()
    {
        _benchmarkRuns.Clear();
        int repetitions = math.max(1, BenchmarkRepetitions);
        for (int repetition = 0; repetition < repetitions; repetition++)
        {
            bool reverse = (repetition & 1) != 0;
            for (int offset = 0; offset < TrialList.Count; offset++)
            {
                int index = reverse ? TrialList.Count - 1 - offset : offset;
                if (TrialList[index] != null)
                    _benchmarkRuns.Add(new BenchmarkRun(TrialList[index], repetition + 1));
            }
        }
    }

    private void StartBenchmarkRun()
    {
        if (_benchmarkRunIndex >= _benchmarkRuns.Count)
        {
            StartNextBenchmarkScenario();
            return;
        }
        try { RestoreBenchmarkSnapshot(_benchmarkSnapshot); }
        catch (Exception exception) { FailBenchmark("恢复基线失败: " + exception.Message); return; }

        if (!_benchmarkHasBaselineSettings && SimulationDebuggerRuntime.TryGetBaselineSettings(out var baseline))
        {
            _benchmarkBaselineSettings = baseline;
            _benchmarkHasBaselineSettings = true;
        }
        if (!_benchmarkHasBaselineSettings && SimulationDebuggerRuntime.TryGetLatest(out var latest))
        {
            _benchmarkBaselineSettings = latest.EffectiveSettings;
            _benchmarkHasBaselineSettings = true;
        }
        if (!_benchmarkHasBaselineSettings)
        {
            FailBenchmark("没有可用的 SimulationDebugger 有效设置快照。");
            return;
        }

        SimulationDebuggerEffectiveSettings settings = _benchmarkBaselineSettings;
        CurrentBenchmarkRun.Trial.ApplyTo(ref settings);
        if (settings.EnablePersistentContactCache != 0 && settings.EnableTimestepContactSetCache == 0)
        {
            settings.EnableTimestepContactSetCache = 1;
            Debug.LogWarning("[Benchmark] A 依赖 B，已将本次 trial 修正为 A1_B1。");
        }
        SimulationDebuggerRuntime.SubmitSettings(settings);
        SimulationDebuggerRuntime.RequestContactCacheReset();
        _benchmarkAccumulator.Reset();
        _benchmarkPrimingFrames = 0;
        _benchmarkMeasuredFrames = 0;
        _benchmarkNoSnapshotFrames = 0;
        _benchmarkLegIndex = 0;
        _benchmarkLastSnapshotVersion = SimulationDebuggerRuntime.PublishedVersion;
        SimulationDebuggerRuntime.SetLocalRecordingCapture(true);
        _benchmarkPhase = BenchmarkPhase.Priming;
        Debug.Log($"[Benchmark] {CurrentBenchmarkScenario.Label}/{CurrentBenchmarkRun.Trial.Label}/r{CurrentBenchmarkRun.Repetition}: 已恢复基线且清空跨帧缓存。");
    }

    private void UpdateBenchmarkPriming()
    {
        if (!IsBenchmarkRestoreFlowReady())
            return;
        if (!TryGetNewBenchmarkSnapshot(out _))
        {
            if (++_benchmarkNoSnapshotFrames > 300)
            {
                FailBenchmark("超过 300 帧未收到 SimulationDebugger 快照。确认 F8 面板已开启且至少一个窗口可见。");
                return;
            }
            return;
        }
        _benchmarkNoSnapshotFrames = 0;
        if (++_benchmarkPrimingFrames < BenchmarkCachePrimingFrames)
            return;
        if (CurrentBenchmarkScenario.Mode != BenchmarkScenarioMode.MoveThenHold)
            IssueBenchmarkMoveOrder(CurrentBenchmarkScenario.PointB);
        SimulationDebuggerRuntime.SetLocalRecordingCapture(true);
        _benchmarkPhase = BenchmarkPhase.Measuring;
    }

    private void UpdateBenchmarkMeasurement()
    {
        if (!TryGetNewBenchmarkSnapshot(out SimulationDebuggerFrameSnapshot snapshot))
            return;
        _benchmarkMeasuredFrames++;
        _benchmarkAccumulator.Add(snapshot);
        if (_benchmarkMeasuredFrames % math.max(1, BenchmarkRawSampleInterval) == 0)
            WriteBenchmarkRawSample(snapshot);

        bool completed;
        if (CurrentBenchmarkScenario.Mode == BenchmarkScenarioMode.MoveThenHold)
        {
            completed = _benchmarkMeasuredFrames >= BenchmarkHoldFrames;
        }
        else
        {
            int framesPerLeg = math.max(1, BenchmarkFramesPerLeg);
            int totalLegs = math.max(1, CurrentBenchmarkScenario.RoundTripCount) * 2;
            completed = _benchmarkMeasuredFrames >= totalLegs * framesPerLeg;
            if (!completed && _benchmarkMeasuredFrames % framesPerLeg == 0)
            {
                _benchmarkLegIndex++;
                IssueBenchmarkMoveOrder((_benchmarkLegIndex & 1) == 0
                    ? CurrentBenchmarkScenario.PointB
                    : CurrentBenchmarkScenario.PointA);
            }
        }
        if (!completed)
            return;

        _benchmarkResults.Add(_benchmarkAccumulator.ToResult(
            CurrentBenchmarkScenario, CurrentBenchmarkRun, _benchmarkSnapshot.Hash, ComputeBenchmarkStateHash()));
        BenchmarkResult result = _benchmarkResults[_benchmarkResults.Count - 1];
        Debug.Log($"[Benchmark] 完成 {result.Scenario}/{result.Profile}/r{result.Repetition}: solver={result.AverageSolverNs / 1000f:F1}us, dirty={result.AverageDirtyBodies:F1}, pairs={result.AveragePersistentPairs:F0}。");
        _benchmarkRunIndex++;
        if (_benchmarkRunIndex >= _benchmarkRuns.Count)
            Debug.Log($"[Benchmark] 场景 {CurrentBenchmarkScenario.Label} 全部 {_benchmarkRuns.Count} 次运行已完成，切换到下一个场景...");
        StartBenchmarkRun();
    }

    private bool TryGetNewBenchmarkSnapshot(out SimulationDebuggerFrameSnapshot snapshot)
    {
        snapshot = null;
        ulong version = SimulationDebuggerRuntime.PublishedVersion;
        if (version == 0 || version == _benchmarkLastSnapshotVersion || !SimulationDebuggerRuntime.TryGetLatest(out snapshot))
            return false;
        _benchmarkLastSnapshotVersion = version;
        return true;
    }

    private bool TryGetBenchmarkEntityManager(out EntityManager manager)
    {
        World world = World.DefaultGameObjectInjectionWorld;
        if (world == null || !world.IsCreated)
        {
            manager = default;
            FailBenchmark("Local World 不可用。");
            return false;
        }
        manager = world.EntityManager;
        return true;
    }

    private void EnsureBenchmarkFlowFieldActive()
    {
        using var query = _benchmarkEntityManager.CreateEntityQuery(typeof(FlowFieldRuntimeState));
        if (query.IsEmptyIgnoreFilter)
            return;
        FlowFieldRuntimeState state = query.GetSingleton<FlowFieldRuntimeState>();
        if (state.ActiveVersion == 0)
        {
            state.ActiveVersion = 1;
            query.SetSingleton(state);
        }
    }

    private void IssueBenchmarkMoveOrder(float3 target)
    {
        _benchmarkEntityManager.CompleteAllTrackedJobs();
        // MoveOrder 是 IEnableableComponent，不能与普通组件混在同一 GetSingletonEntity 查询里。
        using var orderQuery = _benchmarkEntityManager.CreateEntityQuery(typeof(FlowFieldGlobalTarget));
        if (orderQuery.IsEmptyIgnoreFilter)
        {
            FailBenchmark("缺少 FlowFieldGlobalTarget。");
            return;
        }
        Entity gridEntity = orderQuery.GetSingletonEntity();
        DynamicBuffer<MoveOrderSelectionElement> recipients = _benchmarkEntityManager.GetBuffer<MoveOrderSelectionElement>(gridEntity);
        recipients.Clear();
        using var unitQuery = _benchmarkEntityManager.CreateEntityQuery(ComponentType.ReadOnly<BasicUnitTag>(), ComponentType.ReadOnly<UnitMoveDestination>());
        using NativeArray<Entity> entities = unitQuery.ToEntityArray(Allocator.Temp);
        for (int i = 0; i < entities.Length; i++)
            recipients.Add(new MoveOrderSelectionElement { Entity = entities[i] });
        MoveOrder order = _benchmarkEntityManager.GetComponentData<MoveOrder>(gridEntity);
        order.TargetPosition = target;
        _benchmarkEntityManager.SetComponentData(gridEntity, order);
        _benchmarkEntityManager.SetComponentEnabled<MoveOrder>(gridEntity, true);
    }

    private bool AreBenchmarkUnitsSettled()
    {
        _benchmarkEntityManager.CompleteAllTrackedJobs();
        using var query = _benchmarkEntityManager.CreateEntityQuery(ComponentType.ReadOnly<BasicUnitTag>(), ComponentType.ReadOnly<Velocity>());
        using NativeArray<Velocity> velocities = query.ToComponentDataArray<Velocity>(Allocator.Temp);
        if (velocities.Length == 0)
            return false;
        // 所有场景统一用比例判定，避免单个单位 XPBD 残余抖动阻塞整个 benchmark。
        float requiredRatio = BenchmarkSettledUnitRatio;
        int required = math.max(1, (int)(velocities.Length * requiredRatio));
        float thresholdSq = BenchmarkSettledSpeedThreshold * BenchmarkSettledSpeedThreshold;
        int settled = 0;
        for (int i = 0; i < velocities.Length; i++)
            if (math.lengthsq(velocities[i].Value) <= thresholdSq)
                settled++;
        return settled >= required;
    }

    private void LogSettlementDiagnostics()
    {
        _benchmarkEntityManager.CompleteAllTrackedJobs();
        using var query = _benchmarkEntityManager.CreateEntityQuery(
            ComponentType.ReadOnly<BasicUnitTag>(),
            ComponentType.ReadOnly<UnitMoveDestination>(),
            ComponentType.ReadOnly<Velocity>(),
            ComponentType.ReadOnly<LocalTransform>());
        using NativeArray<UnitMoveDestination> destinations = query.ToComponentDataArray<UnitMoveDestination>(Allocator.Temp);
        using NativeArray<Velocity> velocities = query.ToComponentDataArray<Velocity>(Allocator.Temp);
        using NativeArray<LocalTransform> transforms = query.ToComponentDataArray<LocalTransform>(Allocator.Temp);
        if (velocities.Length == 0) { Debug.Log("[Benchmark] 稳定诊断：当前无单位。"); return; }
        int tooFast = 0, noDest = 0;
        float maxSpeed = 0f, avgSpeed = 0f, maxDist = 0f, avgDist = 0f;
        float thresholdSq = BenchmarkSettledSpeedThreshold * BenchmarkSettledSpeedThreshold;
        for (int i = 0; i < velocities.Length; i++)
        {
            float speedSq = math.lengthsq(velocities[i].Value);
            float speed = math.sqrt(speedSq);
            maxSpeed = math.max(maxSpeed, speed);
            avgSpeed += speed;
            if (destinations[i].IsActive == 0) noDest++;
            if (speedSq > thresholdSq) tooFast++;
            float dist = math.distance(transforms[i].Position, destinations[i].Position);
            maxDist = math.max(maxDist, dist);
            avgDist += dist;
        }
        avgSpeed /= velocities.Length;
        avgDist /= velocities.Length;
        float settledRatio = (velocities.Length - tooFast) / (float)velocities.Length;
        Debug.Log($"[Benchmark] 帧 {_benchmarkPreparationFrames} 稳定诊断："
                + $"总数={velocities.Length} 无目标={noDest} 静止={velocities.Length - tooFast}({settledRatio:P0}) 超速={tooFast}(>{BenchmarkSettledSpeedThreshold:0.00})"
                + $" 最大速度={maxSpeed:0.000} 平均={avgSpeed:0.000}"
                + $" 最大距离={maxDist:0.00} 平均={avgDist:0.00}");
    }

    private ScenarioSnapshot CaptureBenchmarkSnapshot()
    {
        _benchmarkEntityManager.CompleteAllTrackedJobs();
        using var query = CreateBenchmarkStateQuery();
        using NativeArray<Entity> entities = query.ToEntityArray(Allocator.Temp);
        using NativeArray<LocalTransform> transforms = query.ToComponentDataArray<LocalTransform>(Allocator.Temp);
        using NativeArray<Velocity> velocities = query.ToComponentDataArray<Velocity>(Allocator.Temp);
        using NativeArray<UnitMoveDestination> destinations = query.ToComponentDataArray<UnitMoveDestination>(Allocator.Temp);
        using NativeArray<FlowArrivalState> arrivals = query.ToComponentDataArray<FlowArrivalState>(Allocator.Temp);
        var units = new List<BenchmarkUnitState>(entities.Length);
        for (int i = 0; i < entities.Length; i++)
            units.Add(new BenchmarkUnitState
            {
                Entity = entities[i], Transform = transforms[i], Velocity = velocities[i],
                Destination = destinations[i], Arrival = arrivals[i]
            });
        units.Sort((a, b) => a.Entity.Index != b.Entity.Index ? a.Entity.Index.CompareTo(b.Entity.Index) : a.Entity.Version.CompareTo(b.Entity.Version));
        return new ScenarioSnapshot
        {
            Units = units,
            Hash = HashBenchmarkStates(units),
            GlobalTarget = GetBenchmarkGlobalTarget()
        };
    }

    private void RestoreBenchmarkSnapshot(ScenarioSnapshot snapshot)
    {
        _benchmarkEntityManager.CompleteAllTrackedJobs();
        foreach (BenchmarkUnitState unit in snapshot.Units)
        {
            if (!_benchmarkEntityManager.Exists(unit.Entity))
                throw new InvalidOperationException($"单位 {unit.Entity} 已不存在。");
            _benchmarkEntityManager.SetComponentData(unit.Entity, unit.Transform);
            _benchmarkEntityManager.SetComponentData(unit.Entity, unit.Velocity);
            _benchmarkEntityManager.SetComponentData(unit.Entity, unit.Destination);
            _benchmarkEntityManager.SetComponentData(unit.Entity, unit.Arrival);
        }
        SetBenchmarkGlobalTarget(snapshot.GlobalTarget);
        ulong restoredHash = ComputeBenchmarkStateHash();
        if (restoredHash != snapshot.Hash)
            throw new InvalidOperationException($"hash 不一致 expected={snapshot.Hash:X16} actual={restoredHash:X16}");
    }

    private EntityQuery CreateBenchmarkStateQuery() => _benchmarkEntityManager.CreateEntityQuery(
        ComponentType.ReadOnly<BasicUnitTag>(), ComponentType.ReadOnly<LocalTransform>(), ComponentType.ReadOnly<Velocity>(),
        ComponentType.ReadOnly<UnitMoveDestination>(), ComponentType.ReadOnly<FlowArrivalState>());

    private float3 GetBenchmarkGlobalTarget()
    {
        using var query = _benchmarkEntityManager.CreateEntityQuery(typeof(FlowFieldGlobalTarget));
        return query.GetSingleton<FlowFieldGlobalTarget>().TargetPosition;
    }

    private void SetBenchmarkGlobalTarget(float3 target)
    {
        using var query = _benchmarkEntityManager.CreateEntityQuery(typeof(FlowFieldGlobalTarget));
        Entity entity = query.GetSingletonEntity();
        _benchmarkEntityManager.SetComponentData(entity, new FlowFieldGlobalTarget { TargetPosition = target });
        if (_benchmarkEntityManager.HasComponent<RecalculateFlowFieldTag>(entity))
        {
            RecalculateFlowFieldTag request = _benchmarkEntityManager.GetComponentData<RecalculateFlowFieldTag>(entity);
            request.RequestVersion++;
            _benchmarkEntityManager.SetComponentData(entity, request);
            _benchmarkEntityManager.SetComponentEnabled<RecalculateFlowFieldTag>(entity, true);
            _benchmarkRestoreRequestVersion = request.RequestVersion;
        }
    }

    private bool IsBenchmarkRestoreFlowReady()
    {
        if (_benchmarkRestoreRequestVersion == 0)
            return true;
        using var query = _benchmarkEntityManager.CreateEntityQuery(typeof(FlowFieldRuntimeState));
        if (query.IsEmptyIgnoreFilter)
            return false;
        return query.GetSingleton<FlowFieldRuntimeState>().ActiveRequestVersion == _benchmarkRestoreRequestVersion;
    }

    private ulong ComputeBenchmarkStateHash() => CaptureBenchmarkSnapshot().Hash;

    private static ulong HashBenchmarkStates(List<BenchmarkUnitState> units)
    {
        ulong hash = 1469598103934665603UL;
        foreach (BenchmarkUnitState unit in units)
        {
            HashBenchmark(ref hash, (uint)unit.Entity.Index);
            HashBenchmark(ref hash, (uint)unit.Entity.Version);
            HashBenchmark(ref hash, math.asuint(unit.Transform.Position.x));
            HashBenchmark(ref hash, math.asuint(unit.Transform.Position.y));
            HashBenchmark(ref hash, math.asuint(unit.Transform.Position.z));
            HashBenchmark(ref hash, math.asuint(unit.Velocity.Value.x));
            HashBenchmark(ref hash, math.asuint(unit.Velocity.Value.y));
            HashBenchmark(ref hash, math.asuint(unit.Velocity.Value.z));
            HashBenchmark(ref hash, math.asuint(unit.Destination.Position.x));
            HashBenchmark(ref hash, math.asuint(unit.Destination.Position.y));
            HashBenchmark(ref hash, math.asuint(unit.Destination.Position.z));
            HashBenchmark(ref hash, unit.Destination.OrderVersion);
            HashBenchmark(ref hash, unit.Destination.IsActive);
            HashBenchmark(ref hash, unit.Arrival.IsSettled ? 1u : 0u);
        }
        return hash;
    }

    private static void HashBenchmark(ref ulong hash, uint value)
    {
        hash ^= value;
        hash *= 1099511628211UL;
    }

    private void EnsureBenchmarkOutputWriter()
    {
        if (_benchmarkRawWriter != null)
            return;
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        _benchmarkDirectory = Path.Combine(Application.dataPath, "..", "BenchmarkResults", "contact_benchmark_" + stamp);
        Directory.CreateDirectory(_benchmarkDirectory);
        _benchmarkRawWriter = new StreamWriter(
            Path.Combine(_benchmarkDirectory, "adaptive_tuning_raw.csv"),
            false, new System.Text.UTF8Encoding(true)); // UTF-8 BOM，Excel 中文不乱码
        _benchmarkRawWriter.WriteLine(string.Join(",", new[]
        {
            "场景", "场景模式", "缓存配置", "重复", "采样帧", "往返段", "基线哈希",
            "求解总耗时ns", "接触对生成耗时ns", "接触集构建耗时ns", "迭代投影耗时ns", "软避让耗时ns",
            "全量扫描耗时ns", "持久对映射耗时ns", "代理校验耗时ns", "局部broadphase耗时ns", "对差异耗时ns", "分类耗时ns", "接触激活耗时ns", "回退耗时ns",
            "拓扑脏体数", "运动脏体数", "持久邻居对数", "交互对数", "软避让对数",
            "分类评估次数", "分类跳过次数", "软对评估次数", "约束评估次数",
            "持久视图复用", "持久视图重建", "交互包络逃逸", "全量重建次数", "增量修复次数",
            "接触对数", "活跃对数", "预测对数", "软避让漏检"
        }));
    }

    private void WriteBenchmarkRawSample(SimulationDebuggerFrameSnapshot snapshot)
    {
        EnsureBenchmarkOutputWriter();
        IncrementalContactPipelineStatistics p = IncrementalContactPipelineDiagnosticsRuntime.Latest.Statistics;
        PredictiveDiscContactStatistics solver =
            IncrementalContactPipelineDiagnosticsRuntime.Latest.SolverStatistics;
        _benchmarkRawWriter.WriteLine(string.Join(",", new[]
        {
            CsvBenchmark(CurrentBenchmarkScenario.Label), CsvBenchmark(CurrentBenchmarkScenario.Mode.ToString()), CsvBenchmark(CurrentBenchmarkRun.Trial.Label),
            CurrentBenchmarkRun.Repetition.ToString(CultureInfo.InvariantCulture), _benchmarkMeasuredFrames.ToString(CultureInfo.InvariantCulture), _benchmarkLegIndex.ToString(CultureInfo.InvariantCulture), _benchmarkSnapshot.Hash.ToString("X16", CultureInfo.InvariantCulture),
            snapshot.Overview.SolverNanoseconds.ToString(CultureInfo.InvariantCulture), snapshot.Overview.PairGenerationNanoseconds.ToString(CultureInfo.InvariantCulture), solver.TimestepContactSetBuildNanoseconds.ToString(CultureInfo.InvariantCulture), snapshot.Overview.IterationNanoseconds.ToString(CultureInfo.InvariantCulture), snapshot.Overview.SoftAvoidanceNanoseconds.ToString(CultureInfo.InvariantCulture),
            p.FullSweepSourceNanoseconds.ToString(CultureInfo.InvariantCulture), p.PersistentPairMappingNanoseconds.ToString(CultureInfo.InvariantCulture), p.ProxyValidationNanoseconds.ToString(CultureInfo.InvariantCulture), p.LocalBroadPhaseNanoseconds.ToString(CultureInfo.InvariantCulture), p.PairDiffNanoseconds.ToString(CultureInfo.InvariantCulture), p.SweptClassificationNanoseconds.ToString(CultureInfo.InvariantCulture), p.ContactActivationNanoseconds.ToString(CultureInfo.InvariantCulture), p.FallbackNanoseconds.ToString(CultureInfo.InvariantCulture),
            p.TopologyDirtyBodyCount.ToString(CultureInfo.InvariantCulture), p.MotionDirtyBodyCount.ToString(CultureInfo.InvariantCulture), p.PersistentNeighborPairCount.ToString(CultureInfo.InvariantCulture), p.CurrentInteractionPairCount.ToString(CultureInfo.InvariantCulture), p.CurrentSoftAvoidancePairCount.ToString(CultureInfo.InvariantCulture), p.SweptClassificationEvaluationCount.ToString(CultureInfo.InvariantCulture), p.ClassificationSkippedCount.ToString(CultureInfo.InvariantCulture), p.SoftAvoidancePairEvaluationCount.ToString(CultureInfo.InvariantCulture), p.ActiveConstraintEvaluationCount.ToString(CultureInfo.InvariantCulture), p.PersistentViewReuseCount.ToString(CultureInfo.InvariantCulture), p.PersistentViewRebuildCount.ToString(CultureInfo.InvariantCulture), p.InteractionEnvelopeEscapeCount.ToString(CultureInfo.InvariantCulture), p.FullRebuildCount.ToString(CultureInfo.InvariantCulture), p.IncrementalRepairCount.ToString(CultureInfo.InvariantCulture),
            snapshot.ContactSet.ContactSetSize.ToString(CultureInfo.InvariantCulture), snapshot.ContactSet.ActiveContactCount.ToString(CultureInfo.InvariantCulture), snapshot.ContactSet.PredictiveContactCount.ToString(CultureInfo.InvariantCulture), p.SoftAvoidanceOracleMissingPairCount.ToString(CultureInfo.InvariantCulture)
        }));
    }

    private void FinalizeScenarioBenchmark()
    {
        if (_benchmarkFinalized)
            return;
        _benchmarkFinalized = true;
        SimulationDebuggerRuntime.SetLocalRecordingCapture(false);
        _benchmarkRawWriter?.Dispose();
        _benchmarkRawWriter = null;
        if (_benchmarkResults.Count == 0)
            return;
        if (string.IsNullOrEmpty(_benchmarkDirectory))
            EnsureBenchmarkOutputWriter();
        _benchmarkRawWriter?.Dispose();
        _benchmarkRawWriter = null;
        string summaryPath = Path.Combine(_benchmarkDirectory, "adaptive_tuning_summary.csv");
        using var writer = new StreamWriter(summaryPath, false, new System.Text.UTF8Encoding(true)); // UTF-8 BOM
        writer.WriteLine(string.Join(",", new[]
        {
            "场景", "场景模式", "缓存配置", "重复", "单位数", "采样帧数", "基线哈希", "终态哈希",
            "均求解耗时ns", "均接触对生成耗时ns", "均接触集构建耗时ns", "均迭代投影耗时ns", "均软避让耗时ns",
            "均全量扫描耗时ns", "均持久对映射耗时ns", "均代理校验耗时ns", "均局部broadphase耗时ns", "均对差异耗时ns", "均分类耗时ns", "均接触激活耗时ns", "均回退耗时ns",
            "均拓扑脏体数", "均运动脏体数", "均持久邻居对数", "均交互对数", "均软避让对数",
            "均分类评估次数", "均分类跳过次数", "均软对评估次数", "均约束评估次数",
            "均持久视图复用", "均持久视图重建", "均交互包络逃逸", "均全量重建次数", "均增量修复次数",
            "均接触对数", "均活跃对数", "均预测对数", "均软避让漏检",
            "跨帧缓存", "跨子步缓存", "诊断", "Guard裕度", "子步数", "迭代数"
        }));
        foreach (BenchmarkResult result in _benchmarkResults)
            writer.WriteLine(result.ToCsv());
        File.WriteAllText(Path.Combine(_benchmarkDirectory, "adaptive_tuning_manifest.txt"),
            $"status={_benchmarkPhase}\nunit_count={UnitCount}\nrepetitions={BenchmarkRepetitions}\ncache_priming_frames={BenchmarkCachePrimingFrames}\nhold_frames={BenchmarkHoldFrames}\nframes_per_leg={BenchmarkFramesPerLeg}\n");
        Debug.Log($"[Benchmark] 输出目录: {_benchmarkDirectory}");
    }

    private void FailBenchmark(string reason)
    {
        if (_benchmarkPhase == BenchmarkPhase.Failed)
            return;
        _benchmarkPhase = BenchmarkPhase.Failed;
        Debug.LogError("[Benchmark] " + reason);
        FinalizeScenarioBenchmark();
    }

    private static string CsvBenchmark(string value) => "\"" + (value ?? string.Empty).Replace("\"", "\"\"") + "\"";
    private static string NumberBenchmark(float value) => value.ToString("F4", CultureInfo.InvariantCulture);
    private static string FormatBenchmark(float3 value) => $"({NumberBenchmark(value.x)}, {NumberBenchmark(value.y)}, {NumberBenchmark(value.z)})";

    public enum BenchmarkScenarioMode : byte { MoveThenHold, OpenPingPong, ObstaclePingPong }

    [Serializable]
    public sealed class BenchmarkScenario
    {
        [Tooltip("是否启用此场景参与本轮基准测试。")]
        public bool Enabled = true;
        [Tooltip("场景名称，会写入 CSV 的“场景”列。")]
        public string Label;
        [Tooltip("场景模式：MoveThenHold=移动到位后静止；OpenPingPong=开阔地往返；ObstaclePingPong=绕障碍往返。")]
        public BenchmarkScenarioMode Mode;
        [Tooltip("往返/到位的目标点 A（也是 MoveThenHold 的终点）。")]
        public float3 PointA;
        [Tooltip("往返的目标点 B（仅 PingPong 模式用）。")]
        public float3 PointB;
        [Tooltip("往返趟数。1 = A→B 一趟；2 = A→B→A 一个来回。RoundTripCount*2 = 总测量段数。")]
        [Min(1)] public int RoundTripCount = 1;
    }

    private readonly struct BenchmarkRun
    {
        public readonly ParameterTrial Trial;
        public readonly int Repetition;
        public BenchmarkRun(ParameterTrial trial, int repetition) { Trial = trial; Repetition = repetition; }
    }

    private struct BenchmarkUnitState
    {
        public Entity Entity;
        public LocalTransform Transform;
        public Velocity Velocity;
        public UnitMoveDestination Destination;
        public FlowArrivalState Arrival;
    }

    private sealed class ScenarioSnapshot
    {
        public List<BenchmarkUnitState> Units;
        public float3 GlobalTarget;
        public ulong Hash;
    }

    private sealed class BenchmarkAccumulator
    {
        private int _frames, _unitCount;
        private long _solver, _pairGeneration, _timestepContactSetBuild, _iteration, _softAvoid, _fullSweepSource, _persistentMap, _proxyValidation, _localBroadPhase, _pairDiff, _classification, _contactActivation, _fallback;
        private long _dirtyBodies, _motionDirtyBodies, _persistentPairs, _interactionPairs;
        private long _softAvoidancePairs, _classificationEvaluations, _classificationSkipped;
        private long _softPairEvaluations, _constraintEvaluations, _persistentViewReuse;
        private long _persistentViewRebuild, _interactionEnvelopeEscapes, _fullRebuilds;
        private long _incrementalRepairs, _contactPairs, _activePairs, _predictivePairs;
        private long _softOracleMissing;

        public void Reset()
        {
            _frames = _unitCount = 0;
            _solver = _pairGeneration = _timestepContactSetBuild = _iteration = _softAvoid = _fullSweepSource = _persistentMap = _proxyValidation = _localBroadPhase = _pairDiff = _classification = _contactActivation = _fallback = 0;
            _dirtyBodies = _motionDirtyBodies = _persistentPairs = _interactionPairs = 0;
            _softAvoidancePairs = _classificationEvaluations = _classificationSkipped = 0;
            _softPairEvaluations = _constraintEvaluations = _persistentViewReuse = 0;
            _persistentViewRebuild = _interactionEnvelopeEscapes = _fullRebuilds = 0;
            _incrementalRepairs = _contactPairs = _activePairs = _predictivePairs = 0;
            _softOracleMissing = 0;
        }

        public void Add(SimulationDebuggerFrameSnapshot snapshot)
        {
            _frames++;
            _unitCount = snapshot.Overview.UnitCount;
            _solver += snapshot.Overview.SolverNanoseconds;
            _pairGeneration += snapshot.Overview.PairGenerationNanoseconds;
            _timestepContactSetBuild += IncrementalContactPipelineDiagnosticsRuntime.Latest.SolverStatistics.TimestepContactSetBuildNanoseconds;
            _iteration += snapshot.Overview.IterationNanoseconds;
            _softAvoid += snapshot.Overview.SoftAvoidanceNanoseconds;
            _contactPairs += snapshot.ContactSet.ContactSetSize;
            _activePairs += snapshot.ContactSet.ActiveContactCount;
            _predictivePairs += snapshot.ContactSet.PredictiveContactCount;
            IncrementalContactPipelineStatistics p = IncrementalContactPipelineDiagnosticsRuntime.Latest.Statistics;
            _fullSweepSource += p.FullSweepSourceNanoseconds;
            _persistentMap += p.PersistentPairMappingNanoseconds;
            _proxyValidation += p.ProxyValidationNanoseconds;
            _localBroadPhase += p.LocalBroadPhaseNanoseconds;
            _pairDiff += p.PairDiffNanoseconds;
            _classification += p.SweptClassificationNanoseconds;
            _contactActivation += p.ContactActivationNanoseconds;
            _fallback += p.FallbackNanoseconds;
            _dirtyBodies += p.TopologyDirtyBodyCount;
            _motionDirtyBodies += p.MotionDirtyBodyCount;
            _persistentPairs += p.PersistentNeighborPairCount;
            _interactionPairs += p.CurrentInteractionPairCount;
            _softAvoidancePairs += p.CurrentSoftAvoidancePairCount;
            _classificationEvaluations += p.SweptClassificationEvaluationCount;
            _classificationSkipped += p.ClassificationSkippedCount;
            _softPairEvaluations += p.SoftAvoidancePairEvaluationCount;
            _constraintEvaluations += p.ActiveConstraintEvaluationCount;
            _persistentViewReuse += p.PersistentViewReuseCount;
            _persistentViewRebuild += p.PersistentViewRebuildCount;
            _interactionEnvelopeEscapes += p.InteractionEnvelopeEscapeCount;
            _fullRebuilds += p.FullRebuildCount;
            _incrementalRepairs += p.IncrementalRepairCount;
            _softOracleMissing += p.SoftAvoidanceOracleMissingPairCount;
        }

        public BenchmarkResult ToResult(BenchmarkScenario scenario, BenchmarkRun run, ulong baselineHash, ulong finalHash)
        {
            float inv = 1f / math.max(1, _frames);
            return new BenchmarkResult
            {
                Scenario = scenario.Label, Mode = scenario.Mode, Profile = run.Trial.Label, Repetition = run.Repetition,
                UnitCount = _unitCount, FrameCount = _frames, BaselineHash = baselineHash, FinalHash = finalHash,
                AverageSolverNs = (long)(_solver * inv), AveragePairGenerationNs = (long)(_pairGeneration * inv), AverageTimestepContactSetBuildNs = (long)(_timestepContactSetBuild * inv), AverageIterationNs = (long)(_iteration * inv), AverageSoftAvoidNs = (long)(_softAvoid * inv),
                AverageFullSweepSourceNs = (long)(_fullSweepSource * inv), AveragePersistentMapNs = (long)(_persistentMap * inv), AverageProxyValidationNs = (long)(_proxyValidation * inv), AverageLocalBroadPhaseNs = (long)(_localBroadPhase * inv), AveragePairDiffNs = (long)(_pairDiff * inv), AverageClassificationNs = (long)(_classification * inv), AverageContactActivationNs = (long)(_contactActivation * inv), AverageFallbackNs = (long)(_fallback * inv),
                AverageDirtyBodies = _dirtyBodies * inv, AverageMotionDirtyBodies = _motionDirtyBodies * inv, AveragePersistentPairs = _persistentPairs * inv, AverageInteractionPairs = _interactionPairs * inv,
                AverageSoftAvoidancePairs = _softAvoidancePairs * inv, AverageClassificationEvaluations = _classificationEvaluations * inv, AverageClassificationSkipped = _classificationSkipped * inv, AverageSoftPairEvaluations = _softPairEvaluations * inv, AverageConstraintEvaluations = _constraintEvaluations * inv,
                AveragePersistentViewReuse = _persistentViewReuse * inv, AveragePersistentViewRebuild = _persistentViewRebuild * inv, AverageInteractionEnvelopeEscapes = _interactionEnvelopeEscapes * inv, AverageFullRebuilds = _fullRebuilds * inv, AverageIncrementalRepairs = _incrementalRepairs * inv,
                AverageContactPairs = _contactPairs * inv, AverageActivePairs = _activePairs * inv, AveragePredictivePairs = _predictivePairs * inv,
                AverageSoftOracleMissing = _softOracleMissing * inv,
                CrossFrameCache = run.Trial.EnablePersistentContactCache, CrossSubstepCache = run.Trial.EnableTimestepContactSetCache,
                Diagnostics = run.Trial.EnableDiagnostics, GuardMargin = run.Trial.PersistentGuardEnvelopeMargin, Substeps = run.Trial.SubstepCount, Iterations = run.Trial.IterationCount
            };
        }
    }

    private sealed class BenchmarkResult
    {
        public string Scenario, Profile;
        public BenchmarkScenarioMode Mode;
        public int Repetition, UnitCount, FrameCount, Substeps, Iterations;
        public ulong BaselineHash, FinalHash;
        public long AverageSolverNs, AveragePairGenerationNs, AverageTimestepContactSetBuildNs, AverageIterationNs, AverageSoftAvoidNs, AverageFullSweepSourceNs, AveragePersistentMapNs, AverageProxyValidationNs, AverageLocalBroadPhaseNs, AveragePairDiffNs, AverageClassificationNs, AverageContactActivationNs, AverageFallbackNs;
        public float AverageDirtyBodies, AverageMotionDirtyBodies, AveragePersistentPairs, AverageInteractionPairs;
        public float AverageSoftAvoidancePairs, AverageClassificationEvaluations, AverageClassificationSkipped;
        public float AverageSoftPairEvaluations, AverageConstraintEvaluations, AveragePersistentViewReuse;
        public float AveragePersistentViewRebuild, AverageInteractionEnvelopeEscapes, AverageFullRebuilds;
        public float AverageIncrementalRepairs, AverageContactPairs, AverageActivePairs, AveragePredictivePairs;
        public float AverageSoftOracleMissing, GuardMargin;
        public byte CrossFrameCache, CrossSubstepCache, Diagnostics;

        public string ToCsv() => string.Join(",", new[]
        {
            CsvBenchmark(Scenario), CsvBenchmark(Mode.ToString()), CsvBenchmark(Profile), Repetition.ToString(CultureInfo.InvariantCulture), UnitCount.ToString(CultureInfo.InvariantCulture), FrameCount.ToString(CultureInfo.InvariantCulture),
            BaselineHash.ToString("X16", CultureInfo.InvariantCulture), FinalHash.ToString("X16", CultureInfo.InvariantCulture),
            AverageSolverNs.ToString(CultureInfo.InvariantCulture), AveragePairGenerationNs.ToString(CultureInfo.InvariantCulture), AverageTimestepContactSetBuildNs.ToString(CultureInfo.InvariantCulture), AverageIterationNs.ToString(CultureInfo.InvariantCulture), AverageSoftAvoidNs.ToString(CultureInfo.InvariantCulture), AverageFullSweepSourceNs.ToString(CultureInfo.InvariantCulture), AveragePersistentMapNs.ToString(CultureInfo.InvariantCulture), AverageProxyValidationNs.ToString(CultureInfo.InvariantCulture), AverageLocalBroadPhaseNs.ToString(CultureInfo.InvariantCulture), AveragePairDiffNs.ToString(CultureInfo.InvariantCulture), AverageClassificationNs.ToString(CultureInfo.InvariantCulture), AverageContactActivationNs.ToString(CultureInfo.InvariantCulture), AverageFallbackNs.ToString(CultureInfo.InvariantCulture),
            NumberBenchmark(AverageDirtyBodies), NumberBenchmark(AverageMotionDirtyBodies), NumberBenchmark(AveragePersistentPairs), NumberBenchmark(AverageInteractionPairs), NumberBenchmark(AverageSoftAvoidancePairs), NumberBenchmark(AverageClassificationEvaluations), NumberBenchmark(AverageClassificationSkipped), NumberBenchmark(AverageSoftPairEvaluations), NumberBenchmark(AverageConstraintEvaluations), NumberBenchmark(AveragePersistentViewReuse), NumberBenchmark(AveragePersistentViewRebuild), NumberBenchmark(AverageInteractionEnvelopeEscapes), NumberBenchmark(AverageFullRebuilds), NumberBenchmark(AverageIncrementalRepairs), NumberBenchmark(AverageContactPairs), NumberBenchmark(AverageActivePairs), NumberBenchmark(AveragePredictivePairs), NumberBenchmark(AverageSoftOracleMissing),
            CrossFrameCache.ToString(CultureInfo.InvariantCulture), CrossSubstepCache.ToString(CultureInfo.InvariantCulture), Diagnostics.ToString(CultureInfo.InvariantCulture), NumberBenchmark(GuardMargin), Substeps.ToString(CultureInfo.InvariantCulture), Iterations.ToString(CultureInfo.InvariantCulture)
        });
    }
}
}
