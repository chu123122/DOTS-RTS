using System.IO;
using System.Text;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.InputSystem;
using 客户端;
using 通用;
using RaycastHit = Unity.Physics.RaycastHit;

[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial class Stage3ContactDiagnosticVisualizationSystem : SystemBase
{
    private const float DrawHeight = 0.18f;
    private Stage3ContactDiagnosticOverlay _overlay;
    private readonly Stage3ContactDiagnosticCaptureSession _captureSession = new();
    private readonly Stage3ContactDiagnosticAutoCapture _automaticCapture = new();
    private bool _diagnosticsEnabledBeforeCapture;

    protected override void OnCreate()
    {
        RequireForUpdate<UnitContactSolverSettings>();
        RequireForUpdate<FlowFieldSettings>();
        RequireForUpdate<PredictiveDiscContactStatistics>();
        RequireForUpdate<ShadowNeighborCacheStatistics>();
        RequireForUpdate<Stage3ContactDiagnosticSelection>();
        RequireForUpdate<Stage3SelectedBodyDiagnostic>();
        RequireForUpdate<PhysicsWorldSingleton>();
        RequireForUpdate<MainCameraTag>();

        var overlayObject = new GameObject($"Stage3 Contact Diagnostic ({World.Name})")
        {
            hideFlags = HideFlags.DontSave
        };
        Object.DontDestroyOnLoad(overlayObject);
        _overlay = overlayObject.AddComponent<Stage3ContactDiagnosticOverlay>();
    }

    protected override void OnDestroy()
    {
        if (_captureSession.Active)
            WriteCaptureFile();
        if (_overlay != null)
            Object.Destroy(_overlay.gameObject);
    }

    protected override void OnUpdate()
    {
        RefRW<UnitContactSolverSettings> settingsReference =
            SystemAPI.GetSingletonRW<UnitContactSolverSettings>();
        UnitContactSolverSettings settings = settingsReference.ValueRO;
        FlowFieldSettings flowSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        double simulationTime = SystemAPI.Time.ElapsedTime;

        Keyboard keyboard = Keyboard.current;
        bool automaticCaptureKeyPressed =
            keyboard != null &&
            keyboard.f12Key.wasPressedThisFrame &&
            !keyboard.leftCtrlKey.isPressed &&
            !keyboard.rightCtrlKey.isPressed &&
            !keyboard.leftShiftKey.isPressed &&
            !keyboard.rightShiftKey.isPressed &&
            !keyboard.leftAltKey.isPressed &&
            !keyboard.rightAltKey.isPressed;
        if (automaticCaptureKeyPressed)
        {
            if (_automaticCapture.Active)
            {
                if (_captureSession.Active)
                    FinishCapture(ref settings);
                _automaticCapture.Cancel(ref settings);
                Debug.Log("Stage 3 automatic Fat AABB experiment cancelled; original settings restored.");
            }
            else if (!_captureSession.Active)
            {
                _automaticCapture.Start(ref settings, simulationTime);
                Debug.Log(
                    $"Stage 3 automatic Fat AABB experiment started: " +
                    $"runs={_automaticCapture.TotalRuns}, sequence=OFF->ON->OFF, " +
                    "configurations=1x8,2x4,4x2");
            }
        }
        if (!_automaticCapture.Active &&
            keyboard != null && keyboard.f6Key.wasPressedThisFrame)
        {
            if (_captureSession.Active)
                FinishCapture(ref settings);
            else
                StartCapture(ref settings, simulationTime);
        }
        if (!_automaticCapture.Active && keyboard != null && keyboard.f7Key.wasPressedThisFrame)
            settings.EnablePredictivePairGeneration = !settings.EnablePredictivePairGeneration;
        if (!_automaticCapture.Active && keyboard != null && keyboard.f8Key.wasPressedThisFrame)
            settings.EnableDiagnostics = !settings.EnableDiagnostics;
        if (!_automaticCapture.Active && keyboard != null && keyboard.f9Key.wasPressedThisFrame)
            settings.EnablePredictiveContacts = !settings.EnablePredictiveContacts;
        if (!_automaticCapture.Active && keyboard != null && keyboard.f10Key.wasPressedThisFrame)
            settings.VisualizeSelectedContacts = !settings.VisualizeSelectedContacts;
        if (!_automaticCapture.Active && keyboard != null && keyboard.f11Key.wasPressedThisFrame)
            settings.EnableFatAabbCache = !settings.EnableFatAabbCache;
        if (keyboard != null && keyboard.pageUpKey.wasPressedThisFrame)
            _overlay.Scale = math.min(2f, _overlay.Scale + 0.1f);
        if (keyboard != null && keyboard.pageDownKey.wasPressedThisFrame)
            _overlay.Scale = math.max(0.8f, _overlay.Scale - 0.1f);

        bool automaticExperimentCompleted;
        if (_automaticCapture.Tick(
                ref settings,
                simulationTime,
                _captureSession.Active,
                out string automaticRunLabel,
                out automaticExperimentCompleted))
        {
            StartCapture(ref settings, simulationTime, automaticRunLabel);
        }
        if (automaticExperimentCompleted)
        {
            Debug.Log(
                "Stage 3 automatic Fat AABB experiment completed; " +
                "original settings restored.");
        }
        if (_captureSession.Active)
            settings.EnableDiagnostics = true;
        settingsReference.ValueRW = settings;

        if (settings.EnableDiagnostics && settings.VisualizeSelectedContacts)
            TrySelectDiagnosticUnitWithMiddleMouse();

        bool shouldShowOverlay =
            settings.EnableDiagnostics || settings.EnableFatAabbCache;
        _overlay.Visible = shouldShowOverlay;
        if (!shouldShowOverlay)
        {
            _overlay.HeaderText = string.Empty;
            _overlay.Stage3Text = string.Empty;
            _overlay.ShadowText = string.Empty;
            return;
        }

        // Singleton/Buffer API 不会自动完成生产它们的异步 Job。
        // Presentation 需要本帧最终诊断快照，因此在读取前建立明确同步边界。
        EntityManager.CompleteDependencyBeforeRO<PredictiveDiscContactStatistics>();
        EntityManager.CompleteDependencyBeforeRO<ShadowNeighborCacheStatistics>();
        EntityManager.CompleteDependencyBeforeRO<Stage3ContactIterationDiagnostic>();
        EntityManager.CompleteDependencyBeforeRO<Stage3ContactPairDiagnostic>();
        EntityManager.CompleteDependencyBeforeRO<Stage3SelectedBodyDiagnostic>();

        PredictiveDiscContactStatistics statistics =
            SystemAPI.GetSingleton<PredictiveDiscContactStatistics>();
        ShadowNeighborCacheStatistics shadowStatistics =
            SystemAPI.GetSingleton<ShadowNeighborCacheStatistics>();
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDiagnostics =
            SystemAPI.GetSingletonBuffer<Stage3ContactIterationDiagnostic>(true);
        DynamicBuffer<Stage3ContactPairDiagnostic> pairDiagnostics =
            SystemAPI.GetSingletonBuffer<Stage3ContactPairDiagnostic>(true);
        Stage3SelectedBodyDiagnostic selectedBody =
            SystemAPI.GetSingleton<Stage3SelectedBodyDiagnostic>();
        Stage3ContactDiagnosticSelection selection =
            SystemAPI.GetSingleton<Stage3ContactDiagnosticSelection>();

        UpdateCapture(
            ref settings,
            simulationTime,
            flowSettings,
            statistics,
            shadowStatistics,
            iterationDiagnostics);
        settingsReference.ValueRW = settings;

        _overlay.HeaderText = BuildHeaderText(
            settings,
            _overlay.Scale,
            _captureSession,
            _automaticCapture,
            simulationTime);
        _overlay.Stage3Text = BuildStage3PanelText(
            settings,
            flowSettings,
            statistics,
            iterationDiagnostics,
            pairDiagnostics,
            selection.SelectedEntity,
            selectedBody);
        _overlay.ShadowText = BuildShadowPanelText(
            settings,
            statistics,
            shadowStatistics);

        if (!settings.EnableDiagnostics ||
            !settings.VisualizeSelectedContacts ||
            selectedBody.IsValid == 0)
            return;

        DrawSelectedSweep(selectedBody);
        for (int i = 0; i < pairDiagnostics.Length; i++)
            DrawPair(pairDiagnostics[i]);
    }

    private void TrySelectDiagnosticUnitWithMiddleMouse()
    {
        Mouse mouse = Mouse.current;
        if (mouse == null || !mouse.middleButton.wasPressedThisFrame)
            return;

        Entity cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
        Camera camera = EntityManager.GetComponentObject<MainCameraComponents>(cameraEntity).Value;
        if (camera == null)
            return;

        UnityEngine.Ray ray = camera.ScreenPointToRay(mouse.position.ReadValue());
        var raycastInput = new RaycastInput
        {
            Start = ray.origin,
            End = ray.origin + ray.direction * 1000f,
            Filter = new CollisionFilter
            {
                BelongsTo = ~0u,
                CollidesWith = 1u << 1,
                GroupIndex = 0
            }
        };

        CollisionWorld collisionWorld =
            SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
        if (!collisionWorld.CastRay(raycastInput, out RaycastHit hit) ||
            !EntityManager.HasComponent<BasicUnitTag>(hit.Entity))
            return;

        RefRW<Stage3ContactDiagnosticSelection> selection =
            SystemAPI.GetSingletonRW<Stage3ContactDiagnosticSelection>();
        selection.ValueRW.SelectedEntity = hit.Entity;
    }

    private static void DrawSelectedSweep(Stage3SelectedBodyDiagnostic selected)
    {
        float3 height = new float3(0, DrawHeight, 0);
        float3 start = selected.StartPosition + height;
        float3 predicted = selected.UnconstrainedPredictedPosition + height;
        float3 solved = selected.SolvedPosition + height;
        float sweptRadius = selected.Radius + selected.Skin;

        DrawWireCircle(start, selected.Radius, Color.cyan);
        DrawWireCircle(predicted, selected.Radius, Color.blue);
        DrawWireCircle(solved, selected.Radius, Color.green);
        DrawSweptCapsule(start, predicted, sweptRadius, new Color(0f, 1f, 1f));
        DrawSweptAabb(start, predicted, sweptRadius, Color.white);
        if (selected.ShadowReferenceAvailable != 0)
        {
            DrawAabb(
                selected.ShadowFatMin,
                selected.ShadowFatMax,
                start.y + 0.02f,
                selected.ShadowEscaped != 0 ? Color.red : Color.yellow);
        }

        Debug.DrawLine(start, predicted, Color.blue, 0f, false);
        Debug.DrawLine(predicted, solved, Color.green, 0f, false);
        if (math.lengthsq(selected.WallCorrection) > 0.0000001f)
        {
            Debug.DrawLine(
                solved - selected.WallCorrection,
                solved,
                Color.white,
                0f,
                false);
        }
    }

    private static void DrawAabb(float2 min, float2 max, float height, Color color)
    {
        float3 a = new float3(min.x, height, min.y);
        float3 b = new float3(max.x, height, min.y);
        float3 c = new float3(max.x, height, max.y);
        float3 d = new float3(min.x, height, max.y);
        Debug.DrawLine(a, b, color, 0f, false);
        Debug.DrawLine(b, c, color, 0f, false);
        Debug.DrawLine(c, d, color, 0f, false);
        Debug.DrawLine(d, a, color, 0f, false);
    }

    private static void DrawPair(Stage3ContactPairDiagnostic pair)
    {
        Color color = GetPairColor(pair.Kind, pair.WasActivated != 0);
        float3 height = new float3(0, DrawHeight + 0.02f, 0);
        float3 otherStart = pair.OtherStartPosition + height;
        float3 otherPredicted = pair.OtherPredictedPosition + height;
        float3 selectedClosest = pair.SelectedClosestPosition + height;
        float3 otherClosest = pair.OtherClosestPosition + height;

        Debug.DrawLine(otherStart, otherPredicted, color, 0f, false);
        Debug.DrawLine(selectedClosest, otherClosest, color, 0f, false);
        DrawWireCircle(otherPredicted, pair.OtherRadius, color, 16);
        DrawCross(selectedClosest, color, 0.06f);
        DrawCross(otherClosest, color, 0.06f);
    }

    private static Color GetPairColor(
        Stage3ContactDiagnosticPairKind kind,
        bool activated)
    {
        switch (kind)
        {
            case Stage3ContactDiagnosticPairKind.BroadPhaseRejected:
                return Color.gray;
            case Stage3ContactDiagnosticPairKind.Regular:
                return activated ? new Color(1f, 0.45f, 0f) : Color.yellow;
            case Stage3ContactDiagnosticPairKind.Predictive:
                return activated ? Color.red : Color.magenta;
            case Stage3ContactDiagnosticPairKind.PredictiveDisabled:
                return new Color(0.25f, 0.55f, 1f);
            default:
                return Color.white;
        }
    }

    private static void DrawSweptCapsule(
        float3 start,
        float3 end,
        float radius,
        Color color)
    {
        float2 direction = end.xz - start.xz;
        float lengthSq = math.lengthsq(direction);
        if (lengthSq <= 0.0000001f)
        {
            DrawWireCircle(start, radius, color);
            return;
        }

        float2 perpendicular = math.normalize(new float2(-direction.y, direction.x)) * radius;
        float3 offset = new float3(perpendicular.x, 0, perpendicular.y);
        Debug.DrawLine(start + offset, end + offset, color, 0f, false);
        Debug.DrawLine(start - offset, end - offset, color, 0f, false);
        DrawWireCircle(start, radius, color);
        DrawWireCircle(end, radius, color);
    }

    private static void DrawSweptAabb(
        float3 start,
        float3 end,
        float radius,
        Color color)
    {
        float2 minimum = math.min(start.xz, end.xz) - radius;
        float2 maximum = math.max(start.xz, end.xz) + radius;
        float y = start.y + 0.01f;
        float3 a = new float3(minimum.x, y, minimum.y);
        float3 b = new float3(maximum.x, y, minimum.y);
        float3 c = new float3(maximum.x, y, maximum.y);
        float3 d = new float3(minimum.x, y, maximum.y);
        Debug.DrawLine(a, b, color, 0f, false);
        Debug.DrawLine(b, c, color, 0f, false);
        Debug.DrawLine(c, d, color, 0f, false);
        Debug.DrawLine(d, a, color, 0f, false);
    }

    private static void DrawWireCircle(
        float3 center,
        float radius,
        Color color,
        int segments = 24)
    {
        float angleStep = math.PI * 2f / segments;
        float3 previous = center + new float3(radius, 0, 0);
        for (int i = 1; i <= segments; i++)
        {
            float angle = angleStep * i;
            float3 next = center + new float3(math.cos(angle) * radius, 0, math.sin(angle) * radius);
            Debug.DrawLine(previous, next, color, 0f, false);
            previous = next;
        }
    }

    private static void DrawCross(float3 center, Color color, float radius)
    {
        Debug.DrawLine(
            center - new float3(radius, 0, 0),
            center + new float3(radius, 0, 0),
            color,
            0f,
            false);
        Debug.DrawLine(
            center - new float3(0, 0, radius),
            center + new float3(0, 0, radius),
            color,
            0f,
            false);
    }

    private void StartCapture(
        ref UnitContactSolverSettings settings,
        double simulationTime,
        string runLabel = "")
    {
        _diagnosticsEnabledBeforeCapture = settings.EnableDiagnostics;
        settings.EnableDiagnostics = true;
        _captureSession.Start(
            simulationTime,
            settings.DiagnosticCaptureDuration,
            settings.DiagnosticCaptureInterval,
            runLabel);
        Debug.Log(
            $"Stage 3 diagnostic capture started: label={runLabel}, " +
            $"duration={_captureSession.Duration:F1}s, " +
            $"interval={_captureSession.Interval:F2}s");
    }

    private void UpdateCapture(
        ref UnitContactSolverSettings settings,
        double simulationTime,
        FlowFieldSettings flowSettings,
        PredictiveDiscContactStatistics statistics,
        ShadowNeighborCacheStatistics shadowStatistics,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDiagnostics)
    {
        if (!_captureSession.Active)
            return;

        if (_captureSession.ShouldSample(simulationTime))
        {
            _captureSession.AddSample(
                simulationTime,
                settings,
                flowSettings,
                statistics,
                shadowStatistics,
                iterationDiagnostics);
        }

        if (_captureSession.ReachedLimit(simulationTime))
            FinishCapture(ref settings);
    }

    private void FinishCapture(ref UnitContactSolverSettings settings)
    {
        if (!_captureSession.Active)
            return;

        WriteCaptureFile();
        settings.EnableDiagnostics = _diagnosticsEnabledBeforeCapture;
    }

    private void WriteCaptureFile()
    {
        try
        {
            string outputPath = _captureSession.StopAndWrite();
            Debug.Log($"Stage 3 diagnostic capture saved: {outputPath}");
        }
        catch (System.Exception exception)
        {
            Debug.LogException(exception);
        }
    }

    private static string BuildHeaderText(
        UnitContactSolverSettings settings,
        float overlayScale,
        Stage3ContactDiagnosticCaptureSession captureSession,
        Stage3ContactDiagnosticAutoCapture automaticCapture,
        double simulationTime)
    {
        var text = new StringBuilder(256);
        text.Append("<size=19><b>单位接触诊断</b></size>   ")
            .Append("<color=#92A3B8>F6 JSON采集</color> ")
            .Append(ToggleText(captureSession.Active)).Append("   ")
            .Append("<color=#92A3B8>F7 预测生成</color> ").Append(ToggleText(settings.EnablePredictivePairGeneration))
            .Append("   ")
            .Append("<color=#92A3B8>F8 数据</color> ").Append(ToggleText(settings.EnableDiagnostics))
            .Append("   <color=#92A3B8>F9 防换侧约束</color> ").Append(ToggleText(settings.EnablePredictiveContacts))
            .Append("   <color=#92A3B8>F10 场景线框</color> ").Append(ToggleText(settings.VisualizeSelectedContacts))
            .Append("   <color=#92A3B8>F11 Fat缓存</color> ").Append(ToggleText(settings.EnableFatAabbCache))
            .Append("   <color=#92A3B8>F12 自动对照</color> ").Append(ToggleText(automaticCapture.Active))
            .AppendLine()
            .Append("<color=#74849A>PageUp / PageDown 调整面板：")
            .Append(overlayScale.ToString("F1")).Append("x　·　中键选择单位</color>");
        if (captureSession.Active)
        {
            text.Append("　<color=#FFC857>采集中 ")
                .Append(captureSession.GetElapsed(simulationTime).ToString("F1"))
                .Append("/").Append(captureSession.Duration.ToString("F1"))
                .Append("s　间隔 ").Append(captureSession.Interval.ToString("F2"))
                .Append("s　样本 ").Append(captureSession.SampleCount)
                .Append("</color>");
        }
        else if (automaticCapture.Active)
        {
            text.Append("　<color=#FFC857>自动实验 ")
                .Append(automaticCapture.CurrentRunNumber).Append("/")
                .Append(automaticCapture.TotalRuns).Append("　")
                .Append(automaticCapture.CurrentRunLabel)
                .Append("　预热 ")
                .Append(automaticCapture.GetWarmupRemaining(simulationTime).ToString("F1"))
                .Append("s</color>");
        }
        else if (!string.IsNullOrEmpty(captureSession.LastOutputPath))
        {
            text.Append("　<color=#69E39B>已保存 ")
                .Append(Path.GetFileName(captureSession.LastOutputPath))
                .Append("</color>");
        }
        return text.ToString();
    }

    private static string BuildStage3PanelText(
        UnitContactSolverSettings settings,
        FlowFieldSettings flowSettings,
        PredictiveDiscContactStatistics statistics,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterations,
        DynamicBuffer<Stage3ContactPairDiagnostic> selectedPairs,
        Entity selectedEntity,
        Stage3SelectedBodyDiagnostic selectedBody)
    {
        int broadOnly = 0;
        int regular = 0;
        int predictive = 0;
        int predictiveDisabled = 0;
        for (int i = 0; i < selectedPairs.Length; i++)
        {
            switch (selectedPairs[i].Kind)
            {
                case Stage3ContactDiagnosticPairKind.BroadPhaseRejected:
                    broadOnly++;
                    break;
                case Stage3ContactDiagnosticPairKind.Regular:
                    regular++;
                    break;
                case Stage3ContactDiagnosticPairKind.Predictive:
                    predictive++;
                    break;
                case Stage3ContactDiagnosticPairKind.PredictiveDisabled:
                    predictiveDisabled++;
                    break;
            }
        }

        var residualCurve = new StringBuilder(192);
        float firstResidual = 0f;
        float lastResidual = 0f;
        float lastAverageResidual = 0f;
        int residualSamples = 0;
        if (iterations.Length > 0)
        {
            Stage3ContactIterationDiagnostic last = iterations[iterations.Length - 1];
            lastResidual = last.MaxConstraintViolation;
            lastAverageResidual = last.AverageConstraintViolation;
            for (int i = 0; i < iterations.Length && residualSamples < 10; i++)
            {
                Stage3ContactIterationDiagnostic iteration = iterations[i];
                if (iteration.SubstepIndex != last.SubstepIndex)
                    continue;
                if (residualSamples == 0)
                    firstResidual = iteration.MaxConstraintViolationBeforeSolve;
                else
                    residualCurve.Append(" <color=#607086>›</color> ");
                residualCurve.Append(FormatResidual(iteration.MaxConstraintViolationBeforeSolve));
                residualSamples++;
            }
            if (residualSamples < settings.IterationCount)
                residualCurve.Append(" …");
            residualCurve.Append(" <color=#607086>› 最终</color> ")
                .Append(FormatResidual(lastResidual));
        }

        string state;
        if (statistics.ActiveConstraintCount == 0 &&
            statistics.TotalWallPositionCorrection <= 0.000001f)
        {
            state = StatusText("空闲样本：本帧没有有效约束", "#91A0B4");
        }
        else if (residualSamples > 1 && lastResidual > firstResidual + 0.00001f)
        {
            state = StatusText("注意：单位接触残差正在上升", "#FF6B78");
        }
        else if (statistics.MaxPenetration > 0.0001f)
        {
            state = StatusText("求解已生效，但仍有剩余穿透", "#FFC857");
        }
        else
        {
            state = StatusText("当前约束结果稳定", "#69E39B");
        }

        var text = new StringBuilder(1024);
        text.AppendLine("<size=18><color=#62D8FF><b>Stage 3 · 权威接触求解</b></color></size>")
            .AppendLine("<color=#71839A>这些数据真实参与单位的位置和速度结果</color>")
            .Append("<color=#91A0B7>状态　</color>").AppendLine(state)
            .Append("<color=#91A0B7>配置　</color>子步 <b>").Append(settings.SubstepCount)
            .Append("</b>　迭代 <b>").Append(settings.IterationCount)
            .Append("</b>　软避让重算 <b>").Append(statistics.SoftAvoidanceEvaluationCount)
            .AppendLine("</b>")
            .Append("<color=#91A0B7>速度策略　</color><b>")
            .Append(flowSettings.SoftAvoidanceVelocitySolver)
            .Append("</b>　Shell ").Append(flowSettings.SoftAvoidanceShell.ToString("F2"))
            .Append("　Horizon ").Append(flowSettings.RvoTimeHorizon.ToString("F2"))
            .AppendLine("s")
            .Append("<color=#91A0B7>软避让候选　</color>检查 <b>")
            .Append(statistics.SoftAvoidanceCandidatePairCount)
            .Append("</b>　激活 <b>").Append(statistics.SoftAvoidanceActivatedPairCount)
            .Append("</b>　Fat 复用 <b>").Append(statistics.SoftAvoidanceFatAabbUseCount)
            .AppendLine("</b>")
            .Append("<color=#91A0B7>Pair 漏斗　</color><color=#AFC4D8>候选</color> <b>")
            .Append(statistics.CandidatePairCount)
            .Append("</b>　<color=#607086>→</color>　<color=#7AD7F0>接触</color> <b>")
            .Append(statistics.ContactPairCount)
            .Append("</b>　<color=#607086>→</color>　<color=#69E39B>激活</color> <b>")
            .Append(statistics.ActiveConstraintCount).AppendLine("</b>")
            .Append("<color=#91A0B7>Pair 来源　</color>实际生成 <b>").Append(statistics.ActualGeneratedPairCount)
            .Append("</b>　预测生成 <color=#D58CFF><b>").Append(statistics.PredictiveGeneratedPairCount)
            .AppendLine("</b></color>")
            .Append("<color=#91A0B7>防换侧约束　</color>风险 <b>").Append(statistics.PotentialPredictivePairCount)
            .Append("</b>　启用 <b>").Append(statistics.PredictivePairCount)
            .Append("</b>　激活 <color=#FF7BA8><b>").Append(statistics.PredictiveActivatedCount)
            .AppendLine("</b></color>")
            .Append("<color=#91A0B7>最终穿透　</color>最大 <b>").Append(statistics.MaxPenetration.ToString("F5"))
            .Append("</b>　平均 ").AppendLine(statistics.AveragePenetration.ToString("F5"))
            .Append("<color=#91A0B7>单位修正　</color>累计 ").Append(statistics.TotalContactPositionCorrection.ToString("F4"))
            .Append("　单次最大 <b>").Append(statistics.MaxContactPositionCorrection.ToString("F4")).AppendLine("</b>")
            .Append("<color=#91A0B7>墙壁修正　</color>累计 ").Append(statistics.TotalWallPositionCorrection.ToString("F4"))
            .Append("　单次最大 <b>").Append(statistics.MaxWallPositionCorrection.ToString("F4")).AppendLine("</b>")
            .Append("<color=#91A0B7>平均速度　</color>").Append(statistics.AverageSpeedBeforeContact.ToString("F3"))
            .Append(" <color=#607086>→</color> ").Append(statistics.AverageSpeedAfterContact.ToString("F3"))
            .Append("　最大变化 <b>").Append(statistics.MaxVelocityChange.ToString("F3")).AppendLine("</b>")
            .Append("<color=#91A0B7>最终残差　</color>最大 <b>").Append(FormatResidual(lastResidual))
            .Append("</b>　平均 ").AppendLine(FormatResidual(lastAverageResidual))
            .Append("<color=#91A0B7>收敛曲线（轮前→最终）　</color>")
            .AppendLine(residualSamples > 0 ? residualCurve.ToString() : "<color=#607086>暂无样本</color>")
            .Append("<color=#91A0B7>耗时 μs　</color>软避让 ").Append(FormatMicroseconds(statistics.SoftAvoidanceNanoseconds))
            .Append("（单子步 ").Append(FormatMicroseconds(statistics.AverageSoftAvoidanceNanoseconds)).Append("）　Pair ")
            .Append(FormatMicroseconds(statistics.PairGenerationNanoseconds))
            .Append("　单轮 ").Append(FormatMicroseconds(statistics.AverageIterationNanoseconds))
            .Append("　总计 <b>").Append(FormatMicroseconds(statistics.SolverNanoseconds)).AppendLine("</b>")
            .Append("<color=#91A0B7>选中单位　</color>")
            .Append(selectedEntity == Entity.Null
                ? "<color=#607086>未选择</color>"
                : $"<b>{selectedEntity.Index}:{selectedEntity.Version}</b>")
            .Append("　显示 Pair <b>").Append(selectedPairs.Length).AppendLine("</b>")
            .Append("<color=#91A0B7>Pair 分类　</color>宽相排除 ").Append(broadOnly)
            .Append("　径向 ").Append(regular).Append("　防换侧 ").Append(predictive)
            .Append("　防换侧关闭 ").AppendLine(predictiveDisabled.ToString());

        if (selectedBody.IsValid != 0)
        {
            text.Append("<color=#91A0B7>选中单位修正　</color>接触 ")
                .Append(math.length(selectedBody.ContactCorrection).ToString("F4"))
                .Append("　墙壁 ")
                .AppendLine(math.length(selectedBody.WallCorrection).ToString("F4"));
        }

        text.Append("<color=#607086>场景颜色：黄色/橙色径向 Pair　洋红/红色防换侧 Pair　蓝色防换侧关闭</color>");
        return text.ToString();
    }

    private static string BuildShadowPanelText(
        UnitContactSolverSettings settings,
        PredictiveDiscContactStatistics statistics,
        ShadowNeighborCacheStatistics shadow)
    {
        if (!settings.EnableFatAabbCache)
        {
            return "<size=18><color=#C99BFF><b>Fat AABB · Broad Phase 缓存</b></color></size>\n" +
                   "<color=#71839A>缓存候选 Entity Pair；每个子步仍重新执行 Narrow Phase</color>\n\n" +
                   StatusText("当前未启用", "#91A0B4") + "\n\n" +
                   "<color=#91A0B7>按 <b>F11</b> 启用；首次使用会从当前状态完整重建。</color>";
        }

        string state;
        if (shadow.FullBroadPhaseFallbackCount > 0)
            state = StatusText("Fat AABB 失效，已安全回退完整 Broad Phase", "#FFC857");
        else if (shadow.CacheRebuildCount > 0)
            state = StatusText("本帧已重建缓存", "#69E39B");
        else if (shadow.CacheReuseCount > 0)
            state = StatusText("正在复用缓存", "#69E39B");
        else
            state = StatusText("等待缓存样本", "#91A0B4");

        float averageContacts =
            (float)statistics.ContactPairCount / math.max(1, settings.SubstepCount);
        string inflation = averageContacts > 0.0001f
            ? (shadow.CachedCandidatePairCount / averageContacts).ToString("F2") + "x"
            : "--";

        var text = new StringBuilder(1024);
        text.AppendLine("<size=18><color=#C99BFF><b>Fat AABB · Broad Phase 缓存</b></color></size>")
            .AppendLine("<color=#71839A>缓存只替代候选发现；接触分类和 XPBD 求解保持实时</color>")
            .Append("<color=#A999BA>状态　</color>").AppendLine(state)
            .Append("<color=#A999BA>额外边界　</color><b>").Append(settings.FatAabbCacheMargin.ToString("F3"))
            .Append("</b>　候选/接触膨胀 <b>").Append(inflation).AppendLine("</b>")
            .AppendLine("<color=#BBA6D0><b>持久缓存状态</b></color>")
            .Append("<color=#A999BA>有效性　</color>帧首 ").Append(shadow.CacheValidAtFrameStart)
            .Append("　帧末 <b>").Append(shadow.CacheValidAtFrameEnd)
            .Append("</b>　年龄 <b>").Append(shadow.CacheAgeFrames).AppendLine(" 帧</b>")
            .Append("<color=#A999BA>规模　</color>单位 ").Append(shadow.CurrentFrameCacheBodyCount)
            .Append("　候选 Pair <b>").Append(shadow.CachedCandidatePairCount)
            .Append("</b>　Narrow 检查 ").AppendLine(shadow.CachedNarrowPhasePairCheckCount.ToString())
            .AppendLine("<color=#BBA6D0><b>本帧操作</b></color>")
            .Append("<color=#A999BA>使用　</color>").Append(shadow.CacheUseCount)
            .Append("　复用 <color=#69E39B><b>").Append(shadow.CacheReuseCount)
            .Append("</b></color>　重建 <b>").Append(shadow.CacheRebuildCount)
            .Append("</b>　完整回退 <color=#FFC857><b>").Append(shadow.FullBroadPhaseFallbackCount)
            .AppendLine("</b></color>")
            .Append("<color=#A999BA>Pair 映射　</color>构建 ").Append(shadow.CachePairMappingBuildCount)
            .Append("　帧内复用 <b>").Append(shadow.CachePairMappingReuseCount)
            .Append("</b>　修正单位检查 <b>").Append(shadow.CorrectedBodyValidationCount)
            .AppendLine("</b>")
            .Append("<color=#A999BA>失效　</color>总计 ").Append(shadow.CacheInvalidationCount)
            .Append("　Entity集合 ").Append(shadow.EntitySetInvalidationCount)
            .Append("　边界 ").Append(shadow.BoundsInvalidationCount)
            .Append("　求解后逃逸 <b>").Append(shadow.PostSolveInvalidationCount).AppendLine("</b>")
            .Append("<color=#A999BA>耗时 μs　</color>构建 ")
            .Append(FormatMicroseconds(shadow.CacheBuildNanoseconds))
            .Append("　验证 ").Append(FormatMicroseconds(shadow.ValidationNanoseconds))
            .Append("　映射 <b>").Append(FormatMicroseconds(shadow.CachePairMappingNanoseconds)).AppendLine("</b>")
            .Append("<color=#607086>缓存失效会自动回退完整 Broad Phase；重点观察复用/重建比例与候选膨胀。</color>");
        return text.ToString();
    }

    private static string ToggleText(bool enabled)
    {
        return enabled
            ? "<color=#69E39B><b>开启</b></color>"
            : "<color=#758397>关闭</color>";
    }

    private static string StatusText(string value, string color)
    {
        return $"<color={color}><b>{value}</b></color>";
    }

    private static string FormatMicroseconds(long nanoseconds)
    {
        return (nanoseconds / 1000f).ToString("F1");
    }

    private static string FormatResidual(float value)
    {
        float absolute = math.abs(value);
        if (absolute <= 0f)
            return "0";
        return absolute < 0.0001f
            ? value.ToString("0.00E+0")
            : value.ToString("F5");
    }

    private static string FormatCoverage(int hits, int misses)
    {
        int total = hits + misses;
        return total > 0
            ? ((float)hits / total * 100f).ToString("F1") + "%"
            : "--";
    }
}

public sealed class Stage3ContactDiagnosticOverlay : MonoBehaviour
{
    public bool Visible;
    public string HeaderText = string.Empty;
    public string Stage3Text = string.Empty;
    public string ShadowText = string.Empty;
    public float Scale = 1.25f;

    private GUIStyle _headerStyle;
    private GUIStyle _panelStyle;

    private void OnGUI()
    {
        if (!Visible)
            return;

        _headerStyle ??= new GUIStyle(GUI.skin.label)
        {
            richText = true,
            wordWrap = true,
            fontSize = 16,
            normal = { textColor = new Color(0.92f, 0.96f, 1f) }
        };
        _panelStyle ??= new GUIStyle(GUI.skin.label)
        {
            richText = true,
            wordWrap = true,
            fontSize = 16,
            normal = { textColor = new Color(0.9f, 0.94f, 0.98f) }
        };

        const float cardWidth = 510f;
        const float gap = 12f;
        const float logicalWidth = cardWidth * 2f + gap;
        float headerHeight = math.max(
            58f,
            _headerStyle.CalcHeight(new GUIContent(HeaderText), logicalWidth - 28f) + 20f);
        float stage3Height = _panelStyle.CalcHeight(
            new GUIContent(Stage3Text),
            cardWidth - 32f) + 28f;
        float shadowHeight = _panelStyle.CalcHeight(
            new GUIContent(ShadowText),
            cardWidth - 32f) + 28f;
        float cardHeight = math.max(390f, math.max(stage3Height, shadowHeight));
        float logicalHeight = headerHeight + gap + cardHeight;
        float fitScale = math.min(
            (Screen.width - 24f) / logicalWidth,
            (Screen.height - 24f) / logicalHeight);
        float effectiveScale = math.clamp(math.min(Scale, fitScale), 0.5f, 2f);
        Matrix4x4 previousMatrix = GUI.matrix;
        GUI.matrix = Matrix4x4.Scale(new Vector3(effectiveScale, effectiveScale, 1f));

        var headerRect = new Rect(12f, 12f, logicalWidth, headerHeight);
        DrawCardBackground(
            headerRect,
            new Color(0.025f, 0.035f, 0.055f, 0.96f),
            new Color(0.26f, 0.38f, 0.52f, 1f));
        GUI.Label(
            new Rect(headerRect.x + 14f, headerRect.y + 8f, headerRect.width - 28f, headerRect.height - 14f),
            HeaderText,
            _headerStyle);

        float cardY = headerRect.yMax + gap;
        var stage3Rect = new Rect(12f, cardY, cardWidth, cardHeight);
        var shadowRect = new Rect(stage3Rect.xMax + gap, cardY, cardWidth, cardHeight);
        DrawCardBackground(
            stage3Rect,
            new Color(0.025f, 0.065f, 0.09f, 0.95f),
            new Color(0.18f, 0.68f, 0.88f, 1f));
        DrawCardBackground(
            shadowRect,
            new Color(0.065f, 0.04f, 0.085f, 0.95f),
            new Color(0.65f, 0.42f, 0.88f, 1f));
        GUI.Label(
            new Rect(stage3Rect.x + 16f, stage3Rect.y + 12f, stage3Rect.width - 32f, stage3Rect.height - 24f),
            Stage3Text,
            _panelStyle);
        GUI.Label(
            new Rect(shadowRect.x + 16f, shadowRect.y + 12f, shadowRect.width - 32f, shadowRect.height - 24f),
            ShadowText,
            _panelStyle);

        GUI.matrix = previousMatrix;
    }

    private static void DrawCardBackground(Rect rect, Color background, Color accent)
    {
        Color previousColor = GUI.color;
        GUI.color = background;
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = accent;
        GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, 4f), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(rect.x, rect.y, 1f, rect.height), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(rect.xMax - 1f, rect.y, 1f, rect.height), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(rect.x, rect.yMax - 1f, rect.width, 1f), Texture2D.whiteTexture);
        GUI.color = previousColor;
    }
}
