using System;
using UnityEngine;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Runtime IMGUI front-end for the unified simulation diagnostics snapshot.
/// Add it to any scene object, or let the editor/development bootstrap create it.
/// </summary>
public sealed class SimulationDebuggerPanel : MonoBehaviour
{
    [Header("Window")]
    public bool Visible = true;
    public KeyCode ToggleKey = KeyCode.F8;
    public Rect WindowRect = new Rect(18f, 18f, 520f, 520f);
    public bool AutoRefreshCaptureMask = true;

    [Header("Zoom")]
    [Range(0.5f, 2f)]
    public float FontScale = 1f;
    private const float ZoomStep = 0.1f;
    private float _lastFontScale;

    private const int WindowId = 0x51A7;
    private Vector2 _scroll;
    private bool _showDetails;
    private bool _settingsInitialized;
    private SimulationDebuggerEffectiveSettings _settingsDraft;
    private GUIStyle _windowStyle;
    private GUIStyle _headerStyle;
    private GUIStyle _sectionStyle;
    private GUIStyle _metricLabelStyle;
    private GUIStyle _metricValueStyle;
    private GUIStyle _mutedStyle;
    private GUIStyle _statusStyle;
    private GUIStyle _tabStyle;
    private GUIStyle _activeTabStyle;
    private Texture2D _panelTexture;
    private Texture2D _cardTexture;
    private Texture2D _activeTexture;

    private void OnEnable()
    {
        RefreshCaptureMask();
    }

    private void OnDisable()
    {
        if (AutoRefreshCaptureMask)
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.None;
    }

    private void OnDestroy()
    {
        DestroyRuntimeTexture(ref _panelTexture);
        DestroyRuntimeTexture(ref _cardTexture);
        DestroyRuntimeTexture(ref _activeTexture);
    }

    private void Update()
    {
        if (Input.GetKeyDown(ToggleKey))
        {
            Visible = !Visible;
            RefreshCaptureMask();
        }

        if (Visible)
            HandleZoomInput();
    }

    private void HandleZoomInput()
    {
        bool ctrl = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);

        if (ctrl && Input.GetKeyDown(KeyCode.Equals) || ctrl && Input.GetKeyDown(KeyCode.KeypadPlus))
            FontScale = Mathf.Clamp(FontScale + ZoomStep, 0.5f, 2f);
        else if (ctrl && Input.GetKeyDown(KeyCode.Minus) || ctrl && Input.GetKeyDown(KeyCode.KeypadMinus))
            FontScale = Mathf.Clamp(FontScale - ZoomStep, 0.5f, 2f);
        else if (ctrl && Input.GetKeyDown(KeyCode.Alpha0))
            FontScale = 1f;

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (ctrl && Mathf.Abs(scroll) > 0.001f)
            FontScale = Mathf.Clamp(FontScale + scroll * 0.2f, 0.5f, 2f);
    }

    private void OnGUI()
    {
        if (!Visible)
            return;

        EnsureStyles();
        WindowRect = GUILayout.Window(
            WindowId,
            WindowRect,
            DrawWindow,
            GUIContent.none,
            _windowStyle,
            GUILayout.MinWidth(440f),
            GUILayout.MinHeight(360f));
        WindowRect.x = Mathf.Clamp(WindowRect.x, 0f, Mathf.Max(0f, Screen.width - 80f));
        WindowRect.y = Mathf.Clamp(WindowRect.y, 0f, Mathf.Max(0f, Screen.height - 40f));

        if (Event.current.type == EventType.ScrollWheel &&
            WindowRect.Contains(Event.current.mousePosition))
        {
            Event.current.Use();
        }
    }

    private void DrawWindow(int id)
    {
        DrawHeader();
        DrawTabs();
        _scroll = GUILayout.BeginScrollView(_scroll, false, true);

        if (!SimulationDebuggerRuntime.TryGetLatest(out SimulationDebuggerFrameSnapshot snapshot))
        {
            GUILayout.Space(24f);
            GUILayout.Label("等待 Simulation 诊断快照…", _sectionStyle);
            GUILayout.Label(
                "确认 FlowMovementSystem 正在运行，并且 Capture Mask 未设为 None。",
                _mutedStyle);
        }
        else
        {
            DrawFrameStrip(snapshot);
            switch (SimulationDebuggerRuntime.ActiveView)
            {
                case SimulationDebuggerView.Overview:
                    DrawOverview(snapshot);
                    break;
                case SimulationDebuggerView.PersistentBroadPhase:
                    DrawPersistentBroadPhase(snapshot);
                    break;
                case SimulationDebuggerView.TimestepContactSet:
                    DrawContactSet(snapshot);
                    break;
                case SimulationDebuggerView.RuntimeSettings:
                    DrawSettingsSummary(snapshot);
                    break;
            }
        }

        GUILayout.EndScrollView();
        GUI.DragWindow(new Rect(0f, 0f, WindowRect.width - 42f, 30f));
    }

    private void DrawHeader()
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label("SIMULATION DEBUGGER", _headerStyle, GUILayout.ExpandWidth(true));

        if (GUILayout.Button("−", GUILayout.Width(24f), GUILayout.Height(24f)))
            FontScale = Mathf.Clamp(FontScale - ZoomStep, 0.5f, 2f);
        GUILayout.Label($"{FontScale * 100f:0}%", _mutedStyle, GUILayout.Width(36f));
        if (GUILayout.Button("+", GUILayout.Width(24f), GUILayout.Height(24f)))
            FontScale = Mathf.Clamp(FontScale + ZoomStep, 0.5f, 2f);

        bool frozen = SimulationDebuggerRuntime.FreezeSnapshot;
        if (GUILayout.Button(frozen ? "继续" : "冻结", GUILayout.Width(54f), GUILayout.Height(24f)))
            SimulationDebuggerRuntime.FreezeSnapshot = !frozen;

        bool overlay = SimulationDebuggerRuntime.OverlayEnabled;
        if (GUILayout.Button(overlay ? "Overlay" : "Overlay ×", GUILayout.Width(72f), GUILayout.Height(24f)))
            SimulationDebuggerRuntime.OverlayEnabled = !overlay;

        if (GUILayout.Button("×", GUILayout.Width(28f), GUILayout.Height(24f)))
        {
            Visible = false;
            RefreshCaptureMask();
        }
        GUILayout.EndHorizontal();
    }

    private void DrawTabs()
    {
        GUILayout.BeginHorizontal();
        DrawTab("整体", SimulationDebuggerView.Overview);
        DrawTab("跨帧 AABB", SimulationDebuggerView.PersistentBroadPhase);
        DrawTab("跨子步 Contact", SimulationDebuggerView.TimestepContactSet);
        DrawTab("设置", SimulationDebuggerView.RuntimeSettings);
        GUILayout.EndHorizontal();
        GUILayout.Space(5f);
    }

    private void DrawTab(string label, SimulationDebuggerView view)
    {
        bool active = SimulationDebuggerRuntime.ActiveView == view;
        GUIStyle style = active ? _activeTabStyle : _tabStyle;
        if (!GUILayout.Button(label, style, GUILayout.Height(30f)))
            return;

        SimulationDebuggerRuntime.ActiveView = view;
        _showDetails = false;
        SetDefaultHeatmap(view);
        RefreshCaptureMask();
    }

    private void DrawFrameStrip(SimulationDebuggerFrameSnapshot snapshot)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Frame {snapshot.FrameId}", _mutedStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(
            $"dt {snapshot.DeltaTime * 1000f:0.00} ms  ·  " +
            $"{snapshot.SubstepCount} substeps  ·  {snapshot.IterationCount} iterations",
            _mutedStyle);
        GUILayout.EndHorizontal();
        GUILayout.Space(6f);
    }

    private void DrawOverview(SimulationDebuggerFrameSnapshot snapshot)
    {
        SimulationOverviewMetrics metrics = snapshot.Overview;
        DrawStatus("整体仿真", metrics.Health, OverviewStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric("求解耗时", $"{metrics.SolverMilliseconds:0.000} ms", "整套移动与碰撞每帧成本");
        DrawMetric("单位数量", metrics.UnitCount.ToString(), "本帧参与求解的单位");
        DrawMetric("最大接触修正", metrics.MaxContactCorrection.ToString("0.000"), "单轮最强的位置修正");
        GUILayout.EndHorizontal();

        DrawHeatmapSelector(
            "整体热力图",
            new[]
            {
                SimulationDebuggerHeatmap.OverallPressure,
                SimulationDebuggerHeatmap.UnitDensity,
                SimulationDebuggerHeatmap.SolverCorrection
            });
        DrawHeatmapLegend();
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("阶段详情", _sectionStyle);
        DrawTimeBreakdown(metrics);
        DrawDetailRow("Broad 候选", metrics.CandidatePairCount.ToString("N0"));
        DrawDetailRow("Contact Set", metrics.ContactPairCount.ToString("N0"));
        DrawDetailRow("最大墙体修正", metrics.MaxWallCorrection.ToString("0.000"));
        DrawDetailRow("最大速度变化", metrics.MaxVelocityChange.ToString("0.000"));
    }

    private void DrawPersistentBroadPhase(SimulationDebuggerFrameSnapshot snapshot)
    {
        PersistentBroadPhaseMetrics metrics = snapshot.BroadPhase;
        DrawStatus("跨帧 AABB", metrics.Health, BroadPhaseStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric("缓存复用率", Percent(metrics.ReuseRatio), "复用次数 / 复用与重建总次数");
        DrawMetric("候选膨胀", $"{metrics.CandidateExpansion:0.00}×", "缓存候选 / 最终 Contact");
        DrawMetric("重建 / 回退", $"{metrics.RebuildCount} / {metrics.FallbackCount}", "重建越少越好，回退应接近 0");
        GUILayout.EndHorizontal();

        DrawHeatmapSelector(
            "AABB 热力图",
            new[]
            {
                SimulationDebuggerHeatmap.AabbBenefit,
                SimulationDebuggerHeatmap.AabbSlack,
                SimulationDebuggerHeatmap.CandidateExpansion,
                SimulationDebuggerHeatmap.EscapeRisk
            });
        DrawHeatmapLegend();
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("缓存详情", _sectionStyle);
        DrawDetailRow("状态", metrics.Enabled == 0 ? "关闭" : metrics.Valid != 0 ? "有效" : "无效");
        DrawDetailRow("缓存年龄", $"{metrics.CacheAgeFrames} 帧");
        DrawDetailRow("缓存候选 Pair", metrics.CachedCandidatePairCount.ToString("N0"));
        DrawDetailRow("最终 Contact", metrics.FinalContactPairCount.ToString("N0"));
        DrawDetailRow("Invalidation", metrics.InvalidationCount.ToString("N0"));
        DrawDetailRow("估算收益评分", metrics.EstimatedBenefitScore.ToString("+0.00;-0.00;0.00"));
    }

    private void DrawContactSet(SimulationDebuggerFrameSnapshot snapshot)
    {
        TimestepContactSetMetrics metrics = snapshot.ContactSet;
        DrawStatus("跨子步接触缓存", metrics.Health, ContactSetStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric("Contact Set", metrics.ContactSetSize.ToString("N0"), "整个 timestep 复用的约束拓扑");
        DrawMetric("接触激活率", Percent(metrics.ActivationRatio), "至少一次真正产生约束作用的 Contact");
        DrawMetric("补充 / 回退", metrics.SupplementOrFallbackCount.ToString("N0"), "初始 Contact Set 未覆盖的异常路径");
        GUILayout.EndHorizontal();

        DrawHeatmapSelector(
            "Contact Set 热力图",
            new[]
            {
                SimulationDebuggerHeatmap.ContactActivation,
                SimulationDebuggerHeatmap.ContactWaste,
                SimulationDebuggerHeatmap.ContactSupplementRisk
            });
        DrawHeatmapLegend();
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("缓存组成", _sectionStyle);
        DrawDetailRow("Actual / Near", metrics.ActualContactCount.ToString("N0"));
        DrawDetailRow("Predictive", metrics.PredictiveContactCount.ToString("N0"));
        DrawDetailRow("Predictive 已激活", metrics.PredictiveActivatedCount.ToString("N0"));
        DrawDetailRow("缓存但未激活", metrics.InactiveContactCount.ToString("N0"));
        DrawDetailRow("避免重复生成", $"{metrics.AvoidedContactGenerationCount} 次");
        DrawDetailRow("Predictive 激活率", Percent(metrics.PredictiveActivationRatio));
    }

    private void DrawSettingsSummary(SimulationDebuggerFrameSnapshot snapshot)
    {
        DrawStatus("运行时设置", SimulationDebuggerHealth.Healthy, "修改会在下一个 timestep 边界统一生效");
        GUILayout.Space(8f);

        if (!_settingsInitialized)
        {
            _settingsDraft = snapshot.EffectiveSettings;
            _settingsInitialized = true;
        }

        GUILayout.Label("Global / XPBD", _sectionStyle);
        _settingsDraft.SubstepCount = DrawIntSlider(
            "Substeps",
            _settingsDraft.SubstepCount,
            snapshot.EffectiveSettings.SubstepCount,
            1,
            16);
        _settingsDraft.IterationCount = DrawIntSlider(
            "Iterations",
            _settingsDraft.IterationCount,
            snapshot.EffectiveSettings.IterationCount,
            1,
            24);
        _settingsDraft.Compliance = DrawFloatSlider(
            "Compliance",
            _settingsDraft.Compliance,
            snapshot.EffectiveSettings.Compliance,
            0f,
            0.1f,
            "0.0000");
        _settingsDraft.EnableDiagnostics = DrawToggle(
            "Solver diagnostics",
            _settingsDraft.EnableDiagnostics,
            snapshot.EffectiveSettings.EnableDiagnostics);

        GUILayout.Space(6f);
        GUILayout.Label("Soft Avoidance", _sectionStyle);
        _settingsDraft.SoftAvoidanceResponseRate = DrawFloatSlider(
            "Response rate",
            _settingsDraft.SoftAvoidanceResponseRate,
            snapshot.EffectiveSettings.SoftAvoidanceResponseRate,
            0f,
            20f,
            "0.00");
        _settingsDraft.SoftAvoidanceShell = DrawFloatSlider(
            "Surface shell",
            _settingsDraft.SoftAvoidanceShell,
            snapshot.EffectiveSettings.SoftAvoidanceShell,
            0f,
            4f,
            "0.00");
        _settingsDraft.SettledSoftAvoidanceMultiplier = DrawFloatSlider(
            "Settled multiplier",
            _settingsDraft.SettledSoftAvoidanceMultiplier,
            snapshot.EffectiveSettings.SettledSoftAvoidanceMultiplier,
            0f,
            2f,
            "0.00");
        _settingsDraft.RvoTimeHorizon = DrawFloatSlider(
            "RVO horizon",
            _settingsDraft.RvoTimeHorizon,
            snapshot.EffectiveSettings.RvoTimeHorizon,
            0.05f,
            5f,
            "0.00");
        GUILayout.BeginHorizontal();
        GUILayout.Label("Velocity solver", _mutedStyle, GUILayout.Width(170f));
        string[] solverModes = { "Steering", "Reciprocal" };
        _settingsDraft.SoftAvoidanceVelocitySolver = GUILayout.SelectionGrid(
            Mathf.Clamp(_settingsDraft.SoftAvoidanceVelocitySolver, 0, 1),
            solverModes,
            2,
            _tabStyle);
        GUILayout.EndHorizontal();

        GUILayout.Space(6f);
        GUILayout.Label("Persistent Broad Phase", _sectionStyle);
        _settingsDraft.EnableFatAabbCache = DrawToggle(
            "Enable Fat AABB",
            _settingsDraft.EnableFatAabbCache,
            snapshot.EffectiveSettings.EnableFatAabbCache);
        bool previousEnabled = GUI.enabled;
        GUI.enabled = previousEnabled && _settingsDraft.EnableFatAabbCache != 0;
        _settingsDraft.FatAabbCacheMargin = DrawFloatSlider(
            "Fat margin",
            _settingsDraft.FatAabbCacheMargin,
            snapshot.EffectiveSettings.FatAabbCacheMargin,
            0f,
            5f,
            "0.00");
        _settingsDraft.EnableAdaptiveFatAabb = DrawToggle(
            "Adaptive hotspot routing",
            _settingsDraft.EnableAdaptiveFatAabb,
            snapshot.EffectiveSettings.EnableAdaptiveFatAabb);
        _settingsDraft.AdaptiveDetectionCellSpan = DrawIntSlider(
            "Detection cell span",
            _settingsDraft.AdaptiveDetectionCellSpan,
            snapshot.EffectiveSettings.AdaptiveDetectionCellSpan,
            1,
            8);
        _settingsDraft.AdaptiveMinimumUnitsPerCell = DrawIntSlider(
            "Min units / cell",
            _settingsDraft.AdaptiveMinimumUnitsPerCell,
            snapshot.EffectiveSettings.AdaptiveMinimumUnitsPerCell,
            1,
            32);
        _settingsDraft.AdaptiveMinimumUnitsPerRegion = DrawIntSlider(
            "Min units / region",
            _settingsDraft.AdaptiveMinimumUnitsPerRegion,
            snapshot.EffectiveSettings.AdaptiveMinimumUnitsPerRegion,
            1,
            128);
        _settingsDraft.AdaptiveEnableScore = DrawFloatSlider(
            "Enable score",
            _settingsDraft.AdaptiveEnableScore,
            snapshot.EffectiveSettings.AdaptiveEnableScore,
            0f,
            1f,
            "0.00");
        _settingsDraft.AdaptiveDisableScore = DrawFloatSlider(
            "Disable score",
            _settingsDraft.AdaptiveDisableScore,
            snapshot.EffectiveSettings.AdaptiveDisableScore,
            0f,
            _settingsDraft.AdaptiveEnableScore,
            "0.00");
        GUI.enabled = previousEnabled;

        GUILayout.Space(6f);
        GUILayout.Label("Timestep Contact Set", _sectionStyle);
        _settingsDraft.EnablePredictivePairGeneration = DrawToggle(
            "Predictive pair generation",
            _settingsDraft.EnablePredictivePairGeneration,
            snapshot.EffectiveSettings.EnablePredictivePairGeneration);
        GUI.enabled = previousEnabled && _settingsDraft.EnablePredictivePairGeneration != 0;
        _settingsDraft.EnablePredictiveContacts = DrawToggle(
            "Predictive contact solve",
            _settingsDraft.EnablePredictiveContacts,
            snapshot.EffectiveSettings.EnablePredictiveContacts);
        _settingsDraft.PredictiveSkin = DrawFloatSlider(
            "Predictive skin",
            _settingsDraft.PredictiveSkin,
            snapshot.EffectiveSettings.PredictiveSkin,
            0f,
            3f,
            "0.00");
        GUI.enabled = previousEnabled;

        GUILayout.Space(6f);
        GUILayout.Label("Diagnostics", _sectionStyle);
        SimulationDebuggerRuntime.SummarySampleIntervalFrames = DrawIntSlider(
            "Summary interval",
            SimulationDebuggerRuntime.SummarySampleIntervalFrames,
            SimulationDebuggerRuntime.SummarySampleIntervalFrames,
            1,
            30);
        SimulationDebuggerRuntime.SpatialSampleIntervalFrames = DrawIntSlider(
            "Spatial interval",
            SimulationDebuggerRuntime.SpatialSampleIntervalFrames,
            SimulationDebuggerRuntime.SpatialSampleIntervalFrames,
            1,
            30);
        SimulationDebuggerRuntime.MaximumVisualizedPairs = DrawIntSlider(
            "Max pair lines",
            SimulationDebuggerRuntime.MaximumVisualizedPairs,
            SimulationDebuggerRuntime.MaximumVisualizedPairs,
            1,
            128);
        SimulationDebuggerRuntime.HeatmapOpacity = DrawFloatSlider(
            "Heatmap opacity",
            SimulationDebuggerRuntime.HeatmapOpacity,
            SimulationDebuggerRuntime.HeatmapOpacity,
            0f,
            0.8f,
            "0.00");

        GUILayout.Space(10f);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("应用 Override", GUILayout.Height(30f)))
            SimulationDebuggerRuntime.SubmitSettings(_settingsDraft);
        if (GUILayout.Button("读取 Effective", GUILayout.Height(30f)))
            _settingsDraft = snapshot.EffectiveSettings;
        if (GUILayout.Button("恢复 Authoring", GUILayout.Height(30f)))
        {
            SimulationDebuggerRuntime.RequestSettingsReset();
            if (SimulationDebuggerRuntime.TryGetBaselineSettings(out SimulationDebuggerEffectiveSettings baseline))
                _settingsDraft = baseline;
        }
        GUILayout.EndHorizontal();
        GUILayout.Label(
            "Adaptive 参数只有场景中存在 AdaptiveFatAabbSettings singleton 时才会写回。",
            _mutedStyle);
    }

    private int DrawIntSlider(
        string label,
        int draft,
        int effective,
        int minimum,
        int maximum)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, _mutedStyle, GUILayout.Width(170f));
        int value = Mathf.RoundToInt(GUILayout.HorizontalSlider(draft, minimum, maximum));
        GUILayout.Label($"{value}  (有效 {effective})", _mutedStyle, GUILayout.Width(115f));
        GUILayout.EndHorizontal();
        return Mathf.Clamp(value, minimum, maximum);
    }

    private float DrawFloatSlider(
        string label,
        float draft,
        float effective,
        float minimum,
        float maximum,
        string format)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, _mutedStyle, GUILayout.Width(170f));
        float value = GUILayout.HorizontalSlider(draft, minimum, Mathf.Max(minimum, maximum));
        GUILayout.Label(
            $"{value.ToString(format)}  (有效 {effective.ToString(format)})",
            _mutedStyle,
            GUILayout.Width(115f));
        GUILayout.EndHorizontal();
        return Mathf.Clamp(value, minimum, Mathf.Max(minimum, maximum));
    }

    private byte DrawToggle(string label, byte draft, byte effective)
    {
        GUILayout.BeginHorizontal();
        bool enabled = GUILayout.Toggle(draft != 0, label, GUILayout.Width(240f));
        GUILayout.FlexibleSpace();
        GUILayout.Label(effective != 0 ? "有效：开" : "有效：关", _mutedStyle);
        GUILayout.EndHorizontal();
        return (byte)(enabled ? 1 : 0);
    }

    private void DrawHeatmapLegend()
    {
        SimulationDebuggerHeatmap mode = SimulationDebuggerRuntime.ActiveHeatmap;
        if (mode == SimulationDebuggerHeatmap.None)
            return;

        GUILayout.BeginHorizontal();
        GUILayout.Space(122f);
        GUILayout.Label(HeatmapLowLabel(mode), _mutedStyle, GUILayout.Width(82f));
        Rect rect = GUILayoutUtility.GetRect(80f, 8f, GUILayout.ExpandWidth(true));
        if (Event.current.type == EventType.Repaint)
        {
            Color left = HeatmapEndpoint(mode, false);
            Color right = HeatmapEndpoint(mode, true);
            int segments = 20;
            float width = rect.width / segments;
            for (int i = 0; i < segments; i++)
            {
                float t = i / (float)(segments - 1);
                EditorSafeDrawRect(
                    new Rect(rect.x + i * width, rect.y, width + 1f, rect.height),
                    Color.Lerp(left, right, t));
            }
        }
        GUILayout.Label(HeatmapHighLabel(mode), _mutedStyle, GUILayout.Width(82f));
        GUILayout.EndHorizontal();
    }

    private void DrawSelectedUnitSection(SimulationDebuggerFrameSnapshot snapshot)
    {
        GUILayout.Space(8f);
        if (!snapshot.HasSelectedUnit)
        {
            GUILayout.BeginVertical(_sectionStyle);
            GUILayout.Label("选中单位", _metricLabelStyle);
            GUILayout.Label(
                "点击单位后，这里会显示该单位的运动、AABB 和跨子步 Contact；世界中只绘制它相关的范围与 Pair。",
                _mutedStyle);
            GUILayout.EndVertical();
            return;
        }

        SimulationDebuggerUnitSample unit = snapshot.SelectedUnit;
        GUILayout.BeginVertical(_sectionStyle);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"选中单位  Entity {unit.Entity.Index}:{unit.Entity.Version}", _sectionStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label($"Body {unit.BodyIndex}", _mutedStyle);
        GUILayout.EndHorizontal();

        switch (SimulationDebuggerRuntime.ActiveView)
        {
            case SimulationDebuggerView.Overview:
                DrawDetailRow("软避让邻居", unit.SoftNeighborCount.ToString("N0"));
                DrawDetailRow("接触 / 墙体修正", $"{unit.ContactCorrection:0.000} / {unit.WallCorrection:0.000}");
                DrawDetailRow("速度", $"{unit.CurrentVelocity.x:0.00}, {unit.CurrentVelocity.z:0.00}");
                break;
            case SimulationDebuggerView.PersistentBroadPhase:
                if (unit.HasFatBounds == 0)
                {
                    DrawDetailRow("AABB", "当前未捕获到 Fat Bounds");
                    break;
                }
                DrawDetailRow("Swept 尺寸", BoundsSize(unit.SweptMin, unit.SweptMax));
                DrawDetailRow("Fat 尺寸", BoundsSize(unit.FatMin, unit.FatMax));
                DrawDetailRow("最小剩余余量", MinimumSlack(unit).ToString("0.000"));
                break;
            case SimulationDebuggerView.TimestepContactSet:
                DrawDetailRow("捕获 Contact", unit.CapturedPairCount.ToString("N0"));
                DrawDetailRow("缓存 Contact", unit.CachedContactCount.ToString("N0"));
                DrawDetailRow("当前激活", unit.ActiveContactCount.ToString("N0"));
                if (_showDetails)
                    DrawSelectedPairRows(snapshot);
                break;
        }
        GUILayout.EndVertical();
    }

    private void DrawSelectedPairRows(SimulationDebuggerFrameSnapshot snapshot)
    {
        if (snapshot.SelectedPairs.Count == 0)
        {
            GUILayout.Label("当前没有捕获到该单位的 Contact Pair。", _mutedStyle);
            return;
        }

        GUILayout.Space(4f);
        int count = Mathf.Min(12, snapshot.SelectedPairs.Count);
        for (int i = 0; i < count; i++)
        {
            SimulationDebuggerPairSample pair = snapshot.SelectedPairs[i];
            string state = pair.State == SimulationDebuggerPairState.Active ? "Active" : "Cached";
            string kind = PairKindLabel(pair.Kind);
            GUILayout.Label(
                $"{i + 1,2}. {kind,-10} {state,-6}  sep {pair.CurrentSeparation,7:0.000}  " +
                $"λ {pair.Lambda,7:0.000}  substep {pair.FirstActivatedSubstep}",
                _mutedStyle);
        }
        if (snapshot.SelectedPairs.Count > count)
            GUILayout.Label($"其余 {snapshot.SelectedPairs.Count - count} 条仅在世界 Overlay 中按上限绘制。", _mutedStyle);
    }

    private static string BoundsSize(Unity.Mathematics.float2 min, Unity.Mathematics.float2 max)
    {
        Unity.Mathematics.float2 size = max - min;
        return $"{size.x:0.00} × {size.y:0.00}";
    }

    private static float MinimumSlack(SimulationDebuggerUnitSample unit)
    {
        if (unit.HasFatBounds == 0)
            return 0f;
        return Mathf.Min(
            unit.SweptMin.x - unit.FatMin.x,
            unit.SweptMin.y - unit.FatMin.y,
            unit.FatMax.x - unit.SweptMax.x,
            unit.FatMax.y - unit.SweptMax.y);
    }

    private static string PairKindLabel(SimulationDebuggerPairKind kind)
    {
        return kind switch
        {
            SimulationDebuggerPairKind.PredictiveContact => "Predictive",
            SimulationDebuggerPairKind.NearContact => "Near",
            SimulationDebuggerPairKind.SupplementedContact => "Supplement",
            SimulationDebuggerPairKind.BroadCandidate => "Candidate",
            _ => "Actual"
        };
    }

    private static string HeatmapLowLabel(SimulationDebuggerHeatmap mode)
    {
        return mode switch
        {
            SimulationDebuggerHeatmap.AabbBenefit or
            SimulationDebuggerHeatmap.AabbSlack or
            SimulationDebuggerHeatmap.ContactActivation => "差 / 低",
            _ => "低"
        };
    }

    private static string HeatmapHighLabel(SimulationDebuggerHeatmap mode)
    {
        return mode switch
        {
            SimulationDebuggerHeatmap.AabbBenefit or
            SimulationDebuggerHeatmap.AabbSlack or
            SimulationDebuggerHeatmap.ContactActivation => "好 / 高",
            _ => "高 / 风险"
        };
    }

    private static Color HeatmapEndpoint(SimulationDebuggerHeatmap mode, bool high)
    {
        bool positive = mode == SimulationDebuggerHeatmap.AabbBenefit ||
                        mode == SimulationDebuggerHeatmap.AabbSlack ||
                        mode == SimulationDebuggerHeatmap.ContactActivation;
        if (positive)
            return high ? new Color(0.15f, 0.8f, 0.42f) : new Color(0.9f, 0.2f, 0.18f);
        return high ? new Color(0.95f, 0.18f, 0.08f) : new Color(0.12f, 0.4f, 0.95f);
    }

    private static void EditorSafeDrawRect(Rect rect, Color color)
    {
        Color previous = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = previous;
    }

    private void DrawHeatmapSelector(
        string title,
        SimulationDebuggerHeatmap[] modes)
    {
        GUILayout.Space(9f);
        GUILayout.BeginHorizontal();
        GUILayout.Label(title, _sectionStyle, GUILayout.Width(118f));
        for (int i = 0; i < modes.Length; i++)
        {
            SimulationDebuggerHeatmap mode = modes[i];
            bool active = SimulationDebuggerRuntime.ActiveHeatmap == mode;
            if (GUILayout.Button(
                    HeatmapLabel(mode),
                    active ? _activeTabStyle : _tabStyle,
                    GUILayout.Height(25f)))
            {
                SimulationDebuggerRuntime.ActiveHeatmap = active
                    ? SimulationDebuggerHeatmap.None
                    : mode;
                SimulationDebuggerRuntime.OverlayEnabled = true;
                RefreshCaptureMask();
            }
        }
        GUILayout.EndHorizontal();
    }

    private void DrawTimeBreakdown(SimulationOverviewMetrics metrics)
    {
        long known = metrics.SoftAvoidanceNanoseconds +
                     metrics.PairGenerationNanoseconds +
                     metrics.IterationNanoseconds;
        long other = Math.Max(0, metrics.SolverNanoseconds - known);
        DrawDetailRow("软避让", Nanoseconds(metrics.SoftAvoidanceNanoseconds));
        DrawDetailRow("Pair / Contact 生成", Nanoseconds(metrics.PairGenerationNanoseconds));
        DrawDetailRow("XPBD Iteration", Nanoseconds(metrics.IterationNanoseconds));
        DrawDetailRow("其他阶段", Nanoseconds(other));
    }

    private void DrawStatus(string title, SimulationDebuggerHealth health, string explanation)
    {
        Color previous = GUI.color;
        GUI.color = HealthColor(health);
        GUILayout.BeginVertical(_statusStyle);
        GUI.color = previous;
        GUILayout.BeginHorizontal();
        GUILayout.Label(title, _sectionStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(HealthLabel(health), _sectionStyle);
        GUILayout.EndHorizontal();
        GUILayout.Label(explanation, _mutedStyle);
        GUILayout.EndVertical();
    }

    private void DrawMetric(string label, string value, string hint)
    {
        GUILayout.BeginVertical(_sectionStyle, GUILayout.MinWidth(0f), GUILayout.ExpandWidth(true));
        GUILayout.Label(label, _metricLabelStyle);
        GUILayout.Label(value, _metricValueStyle);
        GUILayout.Label(hint, _mutedStyle, GUILayout.MinHeight(30f));
        GUILayout.EndVertical();
    }

    private void DrawDetailsToggle()
    {
        GUILayout.Space(9f);
        if (GUILayout.Button(_showDetails ? "收起详细数据" : "展开详细数据", GUILayout.Height(26f)))
        {
            _showDetails = !_showDetails;
            RefreshCaptureMask();
        }
    }

    private void DrawDetailRow(string name, string value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(name, _mutedStyle, GUILayout.Width(210f));
        GUILayout.FlexibleSpace();
        GUILayout.Label(value, _sectionStyle);
        GUILayout.EndHorizontal();
    }

    private void RefreshCaptureMask()
    {
        if (!AutoRefreshCaptureMask || !Visible)
        {
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.None;
            return;
        }

        SimulationDebuggerCaptureMask mask = SimulationDebuggerCaptureMask.Summary;
        switch (SimulationDebuggerRuntime.ActiveView)
        {
            case SimulationDebuggerView.Overview:
                mask |= SimulationDebuggerCaptureMask.OverviewHeatmap |
                        SimulationDebuggerCaptureMask.SelectedUnit |
                        SimulationDebuggerCaptureMask.Proxies;
                break;
            case SimulationDebuggerView.PersistentBroadPhase:
                mask |= SimulationDebuggerCaptureMask.AabbHeatmap |
                        SimulationDebuggerCaptureMask.Regions |
                        SimulationDebuggerCaptureMask.Proxies |
                        SimulationDebuggerCaptureMask.SelectedUnit;
                break;
            case SimulationDebuggerView.TimestepContactSet:
                mask |= SimulationDebuggerCaptureMask.ContactSetHeatmap |
                        SimulationDebuggerCaptureMask.SelectedUnit |
                        SimulationDebuggerCaptureMask.SelectedPairs |
                        SimulationDebuggerCaptureMask.Proxies;
                break;
        }

        if (_showDetails)
            mask |= SimulationDebuggerCaptureMask.DetailedCounters;
        SimulationDebuggerRuntime.CaptureMask = mask;
    }

    private static void SetDefaultHeatmap(SimulationDebuggerView view)
    {
        SimulationDebuggerRuntime.ActiveHeatmap = view switch
        {
            SimulationDebuggerView.Overview => SimulationDebuggerHeatmap.OverallPressure,
            SimulationDebuggerView.PersistentBroadPhase => SimulationDebuggerHeatmap.AabbBenefit,
            SimulationDebuggerView.TimestepContactSet => SimulationDebuggerHeatmap.ContactActivation,
            _ => SimulationDebuggerHeatmap.None
        };
    }

    private static string OverviewStatus(SimulationOverviewMetrics metrics)
    {
        if (metrics.Health == SimulationDebuggerHealth.Critical)
            return "求解成本或位置修正明显偏高；展开阶段详情定位瓶颈。";
        if (metrics.Health == SimulationDebuggerHealth.Warning)
            return "局部拥堵正在增加求解压力。";
        return "成本和约束修正处于正常范围。";
    }

    private static string BroadPhaseStatus(PersistentBroadPhaseMetrics metrics)
    {
        if (metrics.Enabled == 0)
            return "缓存未启用，当前使用普通 Broad Phase。";
        if (metrics.Health == SimulationDebuggerHealth.Critical)
            return "缓存发生回退或已经成为负收益。";
        if (metrics.Health == SimulationDebuggerHealth.Warning)
            return "候选膨胀或重建频率偏高。";
        return "缓存稳定复用，候选膨胀可控。";
    }

    private static string ContactSetStatus(TimestepContactSetMetrics metrics)
    {
        if (metrics.Health == SimulationDebuggerHealth.Critical)
            return "本 timestep 出现后补或回退，初始 Contact Set 可能不完整。";
        if (metrics.Health == SimulationDebuggerHealth.Warning)
            return "缓存中未激活 Contact 较多，生成范围可能过于保守。";
        return "同一 Contact Set 正在跨 substep 稳定复用。";
    }

    private static string HeatmapLabel(SimulationDebuggerHeatmap mode)
    {
        return mode switch
        {
            SimulationDebuggerHeatmap.OverallPressure => "综合压力",
            SimulationDebuggerHeatmap.UnitDensity => "密度",
            SimulationDebuggerHeatmap.SolverCorrection => "修正量",
            SimulationDebuggerHeatmap.AabbBenefit => "缓存收益",
            SimulationDebuggerHeatmap.AabbSlack => "剩余余量",
            SimulationDebuggerHeatmap.CandidateExpansion => "候选膨胀",
            SimulationDebuggerHeatmap.EscapeRisk => "逃逸风险",
            SimulationDebuggerHeatmap.ContactActivation => "激活",
            SimulationDebuggerHeatmap.ContactWaste => "未使用",
            SimulationDebuggerHeatmap.ContactSupplementRisk => "漏检风险",
            _ => "关闭"
        };
    }

    private static string Percent(float value) => $"{Mathf.Clamp01(value) * 100f:0.0}%";
    private static string Nanoseconds(long value) => $"{value / 1_000_000f:0.000} ms";

    private static string HealthLabel(SimulationDebuggerHealth health)
    {
        return health switch
        {
            SimulationDebuggerHealth.Healthy => "正常",
            SimulationDebuggerHealth.Warning => "注意",
            SimulationDebuggerHealth.Critical => "异常",
            _ => "关闭"
        };
    }

    private static Color HealthColor(SimulationDebuggerHealth health)
    {
        return health switch
        {
            SimulationDebuggerHealth.Healthy => new Color(0.28f, 0.72f, 0.46f),
            SimulationDebuggerHealth.Warning => new Color(0.95f, 0.68f, 0.20f),
            SimulationDebuggerHealth.Critical => new Color(0.92f, 0.28f, 0.22f),
            _ => new Color(0.45f, 0.48f, 0.54f)
        };
    }

    private void EnsureStyles()
    {
        if (_headerStyle != null && Mathf.Approximately(_lastFontScale, FontScale))
            return;

        _lastFontScale = FontScale;

        DestroyRuntimeTexture(ref _panelTexture);
        DestroyRuntimeTexture(ref _cardTexture);
        DestroyRuntimeTexture(ref _activeTexture);

        _panelTexture = SolidTexture(new Color(0.065f, 0.075f, 0.095f, 0.97f));
        _cardTexture = SolidTexture(new Color(0.105f, 0.12f, 0.15f, 0.96f));
        _activeTexture = SolidTexture(new Color(0.18f, 0.34f, 0.55f, 0.98f));

        _windowStyle = new GUIStyle(GUI.skin.window)
        {
            padding = new RectOffset(12, 12, 10, 12),
            normal = { background = _panelTexture }
        };

        _headerStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = Mathf.RoundToInt(15f * FontScale),
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft
        };
        _sectionStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = Mathf.RoundToInt(12f * FontScale),
            fontStyle = FontStyle.Bold,
            wordWrap = true,
            padding = new RectOffset(8, 8, 7, 7),
            normal = { background = _cardTexture }
        };
        _metricLabelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = Mathf.RoundToInt(11f * FontScale),
            normal = { textColor = new Color(0.63f, 0.7f, 0.8f) }
        };
        _metricValueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = Mathf.RoundToInt(20f * FontScale),
            fontStyle = FontStyle.Bold
        };
        _mutedStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = Mathf.RoundToInt(10f * FontScale),
            wordWrap = true,
            normal = { textColor = new Color(0.58f, 0.64f, 0.72f) }
        };
        _statusStyle = new GUIStyle(GUI.skin.box)
        {
            padding = new RectOffset(8, 8, 5, 7),
            normal = { background = _cardTexture }
        };
        _tabStyle = new GUIStyle(GUI.skin.button)
        {
            fontSize = Mathf.RoundToInt(11f * FontScale),
            normal = { background = _cardTexture },
            hover = { background = _cardTexture }
        };
        _activeTabStyle = new GUIStyle(_tabStyle)
        {
            fontStyle = FontStyle.Bold,
            normal = { background = _activeTexture },
            hover = { background = _activeTexture }
        };
    }

    private static void DestroyRuntimeTexture(ref Texture2D texture)
    {
        if (texture == null)
            return;
        Destroy(texture);
        texture = null;
    }

    private static Texture2D SolidTexture(Color color)
    {
        var texture = new Texture2D(1, 1, TextureFormat.RGBA32, false)
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        texture.SetPixel(0, 0, color);
        texture.Apply();
        return texture;
    }
}

internal static class SimulationDebuggerPanelBootstrap
{
#if UNITY_EDITOR || DEVELOPMENT_BUILD
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void CreatePanelWhenMissing()
    {
        SimulationDebuggerPanel panel =
            UnityEngine.Object.FindFirstObjectByType<SimulationDebuggerPanel>();
        GameObject gameObject;
        if (panel == null)
        {
            gameObject = new GameObject("Simulation Debugger");
            gameObject.hideFlags = HideFlags.DontSave;
            panel = gameObject.AddComponent<SimulationDebuggerPanel>();
            UnityEngine.Object.DontDestroyOnLoad(gameObject);
        }
        else
        {
            gameObject = panel.gameObject;
        }

        if (gameObject.GetComponent<SimulationDebuggerWorldOverlay>() == null)
            gameObject.AddComponent<SimulationDebuggerWorldOverlay>();
    }
#endif
}
}
