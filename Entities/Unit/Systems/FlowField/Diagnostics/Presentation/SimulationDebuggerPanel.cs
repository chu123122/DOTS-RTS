using System;
using UnityEngine;

namespace RTS.Unit.FlowField.Diagnostics
{
[Serializable]
public sealed class SimulationDebuggerWindowState
{
    public Rect Rect;
    public bool Visible = true;
    public bool ShowDetails;
    public bool ShowGridHeatmap = true;
    public SimulationDebuggerHeatmap Heatmap;
    [NonSerialized] public Vector2 Scroll;
    [NonSerialized] public bool Resizing;
    [NonSerialized] public Vector2 ResizeStartMouse;
    [NonSerialized] public Vector2 ResizeStartSize;

    public static SimulationDebuggerWindowState Create(
        Rect rect,
        SimulationDebuggerHeatmap heatmap)
    {
        return new SimulationDebuggerWindowState
        {
            Rect = rect,
            Heatmap = heatmap,
            Visible = true,
            ShowGridHeatmap = true
        };
    }
}

/// <summary>
/// Runtime IMGUI front-end for the unified simulation diagnostics snapshot.
/// Add it to any scene object, or let the editor/development bootstrap create it.
/// </summary>
public sealed partial class SimulationDebuggerPanel : MonoBehaviour
{
    public static SimulationDebuggerPanel Instance { get; private set; }

    [Header("Window")]
    public bool Visible = true;
    public KeyCode ToggleKey = KeyCode.F8;
    public bool AutoRefreshCaptureMask = true;

    [Header("界面缩放")]
    [Range(0.5f, 2f)] public float FontScale = 1f;
    private const float ZoomStep = 0.1f;
    private float _lastFontScale;

    [Header("四窗口布局")]
    public SimulationDebuggerWindowState OverviewWindow =
        SimulationDebuggerWindowState.Create(new Rect(18f, 58f, 510f, 440f), SimulationDebuggerHeatmap.OverallPressure);
    public SimulationDebuggerWindowState AabbWindow =
        SimulationDebuggerWindowState.Create(new Rect(542f, 58f, 510f, 440f), SimulationDebuggerHeatmap.AabbBenefit);
    public SimulationDebuggerWindowState ContactWindow =
        SimulationDebuggerWindowState.Create(new Rect(18f, 512f, 510f, 440f), SimulationDebuggerHeatmap.ContactActivation);
    public SimulationDebuggerWindowState SettingsWindow =
        SimulationDebuggerWindowState.Create(new Rect(542f, 512f, 510f, 520f), SimulationDebuggerHeatmap.None);
    public Rect LauncherRect = new Rect(18f, 18f, 620f, 34f);

    private const int LauncherWindowId = 0x51A0;
    private const int OverviewWindowId = 0x51A1;
    private const int AabbWindowId = 0x51A2;
    private const int ContactWindowId = 0x51A3;
    private const int SettingsWindowId = 0x51A4;
    private Vector2 _scroll;
    private bool _showDetails;
    private SimulationDebuggerView _currentView;
    private SimulationDebuggerWindowState _activeWindowState;
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
    private Texture2D _overviewChartTexture;
    private Texture2D _timestepCostChartTexture;
    private Texture2D _timestepLoadChartTexture;
    private Texture2D _substepCostChartTexture;
    private Texture2D _substepLoadChartTexture;
    private readonly float[] _chartBufferA = new float[120];
    private readonly float[] _chartBufferB = new float[120];
    private readonly float[] _chartBufferC = new float[120];
    private readonly float[] _chartBufferD = new float[120];

    private void OnEnable()
    {
        Instance = this;
        EnsureWindowStates();
        RefreshCaptureMask();
    }

    private void OnDisable()
    {
        if (Instance == this)
            Instance = null;
        if (AutoRefreshCaptureMask)
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.None;
    }

    private void OnDestroy()
    {
        if (Instance == this)
            Instance = null;
        DestroyRuntimeTexture(ref _panelTexture);
        DestroyRuntimeTexture(ref _cardTexture);
        DestroyRuntimeTexture(ref _activeTexture);
        DestroyRuntimeTexture(ref _overviewChartTexture);
        DestroyRuntimeTexture(ref _timestepCostChartTexture);
        DestroyRuntimeTexture(ref _timestepLoadChartTexture);
        DestroyRuntimeTexture(ref _substepCostChartTexture);
        DestroyRuntimeTexture(ref _substepLoadChartTexture);
    }

    private void OnApplicationQuit()
    {
        Visible = false;
        if (AutoRefreshCaptureMask)
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.None;
        Destroy(gameObject);
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
        if (!ctrl)
            return;

        if (Input.GetKeyDown(KeyCode.Equals) || Input.GetKeyDown(KeyCode.KeypadPlus))
            FontScale = Mathf.Clamp(FontScale + ZoomStep, 0.5f, 2f);
        else if (Input.GetKeyDown(KeyCode.Minus) || Input.GetKeyDown(KeyCode.KeypadMinus))
            FontScale = Mathf.Clamp(FontScale - ZoomStep, 0.5f, 2f);
        else if (Input.GetKeyDown(KeyCode.Alpha0))
            FontScale = 1f;

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f && IsPointerOverDebugger(Input.mousePosition))
            FontScale = Mathf.Clamp(FontScale + scroll * 0.2f, 0.5f, 2f);
    }

    private void OnGUI()
    {
        if (!Visible)
            return;

        EnsureStyles();

        LauncherRect = GUI.Window(
            LauncherWindowId,
            LauncherRect,
            DrawLauncherWindow,
            GUIContent.none,
            _windowStyle);

        PublishedSimulationDiagnosticsRuntime.TryGetLatest(
            out PublishedSimulationDiagnosticsSnapshot published);
        DrawIndependentWindow(
            OverviewWindowId,
            SimulationDebuggerView.Overview,
            OverviewWindow,
            published);
        DrawIndependentWindow(
            AabbWindowId,
            SimulationDebuggerView.PersistentBroadPhase,
            AabbWindow,
            published);
        DrawIndependentWindow(
            ContactWindowId,
            SimulationDebuggerView.TimestepContactSet,
            ContactWindow,
            published);
        DrawIndependentWindow(
            SettingsWindowId,
            SimulationDebuggerView.RuntimeSettings,
            SettingsWindow,
            published);

        // 在窗口绘制之后占用 hotControl，防止被 GUI.Window 内部重置。
        // 注意：IMGUI 的 Event 系统无法阻止 Unity Input 系统（GetAxis/GetMouseButton）
        // 向 Game View 摄像机透传。RTS 摄像机脚本也需要检查：
        //   SimulationDebuggerPanel.IsPointerOverDebugger(Input.mousePosition)
        Event evt = Event.current;
        if (evt != null && IsGuiPointOverDebugger(evt.mousePosition))
        {
            switch (evt.type)
            {
                case EventType.MouseDown:
                    GUIUtility.hotControl = LauncherWindowId;
                    evt.Use();
                    break;
                case EventType.MouseDrag:
                case EventType.ScrollWheel:
                    GUIUtility.hotControl = LauncherWindowId;
                    evt.Use();
                    break;
                case EventType.MouseUp:
                    if (GUIUtility.hotControl == LauncherWindowId)
                        GUIUtility.hotControl = 0;
                    break;
            }
        }
    }

    private void DrawLauncherWindow(int id)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label("仿真诊断", _headerStyle, GUILayout.Width(78f));
        ulong[] worldIds = SimulationDebuggerRuntime.GetRegisteredWorldIds();
        GUILayout.Label($"W:{SimulationDebuggerRuntime.TargetWorldId}", _mutedStyle, GUILayout.Width(74f));
        GUI.enabled = worldIds.Length > 1;
        if (GUILayout.Button("切换世界", GUILayout.Width(64f)) &&
            SimulationDebuggerRuntime.SelectNextWorld())
            RefreshCaptureMask();
        GUI.enabled = true;
        if (GUILayout.Button("字−", GUILayout.Width(34f)))
            FontScale = Mathf.Clamp(FontScale - ZoomStep, 0.5f, 2f);
        GUILayout.Label($"{FontScale * 100f:0}%", _mutedStyle, GUILayout.Width(38f));
        if (GUILayout.Button("字+", GUILayout.Width(34f)))
            FontScale = Mathf.Clamp(FontScale + ZoomStep, 0.5f, 2f);
        DrawWindowVisibilityButton("整体", OverviewWindow);
        DrawWindowVisibilityButton("跨帧接触缓存", AabbWindow);
        DrawWindowVisibilityButton("跨子步接触", ContactWindow);
        DrawWindowVisibilityButton("设置", SettingsWindow);
        GUILayout.FlexibleSpace();
        if (GUILayout.Button(SimulationDebuggerRuntime.FreezeSnapshot ? "继续" : "冻结", GUILayout.Width(48f)))
            SimulationDebuggerRuntime.FreezeSnapshot = !SimulationDebuggerRuntime.FreezeSnapshot;
        if (GUILayout.Button(SimulationDebuggerRuntime.OverlayEnabled ? "场景图层" : "图层关闭", GUILayout.Width(68f)))
            SimulationDebuggerRuntime.OverlayEnabled = !SimulationDebuggerRuntime.OverlayEnabled;
        if (GUILayout.Button("重置布局", GUILayout.Width(68f)))
            ResetWindowLayout();
        if (GUILayout.Button("×", GUILayout.Width(24f)))
        {
            Visible = false;
            RefreshCaptureMask();
        }
        GUILayout.EndHorizontal();
        GUI.DragWindow(new Rect(0f, 0f, LauncherRect.width - 28f, 28f));
    }

    private void DrawWindowVisibilityButton(string label, SimulationDebuggerWindowState state)
    {
        bool next = GUILayout.Toggle(state.Visible, label, _tabStyle, GUILayout.Height(23f));
        if (next == state.Visible)
            return;
        state.Visible = next;
        RefreshCaptureMask();
    }

    private void DrawIndependentWindow(
        int id,
        SimulationDebuggerView view,
        SimulationDebuggerWindowState state,
        PublishedSimulationDiagnosticsSnapshot published)
    {
        if (state == null || !state.Visible)
            return;

        Rect returnedRect = GUI.Window(
            id,
            state.Rect,
            _ => DrawViewWindow(view, state, published),
            GUIContent.none,
            _windowStyle);

        // GUI.Window 返回的是窗口拖拽后的坐标；窗口内部的缩放手柄则直接修改
        // state.Rect 的宽高。必须在回调结束后读取新宽高，不能用进入 GUI.Window
        // 前保存的旧尺寸覆盖它。
        float resizedWidth = state.Rect.width;
        float resizedHeight = state.Rect.height;
        state.Rect = new Rect(
            returnedRect.x,
            returnedRect.y,
            Mathf.Max(320f, resizedWidth),
            Mathf.Max(220f, resizedHeight));
        state.Rect.x = Mathf.Clamp(state.Rect.x, -state.Rect.width + 90f, Screen.width - 80f);
        state.Rect.y = Mathf.Clamp(state.Rect.y, 0f, Screen.height - 35f);
    }

    private void DrawViewWindow(
        SimulationDebuggerView view,
        SimulationDebuggerWindowState state,
        PublishedSimulationDiagnosticsSnapshot published)
    {
        _currentView = view;
        _activeWindowState = state;
        _scroll = state.Scroll;
        _showDetails = state.ShowDetails;

        DrawWindowHeader(view, state);
        _scroll = GUILayout.BeginScrollView(_scroll, false, true);

        if (published == null)
        {
            GUILayout.Space(20f);
            GUILayout.Label("等待仿真诊断快照…", _sectionStyle);
            GUILayout.Label("确认单位移动系统正在运行，并且该窗口已启用采样。", _mutedStyle);
        }
        else
        {
            SimulationDebuggerFrameSnapshot snapshot = published.Frame;
            IncrementalContactPipelineSnapshot pipeline = published.Pipeline;
            DrawFrameStrip(snapshot);
            switch (view)
            {
                case SimulationDebuggerView.Overview:
                    DrawOverview(snapshot);
                    break;
                case SimulationDebuggerView.PersistentBroadPhase:
                    DrawPersistentBroadPhase(snapshot, pipeline);
                    break;
                case SimulationDebuggerView.TimestepContactSet:
                    DrawContactSet(snapshot, pipeline);
                    break;
                case SimulationDebuggerView.RuntimeSettings:
                    DrawSettingsSummary(snapshot);
                    break;
            }
        }

        GUILayout.EndScrollView();
        state.Scroll = _scroll;
        state.ShowDetails = _showDetails;
        HandleResize(state);
        GUI.DragWindow(new Rect(0f, 0f, state.Rect.width - 54f, 30f));
    }

    private void DrawWindowHeader(
        SimulationDebuggerView view,
        SimulationDebuggerWindowState state)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(ViewTitle(view), _headerStyle, GUILayout.ExpandWidth(true));
        if (view != SimulationDebuggerView.RuntimeSettings &&
            GUILayout.Button("映射到场景", GUILayout.Width(78f), GUILayout.Height(23f)))
        {
            SimulationDebuggerRuntime.WorldHeatmap = state.Heatmap;
            SimulationDebuggerRuntime.WorldOverlayView = view;
            SimulationDebuggerRuntime.OverlayEnabled = true;
        }
        if (GUILayout.Button("×", GUILayout.Width(25f), GUILayout.Height(23f)))
        {
            state.Visible = false;
            RefreshCaptureMask();
        }
        GUILayout.EndHorizontal();
    }

    private void HandleResize(SimulationDebuggerWindowState state)
    {
        Rect grip = new Rect(state.Rect.width - 18f, state.Rect.height - 18f, 18f, 18f);
        GUI.Label(grip, "◢", _mutedStyle);
        Event evt = Event.current;
        if (evt.type == EventType.MouseDown && grip.Contains(evt.mousePosition))
        {
            state.Resizing = true;
            state.ResizeStartMouse = evt.mousePosition;
            state.ResizeStartSize = new Vector2(state.Rect.width, state.Rect.height);
            evt.Use();
        }
        else if (evt.type == EventType.MouseDrag && state.Resizing)
        {
            Vector2 delta = evt.mousePosition - state.ResizeStartMouse;
            state.Rect.width = Mathf.Max(320f, state.ResizeStartSize.x + delta.x);
            state.Rect.height = Mathf.Max(220f, state.ResizeStartSize.y + delta.y);
            evt.Use();
        }
        else if (evt.type == EventType.MouseUp && state.Resizing)
        {
            state.Resizing = false;
            evt.Use();
        }
    }

    public static bool IsPointerOverDebugger(Vector2 inputScreenPosition)
    {
        SimulationDebuggerPanel panel = Instance;
        if (panel == null || !panel.Visible)
            return false;

        // Input.mousePosition 的原点在左下角，IMGUI Rect 的原点在左上角。
        Vector2 guiPoint = new Vector2(
            inputScreenPosition.x,
            Screen.height - inputScreenPosition.y);
        if (panel.LauncherRect.Contains(guiPoint))
            return true;
        return IsVisibleWindowHit(panel.OverviewWindow, guiPoint) ||
               IsVisibleWindowHit(panel.AabbWindow, guiPoint) ||
               IsVisibleWindowHit(panel.ContactWindow, guiPoint) ||
               IsVisibleWindowHit(panel.SettingsWindow, guiPoint);
    }

    private static bool IsVisibleWindowHit(
        SimulationDebuggerWindowState state,
        Vector2 guiPoint)
    {
        return state != null && state.Visible && state.Rect.Contains(guiPoint);
    }

    private bool IsGuiPointOverDebugger(Vector2 guiPoint)
    {
        if (!Visible)
            return false;
        if (LauncherRect.Contains(guiPoint))
            return true;
        return IsVisibleWindowHit(OverviewWindow, guiPoint) ||
               IsVisibleWindowHit(AabbWindow, guiPoint) ||
               IsVisibleWindowHit(ContactWindow, guiPoint) ||
               IsVisibleWindowHit(SettingsWindow, guiPoint);
    }

    private void EnsureWindowStates()
    {
        OverviewWindow ??= SimulationDebuggerWindowState.Create(
            new Rect(18f, 58f, 510f, 440f),
            SimulationDebuggerHeatmap.OverallPressure);
        AabbWindow ??= SimulationDebuggerWindowState.Create(
            new Rect(542f, 58f, 510f, 440f),
            SimulationDebuggerHeatmap.AabbBenefit);
        ContactWindow ??= SimulationDebuggerWindowState.Create(
            new Rect(18f, 512f, 510f, 440f),
            SimulationDebuggerHeatmap.ContactActivation);
        SettingsWindow ??= SimulationDebuggerWindowState.Create(
            new Rect(542f, 512f, 510f, 520f),
            SimulationDebuggerHeatmap.None);
    }

    private void ResetWindowLayout()
    {
        EnsureWindowStates();
        float gap = 12f;
        float top = 58f;
        float availableWidth = Mathf.Max(680f, Screen.width - 36f);
        float width = Mathf.Clamp((availableWidth - gap) * 0.5f, 320f, 620f);
        float height = Mathf.Clamp((Screen.height - top - gap - 24f) * 0.5f, 220f, 520f);
        OverviewWindow.Rect = new Rect(18f, top, width, height);
        AabbWindow.Rect = new Rect(18f + width + gap, top, width, height);
        ContactWindow.Rect = new Rect(18f, top + height + gap, width, height);
        SettingsWindow.Rect = new Rect(18f + width + gap, top + height + gap, width, height);
        OverviewWindow.Visible = true;
        AabbWindow.Visible = true;
        ContactWindow.Visible = true;
        SettingsWindow.Visible = true;
        RefreshCaptureMask();
    }

    private static string ViewTitle(SimulationDebuggerView view)
    {
        return view switch
        {
            SimulationDebuggerView.Overview => "整体仿真",
            SimulationDebuggerView.PersistentBroadPhase => "Timestep 候选缓存",
            SimulationDebuggerView.TimestepContactSet => "Substep 接触集缓存",
            _ => "运行时设置"
        };
    }

    private void DrawFrameStrip(SimulationDebuggerFrameSnapshot snapshot)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label($"帧 {snapshot.FrameId}", _mutedStyle);
        GUILayout.Space(10f);
        GUILayout.Label($"单位 {snapshot.Overview.UnitCount:N0}", _mutedStyle);
        GUILayout.Space(10f);
        GUILayout.Label(
            $"实验 {snapshot.Experiment.ShortId} · 配置 #{snapshot.Experiment.ConfigurationId}",
            _mutedStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(
            $"时间步 {snapshot.DeltaTime * 1000f:0.00} 毫秒  ·  " +
            $"{snapshot.SubstepCount} 子步  ·  {snapshot.IterationCount} 轮迭代",
            _mutedStyle);
        GUILayout.EndHorizontal();
        DrawLocalRecordingStatus();
        if (snapshot.Experiment.IsWarmup != 0)
        {
            GUILayout.Label(
                $"配置切换后的预热阶段：第 {snapshot.Experiment.FramesSinceChanged + 1} 帧，暂不建议纳入正式对比。",
                _mutedStyle);
        }
        GUILayout.Space(6f);
    }

    private void DrawOverview(SimulationDebuggerFrameSnapshot snapshot)
    {
        SimulationOverviewMetrics metrics = snapshot.Overview;
        DrawStatus("接触管线总控", metrics.Health, OverviewStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric(
            "管线总耗时",
            AvailableNanoseconds(metrics.TimingAvailable, metrics.SolverNanoseconds),
            "接触管线 Job 时间跨度，不是整帧时间");
        DrawMetric(
            "Broad Phase",
            AvailableNanoseconds(
                metrics.StageTimingAvailable,
                metrics.BroadPhaseNanoseconds),
            "候选生成、代理校验与缓存维护");
        DrawMetric(
            "Narrow Phase",
            AvailableNanoseconds(
                metrics.StageTimingAvailable,
                metrics.NarrowPhaseNanoseconds),
            "候选几何分类与接触构造");
        DrawMetric(
            "约束求解阶段",
            metrics.SolverSkipReason != ContactSolverSkipReason.None
                ? "已跳过"
                : AvailableNanoseconds(
                    metrics.TimingAvailable,
                    metrics.IterationNanoseconds),
            metrics.SolverSkipReason != ContactSolverSkipReason.None
                ? $"证书校验失败：{SolverSkipReasonLabel(metrics.SolverSkipReason)}"
                : $"墙约束、XPBD 接触投影及恢复处理 · " +
                  $"{snapshot.SubstepCount} 子步 × {snapshot.IterationCount} 轮");
        GUILayout.EndHorizontal();

        DrawMultiTrendChart(
            ref _overviewChartTexture,
            "阶段耗时趋势（最近有效采样）",
            new[]
            {
                SimulationDebuggerRuntime.GetSolverHistory(),
                SimulationDebuggerRuntime.GetBroadPhaseHistory(),
                SimulationDebuggerRuntime.GetNarrowPhaseHistory(),
                SimulationDebuggerRuntime.GetXpbdHistory()
            },
            new[] { "总计", "Broad", "Narrow", "约束" },
            new[]
            {
                new Color(0.35f, 0.78f, 1f),
                new Color(0.28f, 0.85f, 0.45f),
                new Color(1f, 0.67f, 0.2f),
                new Color(0.78f, 0.45f, 1f)
            },
            "ms",
            112);
        DrawTrendRow(
            "管线总耗时",
            SimulationDebuggerRuntime.GetSolverTrend(),
            "0.000",
            " ms");
        DrawTrendRow(
            "Broad Phase",
            SimulationDebuggerRuntime.GetBroadPhaseTrend(),
            "0.000",
            " ms");
        DrawTrendRow(
            "Narrow Phase",
            SimulationDebuggerRuntime.GetNarrowPhaseTrend(),
            "0.000",
            " ms");
        DrawTrendRow(
            "约束求解阶段",
            SimulationDebuggerRuntime.GetXpbdTrend(),
            "0.000",
            " ms");

        DrawHeatmapSelector(
            "整体热力图",
            new[]
            {
                SimulationDebuggerHeatmap.OverallPressure,
                SimulationDebuggerHeatmap.UnitDensity,
                SimulationDebuggerHeatmap.SolverCorrection
            });
        DrawHeatmapLegend();
        DrawPanelGridHeatmap(snapshot);
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("阶段耗时", _sectionStyle);
        DrawTimeBreakdown(metrics);

        GUILayout.Space(6f);
        GUILayout.Label("接触工作量", _sectionStyle);
        DrawContactWorkload(metrics);

        GUILayout.Space(6f);
        GUILayout.Label("稳定性", _sectionStyle);
        DrawDetailRow(
            "最大接触纠偏",
            AvailableWorldUnits(metrics.StabilityAvailable, metrics.MaxContactCorrection));
        DrawDetailRow(
            "最大墙体纠偏",
            AvailableWorldUnits(metrics.StabilityAvailable, metrics.MaxWallCorrection));
        DrawDetailRow(
            "最大速度变化",
            AvailableWorldUnitsPerSecond(metrics.StabilityAvailable, metrics.MaxVelocityChange));

        GUILayout.Space(6f);
        GUILayout.Label("稳定性与工作量", _sectionStyle);
        DrawDetailRow("参与单位", metrics.UnitCount.ToString("N0"));
        DrawDetailRow(
            "当前实际 / 预测接触",
            $"{metrics.CurrentActualPairCount:N0} / {metrics.CurrentPredictivePairCount:N0}");
        DrawDetailRow(
            "候选 / 接触评估累计",
            $"{metrics.CandidatePairCount:N0} / {metrics.ContactPairCount:N0}");
        DrawDetailRow(
            "最大接触纠偏",
            AvailableWorldUnits(metrics.StabilityAvailable, metrics.MaxContactCorrection));
    }

    private void DrawPersistentBroadPhase(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline)
    {
        IncrementalContactPipelineStatistics statistics = pipeline.Statistics;
        bool cacheEnabled = snapshot.EffectiveSettings.EnablePersistentContactCache != 0;
        bool hasPipelineSnapshot = statistics.Timestep != 0;
        bool oracleAvailable =
            snapshot.EffectiveSettings.EnableDiagnostics != 0 &&
            hasPipelineSnapshot;
        SimulationDebuggerCacheComparison comparison =
            SimulationDebuggerRuntime.GetTimestepCacheComparison();
        SimulationDebuggerHealth health;
        string status;
        if (!cacheEnabled)
        {
            health = SimulationDebuggerHealth.Disabled;
            status = "缓存已关闭：正在收集同配置 OFF 基线；开启后才能计算净收益。";
        }
        else if (!hasPipelineSnapshot)
        {
            health = SimulationDebuggerHealth.Warning;
            status = "等待增量接触管线发布首个时间步快照。";
        }
        else if (oracleAvailable &&
                 (statistics.OracleMissingPairCount != 0 ||
                  statistics.OracleMismatch != 0))
        {
            health = SimulationDebuggerHealth.Critical;
            status = "Oracle 发现最终接触视图漏对；检查缓存修复与 Fallback。";
        }
        else if (statistics.FullRebuildCount != 0)
        {
            health = SimulationDebuggerHealth.Warning;
            status = "本时间步发生完整重建；缓存仍正确，但本步收益可能下降。";
        }
        else
        {
            health = SimulationDebuggerHealth.Healthy;
            status = $"缓存开启 · {pipeline.Mode} · {ComparisonStatus(comparison)}";
        }

        DrawStatus("跨时间步候选缓存", health, status);
        GUILayout.Space(8f);

        long maintenanceNanoseconds =
            statistics.ProxyValidationNanoseconds +
            statistics.PersistentPairMappingNanoseconds +
            statistics.LocalBroadPhaseNanoseconds +
            statistics.PairDiffNanoseconds +
            statistics.FallbackNanoseconds;
        GUILayout.BeginHorizontal();
        DrawMetric(
            "缓存维护耗时",
            hasPipelineSnapshot ? Nanoseconds(maintenanceNanoseconds) : "--",
            "校验、局部查询、Pair Diff、映射与回退");
        DrawMetric(
            "管线净变化",
            ComparisonTimeDelta(comparison),
            "同配置 OFF/ON 各 30 次有效采样的 P50 差值");
        DrawMetric(
            "候选评估变化",
            ComparisonPairDelta(comparison),
            "负数表示需要评估的候选 Pair 减少");
        DrawMetric(
            "Oracle 最终漏对",
            oracleAvailable
                ? statistics.OracleMissingPairCount.ToString("N0")
                : "未验证",
            oracleAvailable
                ? $"Oracle 对数 {statistics.OraclePairCount:N0}"
                : "开启深度正确性诊断后才有真值");
        GUILayout.EndHorizontal();

        DrawMultiTrendChart(
            ref _timestepCostChartTexture,
            "成本趋势",
            new[]
            {
                SimulationDebuggerRuntime.GetSolverHistory(),
                SimulationDebuggerRuntime.GetPersistentMaintenanceHistory()
            },
            new[] { "管线总计", "缓存维护" },
            new[]
            {
                new Color(0.35f, 0.78f, 1f),
                new Color(1f, 0.66f, 0.2f)
            },
            "ms",
            86);
        DrawMultiTrendChart(
            ref _timestepLoadChartTexture,
            "候选负载趋势",
            new[] { SimulationDebuggerRuntime.GetPersistentCandidateHistory() },
            new[] { "持久候选对" },
            new[] { new Color(0.3f, 0.88f, 0.5f) },
            " pair",
            70);

        DrawHeatmapSelector(
            "候选缓存热力图",
            new[]
            {
                SimulationDebuggerHeatmap.AabbBenefit,
                SimulationDebuggerHeatmap.AabbSlack,
                SimulationDebuggerHeatmap.CandidateExpansion,
                SimulationDebuggerHeatmap.EscapeRisk
            });
        DrawHeatmapLegend();
        DrawPanelGridHeatmap(snapshot);
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("成本与避免工作", _sectionStyle);
        DrawDetailRow("当前模式", hasPipelineSnapshot ? pipeline.Mode.ToString() : "--");
        DrawDetailRow(
            "代理校验 / Pair 映射",
            hasPipelineSnapshot
                ? $"{Nanoseconds(statistics.ProxyValidationNanoseconds)} / " +
                  $"{Nanoseconds(statistics.PersistentPairMappingNanoseconds)}"
                : "--");
        DrawDetailRow(
            "局部 Broad / Pair Diff",
            hasPipelineSnapshot
                ? $"{Nanoseconds(statistics.LocalBroadPhaseNanoseconds)} / " +
                  $"{Nanoseconds(statistics.PairDiffNanoseconds)}"
                : "--");
        DrawDetailRow(
            "完整扫描 / Fallback",
            hasPipelineSnapshot
                ? $"{Nanoseconds(statistics.FullSweepSourceNanoseconds)} / " +
                  $"{Nanoseconds(statistics.FallbackNanoseconds)}"
                : "--");
        DrawDetailRow(
            "分类复用 / 跳过",
            hasPipelineSnapshot
                ? $"{statistics.ClassificationReuseCount:N0} / " +
                  $"{statistics.ClassificationSkippedCount:N0}"
                : "--");

        GUILayout.Space(6f);
        GUILayout.Label("拓扑与正确性", _sectionStyle);
        DrawDetailRow("拓扑脏体 / 总代理", hasPipelineSnapshot
            ? $"{statistics.TopologyDirtyBodyCount:N0} / {statistics.ProxyCount:N0}"
            : "--");
        DrawDetailRow("运动脏体 / 逃逸", hasPipelineSnapshot
            ? $"{statistics.MotionDirtyBodyCount:N0} / {statistics.CorrectedEscapeBodyCount:N0}"
            : "--");
        DrawDetailRow("新增 / 移除 / 保留 Pair", hasPipelineSnapshot
            ? $"{statistics.NeighborPairAddedCount} / {statistics.NeighborPairRemovedCount} / {statistics.NeighborPairRetainedCount}" : "--");
        DrawDetailRow("完整重建 / 局部修复", hasPipelineSnapshot
            ? $"{statistics.FullRebuildCount} / {statistics.IncrementalRepairCount}" : "--");
        DrawDetailRow("干净代理 / Pair 保留率", hasPipelineSnapshot
            ? $"{pipeline.CleanProxyRatio:P1} / {pipeline.RetainedNeighborPairRatio:P1}"
            : "--");
        DrawDetailRow(
            "Oracle 缺失 / 额外",
            oracleAvailable
                ? $"{statistics.OracleMissingPairCount:N0} / " +
                  $"{statistics.OracleExtraPairCount:N0}"
                : "未验证");
        DrawDetailRow(
            "软避让 Oracle 漏对",
            oracleAvailable
                ? statistics.SoftAvoidanceOracleMissingPairCount.ToString("N0")
                : "未验证");
        DrawComparisonDetails(comparison);
    }

    private void DrawContactSet(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline)
    {
        TimestepContactSetMetrics metrics = snapshot.ContactSet;
        PredictiveDiscContactStatistics statistics = pipeline.SolverStatistics;
        IncrementalContactPipelineStatistics incremental = pipeline.Statistics;
        SimulationDebuggerCacheComparison comparison =
            SimulationDebuggerRuntime.GetSubstepCacheComparison();
        bool cacheEnabled = metrics.CacheEnabled != 0;
        bool oracleAvailable = metrics.OracleAvailable != 0;
        DrawStatus(
            "跨子步接触集缓存",
            metrics.Health,
            cacheEnabled
                ? $"整步唯一接触集复用中 · {ComparisonStatus(comparison)}"
                : "缓存已关闭：每个 Substep 重新构建接触集，并收集 OFF 基线。");
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric(
            cacheEnabled ? "整步唯一接触集" : "当前 Substep 接触集",
            metrics.ContactSetSize.ToString("N0"),
            cacheEnabled
                ? "整个 Timestep 内唯一 Pair 数"
                : "关闭缓存时只表示最后一次构建，不参与利用率");
        DrawMetric(
            "唯一激活利用率",
            snapshot.Overview.SolverSkipReason != ContactSolverSkipReason.None
                ? "求解已跳过"
                : metrics.ActivationAvailable != 0
                ? Percent(metrics.ActivationRatio)
                : "不适用",
            snapshot.Overview.SolverSkipReason != ContactSolverSkipReason.None
                ? SolverSkipReasonLabel(snapshot.Overview.SolverSkipReason)
                : "至少一个 Substep 中产生有效 XPBD 约束的唯一 Pair");
        DrawMetric(
            "避免重复构建",
            cacheEnabled
                ? $"{metrics.AvoidedContactGenerationCount} 次"
                : "0 次",
            $"实际构建 {metrics.ContactGenerationCount} / {metrics.SubstepCount} 次");
        DrawMetric(
            "管线净变化",
            ComparisonTimeDelta(comparison),
            "要求跨时间步缓存关闭，OFF/ON 各 30 次有效采样");
        GUILayout.EndHorizontal();

        if (metrics.FallbackAddedPairCount > 0 ||
            (oracleAvailable && incremental.OracleMissingPairCount > 0))
        {
            GUILayout.Space(5f);
            DrawStatus(
                "正确性事件",
                incremental.OracleMissingPairCount > 0
                    ? SimulationDebuggerHealth.Critical
                    : SimulationDebuggerHealth.Warning,
                $"Fallback 补充 {metrics.FallbackAddedPairCount:N0} Pair · " +
                $"Oracle 最终漏对 " +
                $"{(oracleAvailable ? incremental.OracleMissingPairCount.ToString("N0") : "未验证")}");
        }

        DrawMultiTrendChart(
            ref _substepCostChartTexture,
            "成本趋势",
            new[]
            {
                SimulationDebuggerRuntime.GetSolverHistory(),
                SimulationDebuggerRuntime.GetContactSetBuildHistory()
            },
            new[] { "管线总计", "接触集构建" },
            new[]
            {
                new Color(0.35f, 0.78f, 1f),
                new Color(1f, 0.66f, 0.2f)
            },
            "ms",
            86);
        DrawMultiTrendChart(
            ref _substepLoadChartTexture,
            "接触集利用趋势",
            new[]
            {
                SimulationDebuggerRuntime.GetContactSetSizeHistory(),
                SimulationDebuggerRuntime.GetActiveContactHistoryObj()
            },
            new[] { "唯一接触集", "已激活" },
            new[]
            {
                new Color(0.45f, 0.65f, 1f),
                new Color(0.3f, 0.9f, 0.48f)
            },
            " pair",
            76);

        DrawHeatmapSelector(
            "接触缓存热力图",
            new[]
            {
                SimulationDebuggerHeatmap.ContactActivation,
                SimulationDebuggerHeatmap.ContactWaste,
                SimulationDebuggerHeatmap.ContactSupplementRisk
            });
        DrawHeatmapLegend();
        DrawPanelGridHeatmap(snapshot);
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("构建与分类成本", _sectionStyle);
        DrawDetailRow("接触集构建", Nanoseconds(metrics.BuildNanoseconds));
        DrawDetailRow("Narrow 分类", Nanoseconds(metrics.ClassificationNanoseconds));
        DrawDetailRow("Substep 激活调度", Nanoseconds(metrics.ActivationNanoseconds));
        DrawDetailRow("Fallback", Nanoseconds(metrics.FallbackNanoseconds));
        DrawDetailRow(
            "构建 / 分类 / Substep 使用次数",
            $"{statistics.TimestepContactSetBuildCount:N0} / " +
            $"{statistics.TimestepContactSetClassificationPassCount:N0} / " +
            $"{statistics.TimestepContactSetSubstepUseCount:N0}");

        GUILayout.Space(6f);
        GUILayout.Label("接触集组成与正确性", _sectionStyle);
        DrawDetailRow(
            "Actual / Predictive",
            $"{incremental.CurrentActualPairCount:N0} / " +
            $"{incremental.CurrentPredictivePairCount:N0}");
        DrawDetailRow(
            "Approaching / Dormant",
            $"{incremental.CurrentApproachingPairCount:N0} / " +
            $"{incremental.CurrentDormantPairCount:N0}");
        DrawDetailRow(
            "唯一激活 / 唯一纠偏",
            $"{incremental.UniqueActivatedPairCount:N0} / " +
            $"{incremental.UniqueCorrectedPairCount:N0}");
        DrawDetailRow(
            "缓存但未激活",
            metrics.ActivationAvailable != 0
                ? metrics.InactiveContactCount.ToString("N0")
                : "不适用");
        DrawDetailRow(
            "完整重建 / Fallback 补充",
            $"{metrics.FullRebuildCount:N0} / " +
            $"{metrics.FallbackAddedPairCount:N0}");
        DrawDetailRow(
            "Oracle 最终缺失 / 额外",
            oracleAvailable
                ? $"{incremental.OracleMissingPairCount:N0} / " +
                  $"{incremental.OracleExtraPairCount:N0}"
                : "未验证");
        DrawComparisonDetails(comparison);
    }

    private void DrawSettingsSummary(SimulationDebuggerFrameSnapshot snapshot)
    {
        DrawStatus(
            "运行时设置",
            SimulationDebuggerHealth.Healthy,
            $"有效配置：Timestep " +
            $"{OnOff(snapshot.EffectiveSettings.EnablePersistentContactCache)} · " +
            $"Substep {OnOff(snapshot.EffectiveSettings.EnableTimestepContactSetCache)} · " +
            $"{snapshot.SubstepCount} 子步 × {snapshot.IterationCount} 轮");
        GUILayout.Space(8f);

        SimulationDebuggerEffectiveSettings draft = snapshot.EffectiveSettings;

        GUILayout.Label("运行算法", _sectionStyle);
        GUILayout.Label(
            "跨时间步候选缓存依赖跨子步接触集缓存；关闭 Substep 缓存会同时关闭 Timestep 缓存。",
            _mutedStyle);
        draft.EnablePersistentContactCache = DrawToggle(
            "跨时间步候选缓存",
            draft.EnablePersistentContactCache,
            snapshot.EffectiveSettings.EnablePersistentContactCache);
        draft.EnableTimestepContactSetCache = DrawToggle(
            "跨子步接触集缓存",
            draft.EnableTimestepContactSetCache,
            snapshot.EffectiveSettings.EnableTimestepContactSetCache);
        if (draft.EnablePersistentContactCache != 0 && draft.EnableTimestepContactSetCache == 0)
        {
            bool userTurnedOffSubstepCache =
                snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0;
            if (userTurnedOffSubstepCache)
            {
                draft.EnablePersistentContactCache = 0;
                GUILayout.Label("已同步关闭跨时间步候选缓存。", _mutedStyle);
            }
            else
            {
                draft.EnableTimestepContactSetCache = 1;
                GUILayout.Label("已同步开启跨子步接触集缓存。", _mutedStyle);
            }
        }

        draft.SubstepCount = DrawIntSlider(
            "XPBD 子步数量",
            draft.SubstepCount,
            snapshot.EffectiveSettings.SubstepCount,
            1,
            16);
        draft.IterationCount = DrawIntSlider(
            "每子步迭代次数",
            draft.IterationCount,
            snapshot.EffectiveSettings.IterationCount,
            1,
            24);

        GUILayout.BeginHorizontal();
        GUILayout.Label("接触位置求解器", _mutedStyle, GUILayout.Width(170f));
        draft.ContactPositionSolver = GUILayout.SelectionGrid(
            Mathf.Clamp(draft.ContactPositionSolver, 0, 1),
            new[] { "Gauss-Seidel", "Jacobi" },
            2,
            _tabStyle);
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("软避让求解器", _mutedStyle, GUILayout.Width(170f));
        draft.SoftAvoidanceVelocitySolver = GUILayout.SelectionGrid(
            Mathf.Clamp(draft.SoftAvoidanceVelocitySolver, 0, 1),
            new[] { "预测引导", "RVO 互惠避让" },
            2,
            _tabStyle);
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();

        GUILayout.Space(8f);
        GUILayout.Label("性能对比", _sectionStyle);
        GUILayout.Label(
            "系统自动按相同单位数、时间步和求解配置积累 OFF/ON 样本；预热样本不会进入对比。",
            _mutedStyle);
        DrawComparisonSettingsRow(
            "Timestep 缓存",
            SimulationDebuggerRuntime.GetTimestepCacheComparison());
        DrawComparisonSettingsRow(
            "Substep 缓存",
            SimulationDebuggerRuntime.GetSubstepCacheComparison());
        DrawDetailRow(
            "当前采样状态",
            snapshot.Experiment.IsWarmup != 0
                ? $"预热 {snapshot.Experiment.FramesSinceChanged + 1} / " +
                  $"{SimulationDebuggerRuntime.ExperimentWarmupFrames}"
                : "有效样本");
        if (GUILayout.Button("清除性能对比样本", GUILayout.Height(24f)))
            SimulationDebuggerRuntime.ClearCacheComparisons();

        DrawDetailsToggle();
        if (_showDetails)
        {
            bool previousEnabled = GUI.enabled;
            GUILayout.Space(6f);
            GUILayout.Label("XPBD 与预测接触", _sectionStyle);
            draft.Compliance = DrawFloatSlider(
                "柔顺度",
                draft.Compliance,
                snapshot.EffectiveSettings.Compliance,
                0f,
                0.1f,
                "0.0000");
            draft.EnablePredictivePairGeneration = DrawToggle(
                "生成预测接触对",
                draft.EnablePredictivePairGeneration,
                snapshot.EffectiveSettings.EnablePredictivePairGeneration);
            GUI.enabled =
                previousEnabled && draft.EnablePredictivePairGeneration != 0;
            draft.EnablePredictiveContacts = DrawToggle(
                "启用预测半空间约束",
                draft.EnablePredictiveContacts,
                snapshot.EffectiveSettings.EnablePredictiveContacts);
            draft.PredictiveSkin = DrawFloatSlider(
                "预测接触外扩距离",
                draft.PredictiveSkin,
                snapshot.EffectiveSettings.PredictiveSkin,
                0f,
                3f,
                "0.00");
            draft.TimestepContactMargin = DrawFloatSlider(
                "时间步接触包络余量",
                draft.TimestepContactMargin,
                snapshot.EffectiveSettings.TimestepContactMargin,
                0f,
                5f,
                "0.00");
            GUI.enabled = previousEnabled;

            GUILayout.Space(6f);
            GUILayout.Label("软避让参数", _sectionStyle);
            draft.SoftAvoidanceResponseRate = DrawFloatSlider(
                "响应速度",
                draft.SoftAvoidanceResponseRate,
                snapshot.EffectiveSettings.SoftAvoidanceResponseRate,
                0f,
                20f,
                "0.00");
            draft.SoftAvoidanceShell = DrawFloatSlider(
                "表面缓冲距离",
                draft.SoftAvoidanceShell,
                snapshot.EffectiveSettings.SoftAvoidanceShell,
                0f,
                4f,
                "0.00");
            draft.SettledSoftAvoidanceMultiplier = DrawFloatSlider(
                "已到达单位避让倍率",
                draft.SettledSoftAvoidanceMultiplier,
                snapshot.EffectiveSettings.SettledSoftAvoidanceMultiplier,
                0f,
                2f,
                "0.00");
            GUI.enabled =
                previousEnabled && draft.SoftAvoidanceVelocitySolver == 1;
            draft.RvoTimeHorizon = DrawFloatSlider(
                "RVO 预测时间",
                draft.RvoTimeHorizon,
                snapshot.EffectiveSettings.RvoTimeHorizon,
                0.05f,
                5f,
                "0.00");
            GUI.enabled = previousEnabled;

            GUILayout.Space(6f);
            GUILayout.Label("缓存参数", _sectionStyle);
            GUI.enabled =
                previousEnabled && draft.EnablePersistentContactCache != 0;
            draft.PersistentGuardEnvelopeMargin = DrawFloatSlider(
                "跨时间步预测包络余量",
                draft.PersistentGuardEnvelopeMargin,
                snapshot.EffectiveSettings.PersistentGuardEnvelopeMargin,
                0f,
                5f,
                "0.00");
            GUI.enabled = previousEnabled;

            GUILayout.Space(6f);
            GUILayout.Label("深度正确性诊断", _sectionStyle);
            draft.EnableDiagnostics = DrawToggle(
                "逐 Pair / Oracle 诊断",
                draft.EnableDiagnostics,
                snapshot.EffectiveSettings.EnableDiagnostics);
            GUILayout.Label(
                "Oracle 为 O(N²) 验证，只用于正确性检查；基础阶段计时不依赖此开关。",
                _mutedStyle);
            draft.EnableAdaptiveFatAabb = DrawToggle(
                "热点网格诊断（非执行路径）",
                draft.EnableAdaptiveFatAabb,
                snapshot.EffectiveSettings.EnableAdaptiveFatAabb);
            GUI.enabled = previousEnabled && draft.EnableAdaptiveFatAabb != 0;
            draft.AdaptiveDetectionCellSpan = DrawIntSlider(
                "检测格子跨度",
                draft.AdaptiveDetectionCellSpan,
                snapshot.EffectiveSettings.AdaptiveDetectionCellSpan,
                1,
                8);
            draft.AdaptiveMinimumUnitsPerCell = DrawIntSlider(
                "每格最少单位数",
                draft.AdaptiveMinimumUnitsPerCell,
                snapshot.EffectiveSettings.AdaptiveMinimumUnitsPerCell,
                1,
                32);
            draft.AdaptiveMinimumUnitsPerRegion = DrawIntSlider(
                "每区最少单位数",
                draft.AdaptiveMinimumUnitsPerRegion,
                snapshot.EffectiveSettings.AdaptiveMinimumUnitsPerRegion,
                1,
                128);
            draft.AdaptiveEnableScore = DrawFloatSlider(
                "启用阈值",
                draft.AdaptiveEnableScore,
                snapshot.EffectiveSettings.AdaptiveEnableScore,
                0f,
                1f,
                "0.00");
            draft.AdaptiveDisableScore = DrawFloatSlider(
                "关闭阈值",
                draft.AdaptiveDisableScore,
                snapshot.EffectiveSettings.AdaptiveDisableScore,
                0f,
                draft.AdaptiveEnableScore,
                "0.00");
            GUI.enabled = previousEnabled;

            GUILayout.Space(6f);
            GUILayout.Label("采样与显示", _sectionStyle);
            SimulationDebuggerRuntime.SummarySampleIntervalFrames = DrawIntSlider(
                "汇总采样间隔（帧）",
                SimulationDebuggerRuntime.SummarySampleIntervalFrames,
                SimulationDebuggerRuntime.SummarySampleIntervalFrames,
                1,
                30);
            SimulationDebuggerRuntime.SpatialSampleIntervalFrames = DrawIntSlider(
                "空间采样间隔（帧）",
                SimulationDebuggerRuntime.SpatialSampleIntervalFrames,
                SimulationDebuggerRuntime.SpatialSampleIntervalFrames,
                1,
                30);
            SimulationDebuggerRuntime.ExperimentWarmupFrames = DrawIntSlider(
                "实验预热帧数",
                SimulationDebuggerRuntime.ExperimentWarmupFrames,
                SimulationDebuggerRuntime.ExperimentWarmupFrames,
                0,
                300);
            SimulationDebuggerRuntime.MaximumVisualizedPairs = DrawIntSlider(
                "最多绘制接触线",
                SimulationDebuggerRuntime.MaximumVisualizedPairs,
                SimulationDebuggerRuntime.MaximumVisualizedPairs,
                1,
                128);
            SimulationDebuggerRuntime.HeatmapOpacity = DrawFloatSlider(
                "场景热力图透明度",
                SimulationDebuggerRuntime.HeatmapOpacity,
                SimulationDebuggerRuntime.HeatmapOpacity,
                0f,
                0.8f,
                "0.00");
            SimulationDebuggerRuntime.SlowTimeScale = DrawFloatSlider(
                "选中单位时减缓倍率",
                SimulationDebuggerRuntime.SlowTimeScale,
                SimulationDebuggerRuntime.SlowTimeScale,
                0.01f,
                1f,
                "0.00");
        }

        // 自动提交：每帧检查 draft 是否与有效值有差异，有则提交。
        if (!draft.Equals(snapshot.EffectiveSettings))
            SimulationDebuggerRuntime.SubmitSettings(draft);
    }

    private void DrawMultiTrendChart(
        ref Texture2D texture,
        string title,
        SimulationDebuggerHistory[] histories,
        string[] labels,
        Color[] colors,
        string unit,
        int height)
    {
        if (histories == null || histories.Length == 0)
            return;

        int width = Mathf.Clamp(
            Mathf.RoundToInt(
                (_activeWindowState?.Rect.width ?? 460f) - 40f),
            180,
            640);
        if (texture == null || texture.width != width || texture.height != height)
        {
            DestroyRuntimeTexture(ref texture);
            texture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                hideFlags = HideFlags.HideAndDontSave,
                filterMode = FilterMode.Bilinear
            };
        }

        Color bg = new Color(0.06f, 0.07f, 0.09f, 1f);
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
            pixels[i] = bg;

        Color gridColor = new Color(0.12f, 0.14f, 0.18f);
        int gridStep = Mathf.Max(1, height / 4);
        for (int y = 0; y < height; y += gridStep)
            for (int x = 0; x < width; x++)
                pixels[y * width + x] = gridColor;

        int seriesCount = Mathf.Min(
            Mathf.Min(histories.Length, labels?.Length ?? 0),
            Mathf.Min(colors?.Length ?? 0, 4));
        int[] counts = new int[seriesCount];
        float min = float.MaxValue;
        float max = float.MinValue;
        for (int series = 0; series < seriesCount; series++)
        {
            SimulationDebuggerHistory history = histories[series];
            if (history == null)
                continue;
            float[] buffer = GetChartBuffer(series);
            int count = history.CopyLatestTo(
                buffer,
                Mathf.Min(width, buffer.Length));
            counts[series] = count;
            for (int i = 0; i < count; i++)
            {
                float value = buffer[i];
                min = Mathf.Min(min, value);
                max = Mathf.Max(max, value);
            }
        }

        bool hasSamples = min != float.MaxValue;
        if (hasSamples)
        {
            if (max <= min)
                max = min + 0.0001f;
            for (int series = 0; series < seriesCount; series++)
            {
                int count = counts[series];
                if (count <= 0)
                    continue;
                float[] buffer = GetChartBuffer(series);
                int previousX = count == 1 ? width - 1 : 0;
                int previousY = ChartY(buffer[0], min, max, height);
                if (count == 1)
                {
                    DrawChartLine(
                        pixels,
                        width,
                        height,
                        previousX,
                        previousY,
                        previousX,
                        previousY,
                        colors[series]);
                    continue;
                }
                for (int i = 1; i < count; i++)
                {
                    int x = Mathf.RoundToInt(
                        i * (width - 1f) / (count - 1f));
                    int y = ChartY(buffer[i], min, max, height);
                    DrawChartLine(
                        pixels,
                        width,
                        height,
                        previousX,
                        previousY,
                        x,
                        y,
                        colors[series]);
                    previousX = x;
                    previousY = y;
                }
            }
        }

        texture.SetPixels(pixels);
        texture.Apply();

        GUILayout.Space(7f);
        GUILayout.BeginHorizontal();
        GUILayout.Label(title, _sectionStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(
            hasSamples ? $"{min:0.###}…{max:0.###} {unit}" : "暂无有效采样",
            _mutedStyle);
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        for (int series = 0; series < seriesCount; series++)
        {
            Color saved = GUI.color;
            GUI.color = colors[series];
            float current = histories[series]?.Current ?? 0f;
            GUILayout.Label(
                $"{labels[series]} {current:0.###}",
                _mutedStyle,
                GUILayout.ExpandWidth(false));
            GUI.color = saved;
        }
        GUILayout.EndHorizontal();
        GUILayout.Box(
            texture,
            GUIStyle.none,
            GUILayout.Width(width),
            GUILayout.Height(height));
    }

    private float[] GetChartBuffer(int index)
    {
        return index switch
        {
            0 => _chartBufferA,
            1 => _chartBufferB,
            2 => _chartBufferC,
            _ => _chartBufferD
        };
    }

    private static int ChartY(float value, float min, float max, int height)
    {
        float normalized = Mathf.InverseLerp(min, max, value);
        return Mathf.Clamp(
            Mathf.RoundToInt(normalized * (height - 1)),
            0,
            height - 1);
    }

    private static void DrawChartLine(
        Color[] pixels,
        int width,
        int height,
        int x0,
        int y0,
        int x1,
        int y1,
        Color color)
    {
        int steps = Mathf.Max(Mathf.Abs(x1 - x0), Mathf.Abs(y1 - y0));
        steps = Mathf.Max(1, steps);
        for (int step = 0; step <= steps; step++)
        {
            float t = step / (float)steps;
            int x = Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(x0, x1, t)), 0, width - 1);
            int y = Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(y0, y1, t)), 0, height - 1);
            pixels[y * width + x] = color;
            if (y > 0)
                pixels[(y - 1) * width + x] = color;
        }
    }

    private static void DrawTrendRow(
        string label,
        SimulationDebuggerTrend trend,
        string format,
        string unit = "",
        bool lowerIsBetter = true)
    {
        Color savedColor = GUI.color;
        GUILayout.BeginHorizontal();

        // 趋势箭头颜色
        GUI.color = trend.Direction switch
        {
            TrendDirection.Improving => lowerIsBetter ? Color.green : Color.red,
            TrendDirection.Degrading => lowerIsBetter ? Color.red : Color.green,
            _ => Color.gray
        };
        GUILayout.Label(trend.DirectionGlyph, GUILayout.Width(16f));

        GUI.color = savedColor;
        GUILayout.Label($"{label}", GUILayout.Width(120f));

        GUILayout.Label(
            trend.SampleCount > 0
                ? $"当前 {trend.Current.ToString(format)}{unit}  ·  " +
                  $"P50 {trend.Median.ToString(format)}{unit}  ·  " +
                  $"P95 {trend.Percentile95.ToString(format)}{unit}"
                : $"---",
            GUILayout.ExpandWidth(true));

        GUILayout.EndHorizontal();
        GUI.color = savedColor;
    }

    private static string OnOff(byte enabled) => enabled != 0 ? "开" : "关";

    private static string ComparisonStatus(
        SimulationDebuggerCacheComparison comparison)
    {
        int required = SimulationDebuggerRuntime.CacheComparisonMinimumSamples;
        if (comparison.Eligible == 0)
            return "当前组合不可建立独立对比";
        if (comparison.BaselineAvailable == 0)
            return $"OFF 基线 {comparison.BaselineSampleCount}/{required}";
        if (comparison.ComparisonAvailable == 0)
            return comparison.TargetEnabled != 0
                ? $"ON 样本 {comparison.EnabledSampleCount}/{required}"
                : "OFF 基线已就绪，开启缓存后继续采样";
        return "OFF/ON 对比有效";
    }

    private static string ComparisonTimeDelta(
        SimulationDebuggerCacheComparison comparison)
    {
        if (comparison.ComparisonAvailable == 0)
            return ComparisonStatus(comparison);
        return $"{comparison.DeltaMilliseconds:+0.000;-0.000;0.000} ms " +
               $"({comparison.DeltaPercent:+0.0%;-0.0%;0.0%})";
    }

    private static string ComparisonPairDelta(
        SimulationDebuggerCacheComparison comparison)
    {
        if (comparison.ComparisonAvailable == 0)
            return ComparisonStatus(comparison);
        return $"{comparison.PairDelta:+0;-0;0} " +
               $"({comparison.PairDeltaPercent:+0.0%;-0.0%;0.0%})";
    }

    private void DrawComparisonDetails(
        SimulationDebuggerCacheComparison comparison)
    {
        GUILayout.Space(6f);
        GUILayout.Label("同配置 OFF / ON 对比", _sectionStyle);
        DrawDetailRow(
            "有效样本",
            $"{comparison.BaselineSampleCount:N0} / {comparison.EnabledSampleCount:N0}");
        DrawDetailRow(
            "管线 P50",
            comparison.ComparisonAvailable != 0
                ? $"{comparison.BaselineMedianMilliseconds:0.000} / " +
                  $"{comparison.EnabledMedianMilliseconds:0.000} ms"
                : ComparisonStatus(comparison));
        DrawDetailRow(
            "管线 P95",
            comparison.ComparisonAvailable != 0
                ? $"{comparison.BaselineP95Milliseconds:0.000} / " +
                  $"{comparison.EnabledP95Milliseconds:0.000} ms"
                : "--");
        DrawDetailRow(
            "候选评估 P50",
            comparison.ComparisonAvailable != 0
                ? $"{comparison.BaselineMedianPairCount:0} / " +
                  $"{comparison.EnabledMedianPairCount:0}"
                : "--");
    }

    private void DrawComparisonSettingsRow(
        string label,
        SimulationDebuggerCacheComparison comparison)
    {
        DrawDetailRow(
            label,
            comparison.ComparisonAvailable != 0
                ? ComparisonTimeDelta(comparison)
                : ComparisonStatus(comparison));
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

    private void DrawPanelGridHeatmap(SimulationDebuggerFrameSnapshot snapshot)
    {
        if (_activeWindowState == null ||
            _activeWindowState.Heatmap == SimulationDebuggerHeatmap.None)
            return;

        GUILayout.BeginHorizontal();
        _activeWindowState.ShowGridHeatmap = GUILayout.Toggle(
            _activeWindowState.ShowGridHeatmap,
            "显示网格热力图",
            GUILayout.Width(126f));
        GUILayout.FlexibleSpace();
        if (GUILayout.Button("映射到游戏地图", GUILayout.Width(110f), GUILayout.Height(23f)))
        {
            SimulationDebuggerRuntime.WorldHeatmap = _activeWindowState.Heatmap;
            SimulationDebuggerRuntime.WorldOverlayView = _currentView;
            SimulationDebuggerRuntime.OverlayEnabled = true;
        }
        GUILayout.EndHorizontal();

        if (!_activeWindowState.ShowGridHeatmap)
            return;

        if (snapshot.Cells.Count == 0)
        {
            GUILayout.Label("当前快照没有空间网格数据。", _mutedStyle);
            return;
        }

        Rect available = GUILayoutUtility.GetRect(
            100f,
            Mathf.Clamp(_activeWindowState.Rect.height * 0.34f, 130f, 300f),
            GUILayout.ExpandWidth(true));
        if (Event.current.type != EventType.Repaint &&
            Event.current.type != EventType.MouseMove)
            return;

        int minX = int.MaxValue;
        int minY = int.MaxValue;
        int maxX = int.MinValue;
        int maxY = int.MinValue;
        for (int i = 0; i < snapshot.Cells.Count; i++)
        {
            var coordinate = snapshot.Cells[i].Coordinate;
            minX = Mathf.Min(minX, coordinate.x);
            minY = Mathf.Min(minY, coordinate.y);
            maxX = Mathf.Max(maxX, coordinate.x);
            maxY = Mathf.Max(maxY, coordinate.y);
        }

        int width = Mathf.Max(1, maxX - minX + 1);
        int height = Mathf.Max(1, maxY - minY + 1);
        float scale = Mathf.Min(available.width / width, available.height / height);
        float mapWidth = scale * width;
        float mapHeight = scale * height;
        Rect mapRect = new Rect(
            available.x + (available.width - mapWidth) * 0.5f,
            available.y + (available.height - mapHeight) * 0.5f,
            mapWidth,
            mapHeight);
        EditorSafeDrawRect(mapRect, new Color(0.035f, 0.045f, 0.06f, 0.96f));

        SimulationDebuggerCellSample? hovered = null;
        for (int i = 0; i < snapshot.Cells.Count; i++)
        {
            SimulationDebuggerCellSample cell = snapshot.Cells[i];
            int localX = cell.Coordinate.x - minX;
            int localY = maxY - cell.Coordinate.y;
            Rect cellRect = new Rect(
                mapRect.x + localX * scale,
                mapRect.y + localY * scale,
                scale,
                scale);
            float value = GetPanelHeatmapValue(cell, _activeWindowState.Heatmap);
            Color fill = HeatmapPanelColor(_activeWindowState.Heatmap, value);
            float inset = scale >= 5f ? 1f : 0.25f;
            Rect fillRect = new Rect(
                cellRect.x + inset,
                cellRect.y + inset,
                Mathf.Max(0f, cellRect.width - inset * 2f),
                Mathf.Max(0f, cellRect.height - inset * 2f));
            EditorSafeDrawRect(fillRect, fill);

            if (snapshot.HasSelectedUnit &&
                snapshot.SelectedUnit.CurrentPosition.x >= cell.Min.x &&
                snapshot.SelectedUnit.CurrentPosition.x <= cell.Max.x &&
                snapshot.SelectedUnit.CurrentPosition.z >= cell.Min.y &&
                snapshot.SelectedUnit.CurrentPosition.z <= cell.Max.y)
            {
                DrawGuiRectOutline(cellRect, Color.white, scale >= 6f ? 2f : 1f);
            }

            if (cellRect.Contains(Event.current.mousePosition))
                hovered = cell;
        }
        DrawGuiRectOutline(mapRect, new Color(0.45f, 0.52f, 0.62f), 1f);

        if (hovered.HasValue)
        {
            SimulationDebuggerCellSample cell = hovered.Value;
            GUI.Label(
                new Rect(mapRect.x + 6f, mapRect.y + 5f, mapRect.width - 12f, 22f),
                $"格子 ({cell.Coordinate.x}, {cell.Coordinate.y})  单位 {cell.UnitCount}  数值 {GetPanelHeatmapValue(cell, _activeWindowState.Heatmap):0.000}",
                _mutedStyle);
        }
    }

    private static float GetPanelHeatmapValue(
        SimulationDebuggerCellSample cell,
        SimulationDebuggerHeatmap mode)
    {
        return Mathf.Clamp01(mode switch
        {
            SimulationDebuggerHeatmap.OverallPressure => cell.OverallPressure,
            SimulationDebuggerHeatmap.UnitDensity => cell.Density,
            SimulationDebuggerHeatmap.SolverCorrection => cell.SolverCorrection,
            SimulationDebuggerHeatmap.AabbBenefit => cell.AabbBenefit,
            SimulationDebuggerHeatmap.AabbSlack => cell.AabbSlack,
            SimulationDebuggerHeatmap.CandidateExpansion => cell.CandidateExpansion,
            SimulationDebuggerHeatmap.EscapeRisk => cell.EscapeRisk,
            SimulationDebuggerHeatmap.ContactActivation => cell.ContactActivation,
            SimulationDebuggerHeatmap.ContactWaste => cell.ContactWaste,
            SimulationDebuggerHeatmap.ContactSupplementRisk => cell.ContactSupplementRisk,
            _ => 0f
        });
    }

    private static Color HeatmapPanelColor(
        SimulationDebuggerHeatmap mode,
        float value)
    {
        bool positive = mode == SimulationDebuggerHeatmap.AabbBenefit ||
                        mode == SimulationDebuggerHeatmap.AabbSlack ||
                        mode == SimulationDebuggerHeatmap.ContactActivation;
        Color low = positive
            ? new Color(0.55f, 0.10f, 0.08f, 0.90f)
            : new Color(0.08f, 0.18f, 0.42f, 0.82f);
        Color middle = new Color(0.92f, 0.68f, 0.10f, 0.92f);
        Color high = positive
            ? new Color(0.08f, 0.68f, 0.30f, 0.96f)
            : new Color(0.88f, 0.10f, 0.05f, 0.96f);
        return value < 0.5f
            ? Color.Lerp(low, middle, value * 2f)
            : Color.Lerp(middle, high, (value - 0.5f) * 2f);
    }

    private static void DrawGuiRectOutline(Rect rect, Color color, float thickness)
    {
        EditorSafeDrawRect(new Rect(rect.x, rect.y, rect.width, thickness), color);
        EditorSafeDrawRect(new Rect(rect.x, rect.yMax - thickness, rect.width, thickness), color);
        EditorSafeDrawRect(new Rect(rect.x, rect.y, thickness, rect.height), color);
        EditorSafeDrawRect(new Rect(rect.xMax - thickness, rect.y, thickness, rect.height), color);
    }

    private void DrawHeatmapLegend()
    {
        SimulationDebuggerHeatmap mode = _activeWindowState?.Heatmap ?? SimulationDebuggerHeatmap.None;
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
                "鼠标中键短按单位后，这里会显示它的运动、AABB 和跨子步接触；中键拖动仍可留给相机控制。",
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

        switch (_currentView)
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
            string state = pair.State == SimulationDebuggerPairState.Active ? "已激活" : "已缓存";
            string kind = PairKindLabel(pair.Kind);
            GUILayout.Label(
                $"{i + 1,2}. {kind,-10} {state,-6}  sep {pair.CurrentSeparation,7:0.000}  " +
                $"λ {pair.Lambda,7:0.000}  子步 {pair.FirstActivatedSubstep}",
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
            SimulationDebuggerPairKind.PredictiveContact => "预测接触",
            SimulationDebuggerPairKind.NearContact => "临近接触",
            SimulationDebuggerPairKind.SupplementedContact => "后补接触",
            SimulationDebuggerPairKind.BroadCandidate => "候选对",
            _ => "当前接触"
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
            bool active = _activeWindowState != null && _activeWindowState.Heatmap == mode;
            if (GUILayout.Button(
                    HeatmapLabel(mode),
                    active ? _activeTabStyle : _tabStyle,
                    GUILayout.Height(25f)))
            {
                _activeWindowState.Heatmap = active
                    ? SimulationDebuggerHeatmap.None
                    : mode;
                RefreshCaptureMask();
            }
        }
        GUILayout.EndHorizontal();
    }

    private void DrawTimeBreakdown(SimulationOverviewMetrics metrics)
    {
        if (metrics.TimingAvailable == 0)
        {
            DrawDetailRow("阶段计时", "--");
            return;
        }

        DrawDetailRow("接触管线总计", Nanoseconds(metrics.SolverNanoseconds));
        DrawDetailRow("Broad Phase", Nanoseconds(metrics.BroadPhaseNanoseconds));
        DrawDetailRow("Narrow Phase", Nanoseconds(metrics.NarrowPhaseNanoseconds));
        DrawDetailRow(
            "Substep 激活调度",
            Nanoseconds(metrics.ContactActivationNanoseconds));
        DrawDetailRow("软避让", Nanoseconds(metrics.SoftAvoidanceNanoseconds));
        DrawDetailRow("约束求解阶段总计", Nanoseconds(metrics.IterationNanoseconds));
        DrawDetailRow("约束阶段摊销 / 轮", Nanoseconds(metrics.AverageIterationNanoseconds));
        if (metrics.SolverSkipReason != ContactSolverSkipReason.None)
        {
            DrawDetailRow(
                "求解跳过",
                $"{SolverSkipReasonLabel(metrics.SolverSkipReason)} · " +
                $"{metrics.SolverSkippedSubstepCount} 次");
        }
        DrawDetailRow("其他阶段", Nanoseconds(metrics.OtherStageNanoseconds));
        if (metrics.OverlappingStageNanoseconds > 0)
        {
            DrawDetailRow(
                "计时重叠",
                $"{Nanoseconds(metrics.OverlappingStageNanoseconds)}（阶段不可直接相加）");
        }
        DrawDetailRow(
            "旧版生成+分类计时",
            Nanoseconds(metrics.PairGenerationNanoseconds));
    }

    private void DrawContactWorkload(SimulationOverviewMetrics metrics)
    {
        if (metrics.WorkloadAvailable == 0)
        {
            DrawDetailRow("接触工作量", "--");
            return;
        }

        DrawDetailRow("当前接触关系", metrics.CurrentContactCount.ToString("N0"));
        DrawDetailRow("当前实际接触", metrics.CurrentActualPairCount.ToString("N0"));
        DrawDetailRow("当前预测接触", metrics.CurrentPredictivePairCount.ToString("N0"));
        DrawDetailRow("当前接近关系", metrics.CurrentApproachingPairCount.ToString("N0"));
        DrawDetailRow("当前休眠邻居", metrics.CurrentDormantPairCount.ToString("N0"));
        DrawDetailRow("候选 Pair 评估累计", metrics.CandidatePairCount.ToString("N0"));
        DrawDetailRow("保留接触评估累计", metrics.ContactPairCount.ToString("N0"));
    }

    private static string AvailableNanoseconds(byte available, long nanoseconds) =>
        available != 0 ? Nanoseconds(nanoseconds) : "--";

    private static string AvailableWorldUnits(byte available, float value) =>
        available != 0 ? $"{value:0.000} 世界单位" : "--";

    private static string AvailableWorldUnitsPerSecond(byte available, float value) =>
        available != 0 ? $"{value:0.000} 世界单位/秒" : "--";

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

    private void DrawLocalRecordingStatus()
    {
        SimulationDebuggerLocalRecorder recorder = SimulationDebuggerLocalRecorder.Instance;
        if (recorder == null || !recorder.IsRecording)
        {
            GUILayout.Label("F6：手动开始/停止本地记录　F7：自动记录 10 秒", _mutedStyle);
            return;
        }

        string mode = recorder.IsAutomaticRun ? "F7 自动 10 秒" : "F6 手动";
        GUILayout.Label(
            $"本地记录中：{mode} · {recorder.ElapsedSeconds:0.0}s · {recorder.SampleCount} 条 · {recorder.OutputDirectory}",
            _mutedStyle);
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

        SimulationDebuggerCaptureMask mask = SimulationDebuggerCaptureMask.None;
        if (OverviewWindow != null && OverviewWindow.Visible)
        {
            mask |= SimulationDebuggerCaptureMask.Summary |
                    SimulationDebuggerCaptureMask.OverviewHeatmap |
                    SimulationDebuggerCaptureMask.SelectedUnit |
                    SimulationDebuggerCaptureMask.Proxies;
            if (OverviewWindow.ShowDetails)
                mask |= SimulationDebuggerCaptureMask.DetailedCounters;
        }
        if (AabbWindow != null && AabbWindow.Visible)
        {
            mask |= SimulationDebuggerCaptureMask.Summary |
                    SimulationDebuggerCaptureMask.AabbHeatmap |
                    SimulationDebuggerCaptureMask.Regions |
                    SimulationDebuggerCaptureMask.Proxies |
                    SimulationDebuggerCaptureMask.SelectedUnit;
            if (AabbWindow.ShowDetails)
                mask |= SimulationDebuggerCaptureMask.DetailedCounters;
        }
        if (ContactWindow != null && ContactWindow.Visible)
        {
            mask |= SimulationDebuggerCaptureMask.Summary |
                    SimulationDebuggerCaptureMask.ContactSetHeatmap |
                    SimulationDebuggerCaptureMask.SelectedUnit |
                    SimulationDebuggerCaptureMask.SelectedPairs |
                    SimulationDebuggerCaptureMask.Proxies;
            if (ContactWindow.ShowDetails)
                mask |= SimulationDebuggerCaptureMask.DetailedCounters;
        }
        if (SettingsWindow != null && SettingsWindow.Visible)
            mask |= SimulationDebuggerCaptureMask.Summary;

        SimulationDebuggerRuntime.CaptureMask = mask;
    }

    private static string OverviewStatus(SimulationOverviewMetrics metrics)
    {
        if (metrics.UnitCount <= 0)
            return "等待参与接触仿真的单位。";
        if (metrics.TimingAvailable == 0 || metrics.StabilityAvailable == 0)
            return "单位仿真正在运行；等待接触管线摘要遥测。";
        if (metrics.SolverSkipReason != ContactSolverSkipReason.None)
        {
            return $"约束求解被跳过：{SolverSkipReasonLabel(metrics.SolverSkipReason)}。";
        }

        if (metrics.Health == SimulationDebuggerHealth.Critical)
        {
            if (metrics.SolverMilliseconds > 4f &&
                metrics.MaxContactCorrection > 0.25f)
                return "接触管线耗时和最大纠偏均明显偏高。";
            return metrics.SolverMilliseconds > 4f
                ? "接触管线耗时明显偏高；展开阶段耗时定位瓶颈。"
                : "最大接触纠偏明显偏高；检查穿透和求解稳定性。";
        }
        if (metrics.Health == SimulationDebuggerHealth.Warning)
        {
            if (metrics.SolverMilliseconds > 2f &&
                metrics.MaxContactCorrection > 0.08f)
                return "接触管线耗时和最大纠偏正在升高。";
            return metrics.SolverMilliseconds > 2f
                ? "接触管线耗时正在升高。"
                : "最大接触纠偏正在升高。";
        }
        return "接触管线成本和最大纠偏处于正常范围。";
    }

    private static string SolverSkipReasonLabel(ContactSolverSkipReason reason)
    {
        return reason switch
        {
            ContactSolverSkipReason.CertificateUnavailable => "证书容器不可用",
            ContactSolverSkipReason.CertificateNotIssued => "证书未签发",
            ContactSolverSkipReason.CertificateStructureNotVerified => "视图结构未验证",
            ContactSolverSkipReason.CertificateInteractionCountInvalid => "交互 Pair 数非法",
            ContactSolverSkipReason.CertificateViewUnavailable => "消费者视图不可用",
            ContactSolverSkipReason.CertificateInteractionPairInvalid => "交互 Pair 非法",
            ContactSolverSkipReason.CertificateSoftPairInvalid => "软避让 Pair 非法",
            ContactSolverSkipReason.CertificateContactPairInvalid => "接触 Pair 非法",
            ContactSolverSkipReason.CertificateScheduleInvalid => "预测调度越界",
            ContactSolverSkipReason.CertificateEntityMappingNotVerified => "实体映射未验证",
            ContactSolverSkipReason.CertificateConfigurationNotVerified => "配置未验证",
            ContactSolverSkipReason.CertificateTopologyNotVerified => "拓扑覆盖未验证",
            ContactSolverSkipReason.CertificateClassificationNotVerified => "分类未验证",
            ContactSolverSkipReason.CertificateConsumerViewsNotCommitted => "消费者视图未提交",
            ContactSolverSkipReason.CertificateScopeMismatch => "证书作用域不匹配",
            ContactSolverSkipReason.BodySetMismatch => "单位集合不匹配",
            ContactSolverSkipReason.ConfigurationMismatch => "配置指纹不匹配",
            ContactSolverSkipReason.SoftPairCountMismatch => "软避让 Pair 数不匹配",
            ContactSolverSkipReason.ContactConstraintCountMismatch => "接触 Pair 数不匹配",
            ContactSolverSkipReason.DormantScheduleCountMismatch => "休眠调度数不匹配",
            _ => reason.ToString()
        };
    }

    private static string HeatmapLabel(SimulationDebuggerHeatmap mode)
    {
        return mode switch
        {
            SimulationDebuggerHeatmap.OverallPressure => "综合压力",
            SimulationDebuggerHeatmap.UnitDensity => "密度",
            SimulationDebuggerHeatmap.SolverCorrection => "修正量",
            SimulationDebuggerHeatmap.AabbBenefit => "拓扑稳定度",
            SimulationDebuggerHeatmap.AabbSlack => "低运动风险",
            SimulationDebuggerHeatmap.CandidateExpansion => "接触负载",
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
            normal = { background = _panelTexture },
            focused = { background = _panelTexture },
            active = { background = _panelTexture },
            onNormal = { background = _panelTexture },
            onFocused = { background = _panelTexture },
            onActive = { background = _panelTexture }
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
        // 先清理上一次 Play Mode 遗留的旧面板。
        var existing = UnityEngine.Object.FindObjectsByType<SimulationDebuggerPanel>(
            UnityEngine.FindObjectsSortMode.None);
        foreach (var old in existing)
        {
            if (old != null && old.gameObject != null)
                UnityEngine.Object.Destroy(old.gameObject);
        }

        var gameObject = new GameObject("Simulation Debugger")
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        gameObject.AddComponent<SimulationDebuggerPanel>();
        UnityEngine.Object.DontDestroyOnLoad(gameObject);

        if (gameObject.GetComponent<SimulationDebuggerWorldOverlay>() == null)
            gameObject.AddComponent<SimulationDebuggerWorldOverlay>();
        if (gameObject.GetComponent<SimulationDebuggerUnitPicker>() == null)
        {
            var picker = gameObject.AddComponent<SimulationDebuggerUnitPicker>();
            picker.ClearSelectionWhenNothingHit = true;
        }
        if (gameObject.GetComponent<SimulationDebuggerCameraFollow>() == null)
        {
            gameObject.AddComponent<SimulationDebuggerCameraFollow>();
        }
        if (gameObject.GetComponent<SimulationDebuggerLocalRecorder>() == null)
            gameObject.AddComponent<SimulationDebuggerLocalRecorder>();
    }
#endif
}
}
