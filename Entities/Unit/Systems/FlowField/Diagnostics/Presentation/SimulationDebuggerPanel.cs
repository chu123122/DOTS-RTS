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
    private Texture2D _chartTexture;
    private float[] _chartBuffer = new float[120];

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
        DestroyRuntimeTexture(ref _chartTexture);
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
                    DrawContactSet(snapshot);
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
            SimulationDebuggerView.PersistentBroadPhase => "跨帧接触缓存",
            SimulationDebuggerView.TimestepContactSet => "跨子步接触缓存",
            _ => "运行时设置"
        };
    }

    private void DrawFrameStrip(SimulationDebuggerFrameSnapshot snapshot)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label($"帧 {snapshot.FrameId}", _mutedStyle);
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
        DrawTrendChart(SimulationDebuggerRuntime.GetSolverHistory(), "求解 ms");
        GUILayout.Space(4f);

        SimulationOverviewMetrics metrics = snapshot.Overview;
        DrawStatus("整体仿真", metrics.Health, OverviewStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric("求解耗时", $"{metrics.SolverMilliseconds:0.000} ms", "整套移动与碰撞每帧成本");
        DrawMetric("Pair / Contact", Nanoseconds(metrics.PairGenerationNanoseconds), "Fat AABB 应直接降低的阶段");
        DrawMetric("XPBD Iteration", Nanoseconds(metrics.IterationNanoseconds), "约束投影成本，不应因缓存直接下降");
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
        DrawPanelGridHeatmap(snapshot);
        DrawSelectedUnitSection(snapshot);

        DrawDetailsToggle();
        if (!_showDetails)
            return;

        GUILayout.Label("阶段详情", _sectionStyle);
        DrawTimeBreakdown(metrics);
        DrawDetailRow("单位数量", metrics.UnitCount.ToString("N0"));
        DrawDetailRow("Broad 候选", metrics.CandidatePairCount.ToString("N0"));
        DrawDetailRow("接触缓存", metrics.ContactPairCount.ToString("N0"));
        DrawDetailRow("最大接触修正", metrics.MaxContactCorrection.ToString("0.000"));
        DrawDetailRow("最大墙体修正", metrics.MaxWallCorrection.ToString("0.000"));
        DrawDetailRow("最大速度变化", metrics.MaxVelocityChange.ToString("0.000"));

        GUILayout.Space(6f);
        GUILayout.Label("60 帧趋势", _sectionStyle);
        DrawTrendRow("求解耗时", SimulationDebuggerRuntime.GetSolverTrend(), "0.0", "ms");
        DrawTrendRow("最大修正量", SimulationDebuggerRuntime.GetCorrectionTrend(), "0.000");
        DrawTrendRow("接触对数量", SimulationDebuggerRuntime.GetContactPairTrend(), "0");
    }

    private void DrawPersistentBroadPhase(
        SimulationDebuggerFrameSnapshot snapshot,
        IncrementalContactPipelineSnapshot pipeline)
    {
        var statistics = pipeline.Statistics;
        bool cacheEnabled = snapshot.EffectiveSettings.EnablePersistentContactCache != 0;
        bool hasPipelineSnapshot = statistics.Timestep != 0;
        SimulationDebuggerHealth health;
        string status;
        if (!cacheEnabled)
        {
            health = SimulationDebuggerHealth.Disabled;
            status = "跨帧邻居拓扑已关闭；当前每个子步重新生成接触候选。";
        }
        else if (!hasPipelineSnapshot)
        {
            health = SimulationDebuggerHealth.Warning;
            status = "等待增量接触管线发布首个时间步快照。";
        }
        else if (statistics.OracleMissingPairCount != 0 || statistics.OracleMismatch != 0)
        {
            health = SimulationDebuggerHealth.Critical;
            status = "Oracle 发现增量接触视图存在缺失 Pair；已记录不一致，但诊断系统不会自动改变 Gameplay cache 状态。";
        }
        else if (statistics.FullRebuildCount != 0)
        {
            health = SimulationDebuggerHealth.Warning;
            status = "本时间步发生完整重建；检查脏体比例和局部查询范围。";
        }
        else
        {
            health = SimulationDebuggerHealth.Healthy;
            status = "持久邻居拓扑有效，当前数据来自增量接触管线。";
        }

        DrawStatus("跨帧接触缓存", health, status);
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric("拓扑脏体", hasPipelineSnapshot
            ? $"{statistics.TopologyDirtyBodyCount} / {statistics.ProxyCount}"
            : "--", "脏体越少，跨帧邻居拓扑复用越高");
        DrawMetric("持久邻居对", hasPipelineSnapshot
            ? statistics.PersistentNeighborPairCount.ToString("N0") : "--", "跨帧保留的局部候选对");
        DrawMetric("更新模式", hasPipelineSnapshot ? pipeline.Mode.ToString() : "--",
            "Reuse 最优；Repair 为局部更新；FullRebuild 为完整重建");
        GUILayout.EndHorizontal();

        DrawHeatmapSelector(
            "接触拓扑热力图",
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

        GUILayout.Label("增量拓扑详情", _sectionStyle);
        DrawDetailRow("时间步 / 模式", hasPipelineSnapshot
            ? $"{statistics.Timestep} / {pipeline.Mode}" : "尚无快照");
        DrawDetailRow("运动脏体 / 逃逸", hasPipelineSnapshot
            ? $"{statistics.MotionDirtyBodyCount} / {statistics.CorrectedEscapeBodyCount}" : "--");
        DrawDetailRow("新增 / 移除 / 保留 Pair", hasPipelineSnapshot
            ? $"{statistics.NeighborPairAddedCount} / {statistics.NeighborPairRemovedCount} / {statistics.NeighborPairRetainedCount}" : "--");
        DrawDetailRow("完整重建 / 局部修复", hasPipelineSnapshot
            ? $"{statistics.FullRebuildCount} / {statistics.IncrementalRepairCount}" : "--");
        DrawDetailRow("干净代理 / Pair 保留率", hasPipelineSnapshot
            ? $"{pipeline.CleanProxyRatio:P1} / {pipeline.RetainedNeighborPairRatio:P1}" : "--");
        DrawDetailRow("局部代理查询", hasPipelineSnapshot
            ? statistics.LocalProxyQueryCount.ToString("N0") : "--");
        DrawDetailRow("代理校验 / 局部 Broad / Pair Diff", hasPipelineSnapshot
            ? $"{Nanoseconds(statistics.ProxyValidationNanoseconds)} / {Nanoseconds(statistics.LocalBroadPhaseNanoseconds)} / {Nanoseconds(statistics.PairDiffNanoseconds)}" : "--");
        DrawDetailRow("Oracle 缺失 / 额外", hasPipelineSnapshot
            ? $"{statistics.OracleMissingPairCount} / {statistics.OracleExtraPairCount}" : "--");
    }

    private void DrawContactSet(SimulationDebuggerFrameSnapshot snapshot)
    {
        DrawTrendChart(SimulationDebuggerRuntime.GetContactPairHistory(), "接触对数");
        GUILayout.Space(4f);

        TimestepContactSetMetrics metrics = snapshot.ContactSet;
        DrawStatus("跨子步接触缓存", metrics.Health, ContactSetStatus(metrics));
        GUILayout.Space(8f);

        GUILayout.BeginHorizontal();
        DrawMetric(
            metrics.CacheEnabled != 0 ? "整步接触集" : "子步接触集",
            metrics.ContactSetSize.ToString("N0"),
            metrics.CacheEnabled != 0 ? "一个时间步只生成一次并跨子步复用" : "每个子步重新生成，仅在迭代内复用");
        DrawMetric("接触激活率", Percent(metrics.ActivationRatio), "至少一次真正产生约束作用的接触");
        DrawMetric(
            "重建 / 补充 Pair",
            $"{metrics.FullRebuildCount} / {metrics.FallbackAddedPairCount}",
            "完整重建表示视图重新生成；补充 Pair 表示初始接触集遗漏");
        GUILayout.EndHorizontal();

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

        GUILayout.Label("接触集组成", _sectionStyle);
        DrawDetailRow("生成模式", metrics.CacheEnabled != 0 ? "每时间步一次" : "每子步一次");
        DrawDetailRow("本帧生成次数", metrics.ContactGenerationCount.ToString("N0"));
        DrawDetailRow("完整重建次数", metrics.FullRebuildCount.ToString("N0"));
        DrawDetailRow("Fallback 补充 Pair", metrics.FallbackAddedPairCount.ToString("N0"));
        DrawDetailRow("当前 / 临近接触", metrics.ActualContactCount.ToString("N0"));
        DrawDetailRow("预测接触", metrics.PredictiveContactCount.ToString("N0"));
        DrawDetailRow("预测接触已激活", metrics.PredictiveActivatedCount.ToString("N0"));
        DrawDetailRow("缓存但未激活", metrics.InactiveContactCount.ToString("N0"));
        DrawDetailRow("避免重复生成", $"{metrics.AvoidedContactGenerationCount} 次");
        DrawDetailRow("预测接触激活率", Percent(metrics.PredictiveActivationRatio));

        GUILayout.Space(6f);
        GUILayout.Label("60 帧趋势", _sectionStyle);
        DrawTrendRow("活跃接触数", SimulationDebuggerRuntime.GetActiveContactTrend(), "0");
        DrawTrendRow("接触集大小", SimulationDebuggerRuntime.GetContactPairTrend(), "0");
    }

    private void DrawSettingsSummary(SimulationDebuggerFrameSnapshot snapshot)
    {
        DrawStatus("运行时设置", SimulationDebuggerHealth.Healthy, "修改即时生效");
        GUILayout.Space(8f);

        SimulationDebuggerEffectiveSettings draft = snapshot.EffectiveSettings;

        GUILayout.Label("对比实验（A / B / C）", _sectionStyle);
        GUILayout.Label(
            "A 为跨帧持久邻居拓扑，B 为跨子步接触集；A 依赖 B，关闭 B 会自动关闭 A。",
            _mutedStyle);
        draft.EnablePersistentContactCache = DrawToggle(
            "A：跨帧接触缓存",
            draft.EnablePersistentContactCache,
            snapshot.EffectiveSettings.EnablePersistentContactCache);
        draft.EnableTimestepContactSetCache = DrawToggle(
            "B：跨子步接触缓存",
            draft.EnableTimestepContactSetCache,
            snapshot.EffectiveSettings.EnableTimestepContactSetCache);
        if (draft.EnablePersistentContactCache != 0 && draft.EnableTimestepContactSetCache == 0)
        {
            bool userTurnedOffSubstepCache =
                snapshot.EffectiveSettings.EnableTimestepContactSetCache != 0;
            if (userTurnedOffSubstepCache)
            {
                draft.EnablePersistentContactCache = 0;
                GUILayout.Label("已关闭 A：跨帧接触缓存依赖跨子步接触集。", _mutedStyle);
            }
            else
            {
                draft.EnableTimestepContactSetCache = 1;
                GUILayout.Label("已自动开启 B：跨帧接触缓存需要跨子步接触集。", _mutedStyle);
            }
        }
        draft.EnableAdaptiveFatAabb = DrawToggle(
            "热点网格诊断（非执行路径）",
            draft.EnableAdaptiveFatAabb,
            snapshot.EffectiveSettings.EnableAdaptiveFatAabb);
        GUILayout.BeginHorizontal();
        GUILayout.Label("C：软避让求解器", _mutedStyle, GUILayout.Width(170f));
        string[] solverModes = { "预测引导", "RVO 互惠避让" };
        draft.SoftAvoidanceVelocitySolver = GUILayout.SelectionGrid(
            Mathf.Clamp(draft.SoftAvoidanceVelocitySolver, 0, 1),
            solverModes,
            2,
            _tabStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(
            $"当前有效：{SoftSolverLabel(snapshot.EffectiveSettings.SoftAvoidanceVelocitySolver)}",
            _mutedStyle);
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        GUILayout.Label("D：接触位置求解器", _mutedStyle, GUILayout.Width(170f));
        string[] contactSolverModes = { "Gauss-Seidel", "Jacobi" };
        draft.ContactPositionSolver = GUILayout.SelectionGrid(
            Mathf.Clamp(draft.ContactPositionSolver, 0, 1),
            contactSolverModes,
            2,
            _tabStyle);
        GUILayout.FlexibleSpace();
        GUILayout.Label(
            $"当前有效：{ContactSolverLabel(snapshot.EffectiveSettings.ContactPositionSolver)}",
            _mutedStyle);
        GUILayout.EndHorizontal();
        DrawDetailRow(
            "当前实验编号",
            $"{snapshot.Experiment.ShortId} / 配置 #{snapshot.Experiment.ConfigurationId}");
        DrawDetailRow(
            "统计阶段",
            snapshot.Experiment.IsWarmup != 0
                ? $"预热中（{snapshot.Experiment.FramesSinceChanged + 1} 帧）"
                : "可纳入正式对比");

        GUILayout.Space(6f);
        GUILayout.Label("全局与 XPBD", _sectionStyle);
        draft.SubstepCount = DrawIntSlider(
            "子步数量",
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
        draft.Compliance = DrawFloatSlider(
            "柔顺度",
            draft.Compliance,
            snapshot.EffectiveSettings.Compliance,
            0f,
            0.1f,
            "0.0000");
        draft.EnableDiagnostics = DrawToggle(
            "求解器详细诊断",
            draft.EnableDiagnostics,
            snapshot.EffectiveSettings.EnableDiagnostics);

        GUILayout.Space(6f);
        GUILayout.Label("软避让参数", _sectionStyle);
        DrawDetailRow("当前求解器", SoftSolverLabel(draft.SoftAvoidanceVelocitySolver));
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
        bool previousEnabled = GUI.enabled;
        GUI.enabled = previousEnabled && draft.SoftAvoidanceVelocitySolver == 1;
        draft.RvoTimeHorizon = DrawFloatSlider(
            "RVO 预测时间",
            draft.RvoTimeHorizon,
            snapshot.EffectiveSettings.RvoTimeHorizon,
            0.05f,
            5f,
            "0.00");
        GUI.enabled = previousEnabled;

        GUILayout.Space(6f);
        GUILayout.Label("跨帧接触缓存参数", _sectionStyle);
        bool persistentCacheEnabled = draft.EnablePersistentContactCache != 0;
        GUI.enabled = persistentCacheEnabled;
        draft.PersistentGuardEnvelopeMargin = DrawFloatSlider(
            "跨帧预测包络余量",
            draft.PersistentGuardEnvelopeMargin,
            snapshot.EffectiveSettings.PersistentGuardEnvelopeMargin,
            0f,
            5f,
            "0.00");
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
        GUILayout.Label("跨子步接触集参数", _sectionStyle);
        DrawDetailRow(
            "生成生命周期",
            draft.EnableTimestepContactSetCache != 0
                ? "每时间步生成一次，跨全部子步复用"
                : "每个子步重新生成");
        draft.EnablePredictivePairGeneration = DrawToggle(
            "生成预测接触对",
            draft.EnablePredictivePairGeneration,
            snapshot.EffectiveSettings.EnablePredictivePairGeneration);
        GUI.enabled = previousEnabled && draft.EnablePredictivePairGeneration != 0;
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
        GUILayout.Label("诊断与显示", _sectionStyle);
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
        GUILayout.Space(6f);
        GUILayout.Label("摄像机跟随与时间减缓", _sectionStyle);
        SimulationDebuggerRuntime.SlowTimeScale = DrawFloatSlider(
            "选中单位时减缓倍率",
            SimulationDebuggerRuntime.SlowTimeScale,
            SimulationDebuggerRuntime.SlowTimeScale,
            0.01f,
            1f,
            "0.00");
        DrawDetailRow(
            "说明",
            "中键点击单位→自动跟随+时间减缓；中键点击空地→退出。跟随模式下仍可边缘滚动和缩放。");

        SimulationDebuggerRuntime.HeatmapOpacity = DrawFloatSlider(
            "场景热力图透明度",
            SimulationDebuggerRuntime.HeatmapOpacity,
            SimulationDebuggerRuntime.HeatmapOpacity,
            0f,
            0.8f,
            "0.00");

        // 自动提交：每帧检查 draft 是否与有效值有差异，有则提交。
        if (!draft.Equals(snapshot.EffectiveSettings))
            SimulationDebuggerRuntime.SubmitSettings(draft);
    }

    private static string SoftSolverLabel(int solverMode)
    {
        return solverMode == 1 ? "RVO 互惠避让" : "预测引导";
    }

    private static string ContactSolverLabel(int solverMode)
    {
        return solverMode == 1 ? "Jacobi" : "Gauss-Seidel";
    }

    private void DrawTrendChart(
        SimulationDebuggerHistory history,
        string title,
        int width = 120,
        int height = 44)
    {
        if (history == null)
            return;

        if (_chartTexture == null || _chartTexture.width != width || _chartTexture.height != height)
        {
            DestroyRuntimeTexture(ref _chartTexture);
            _chartTexture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                hideFlags = HideFlags.HideAndDontSave,
                filterMode = FilterMode.Point
            };
        }

        // 清空
        Color bg = new Color(0.06f, 0.07f, 0.09f, 1f);
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
            pixels[i] = bg;

        // 画网格线
        Color gridColor = new Color(0.12f, 0.14f, 0.18f);
        for (int y = 0; y < height; y += height / 4)
            for (int x = 0; x < width; x++)
                pixels[y * width + x] = gridColor;

        // 拷贝数据
        System.Array.Clear(_chartBuffer, 0, _chartBuffer.Length);
        history.CopyTo(_chartBuffer, Math.Min(width, _chartBuffer.Length));

        // 找范围
        float min = float.MaxValue, max = float.MinValue;
        int startIdx = Math.Max(0, _chartBuffer.Length - width);
        for (int i = startIdx; i < _chartBuffer.Length; i++)
        {
            float v = _chartBuffer[i];
            if (v < min) min = v;
            if (v > max) max = v;
        }
        if (max <= min) max = min + 1f;

        // 画曲线
        Color lineColor = new Color(0.2f, 0.6f, 0.95f);
        int bufStart = _chartBuffer.Length - width;
        for (int x = 0; x < width; x++)
        {
            float v = _chartBuffer[bufStart + x];
            float t = (v - min) / (max - min);
            int plotY = Mathf.Clamp(Mathf.RoundToInt(t * (height - 1)), 0, height - 1);
            pixels[plotY * width + x] = lineColor;
            // 加粗：上下各 1px
            if (plotY > 0) pixels[(plotY - 1) * width + x] = lineColor;
            if (plotY < height - 1) pixels[(plotY + 1) * width + x] = lineColor;
        }

        _chartTexture.SetPixels(pixels);
        _chartTexture.Apply();

        GUILayout.BeginHorizontal();
        GUILayout.Label(title, _mutedStyle, GUILayout.Width(80f));
        GUILayout.Label($"{min:F1}…{max:F1}", _mutedStyle, GUILayout.Width(80f));
        GUILayout.EndHorizontal();
        GUILayout.Box(_chartTexture, GUIStyle.none, GUILayout.Width(width), GUILayout.Height(height));
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
                ? $"{trend.Current.ToString(format)}{unit}  [{trend.Minimum.ToString(format)}…{trend.Average.ToString(format)}…{trend.Maximum.ToString(format)}]{unit}"
                : $"---",
            GUILayout.ExpandWidth(true));

        GUILayout.EndHorizontal();
        GUI.color = savedColor;
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
        if (metrics.CacheEnabled == 0)
            return "对比模式：每个子步重新生成接触集，不进行跨子步持久化。";
        if (metrics.FallbackAddedPairCount > 0)
            return "本时间步出现 fallback 补充 Pair，初始接触集存在遗漏。";
        if (metrics.FullRebuildCount > 0)
            return "本时间步执行了完整重建；正确性已保留，但缓存复用中断。";
        if (metrics.Health == SimulationDebuggerHealth.Warning)
            return "缓存中未激活接触较多，生成范围可能过于保守。";
        return "同一接触集正在跨子步稳定复用。";
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

