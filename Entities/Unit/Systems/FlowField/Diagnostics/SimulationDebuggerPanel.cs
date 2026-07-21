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

    private const int WindowId = 0x51A7;
    private Vector2 _scroll;
    private bool _showDetails;
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
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.Summary;
    }

    private void Update()
    {
        if (Input.GetKeyDown(ToggleKey))
        {
            Visible = !Visible;
            RefreshCaptureMask();
        }
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
            GUILayout.MinWidth(440f),
            GUILayout.MinHeight(360f));
        WindowRect.x = Mathf.Clamp(WindowRect.x, 0f, Mathf.Max(0f, Screen.width - 80f));
        WindowRect.y = Mathf.Clamp(WindowRect.y, 0f, Mathf.Max(0f, Screen.height - 40f));
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
        DrawStatus("运行时设置", SimulationDebuggerHealth.Healthy, "统一设置入口；修改在 timestep 边界生效");
        GUILayout.Space(10f);
        GUILayout.Label("当前有效结构", _sectionStyle);
        DrawDetailRow("Substeps", snapshot.SubstepCount.ToString());
        DrawDetailRow("Iterations", snapshot.IterationCount.ToString());
        DrawDetailRow("AABB", snapshot.BroadPhase.Enabled != 0 ? "启用" : "关闭");
        DrawDetailRow("Diagnostics Capture", snapshot.CapturedMask.ToString());
        GUILayout.Space(8f);
        GUILayout.Label(
            "参数编辑、Override/Effective 对照与恢复默认将在设置绑定阶段接入。",
            _mutedStyle);
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
            SimulationDebuggerRuntime.CaptureMask = SimulationDebuggerCaptureMask.Summary;
            return;
        }

        SimulationDebuggerCaptureMask mask = SimulationDebuggerCaptureMask.Summary;
        switch (SimulationDebuggerRuntime.ActiveView)
        {
            case SimulationDebuggerView.Overview:
                mask |= SimulationDebuggerCaptureMask.OverviewHeatmap;
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
        if (_headerStyle != null)
            return;

        _panelTexture = SolidTexture(new Color(0.065f, 0.075f, 0.095f, 0.97f));
        _cardTexture = SolidTexture(new Color(0.105f, 0.12f, 0.15f, 0.96f));
        _activeTexture = SolidTexture(new Color(0.18f, 0.34f, 0.55f, 0.98f));

        GUI.skin.window.normal.background = _panelTexture;
        GUI.skin.window.padding = new RectOffset(12, 12, 10, 12);
        GUI.skin.label.normal.textColor = new Color(0.9f, 0.93f, 0.97f);
        GUI.skin.button.normal.textColor = new Color(0.9f, 0.93f, 0.97f);
        GUI.skin.button.hover.textColor = Color.white;

        _headerStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 15,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft
        };
        _sectionStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold,
            wordWrap = true,
            padding = new RectOffset(8, 8, 7, 7),
            normal = { background = _cardTexture }
        };
        _metricLabelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            normal = { textColor = new Color(0.63f, 0.7f, 0.8f) }
        };
        _metricValueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 20,
            fontStyle = FontStyle.Bold
        };
        _mutedStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 10,
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
            fontSize = 11,
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
        if (UnityEngine.Object.FindFirstObjectByType<SimulationDebuggerPanel>() != null)
            return;
        var gameObject = new GameObject("Simulation Debugger");
        gameObject.hideFlags = HideFlags.DontSave;
        gameObject.AddComponent<SimulationDebuggerPanel>();
        UnityEngine.Object.DontDestroyOnLoad(gameObject);
    }
#endif
}
}
