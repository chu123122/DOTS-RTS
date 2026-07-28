#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Editor
{
public sealed class IncrementalContactPipelineBenchmarkWindow : EditorWindow
{
    private enum TrialPreset
    {
        FullSweptPerSubstep,
        TimestepSweptNoFrameReuse,
        IncrementalTightGuard,
        IncrementalDefault,
        IncrementalWideGuard,
        Custom
    }

    private TrialPreset _preset = TrialPreset.TimestepSweptNoFrameReuse;
    private string _experimentId = "incremental_contact_v5";
    private string _scenario = "static_dense_pack";
    private string _configurationLabel = "incremental_default";
    private string _outputDirectory = "Diagnostics/IncrementalContact";
    private int _warmupFrames = 300;
    private int _measuredFrames = 600;

    private bool _timestepCacheEnabled = true;
    private bool _crossFrameTopologyEnabled;
    private bool _predictiveContactsEnabled = true;
    private int _substeps = 4;
    private int _iterations = 4;
    private float _guardMargin = 0.5f;
    private float _predictiveSkin = 0.05f;
    private float _contactMargin = 0.02f;

    [MenuItem("RTS/Diagnostics/Incremental Contact Benchmark Tuner")]
    public static void Open()
    {
        GetWindow<IncrementalContactPipelineBenchmarkWindow>("Contact Benchmark");
    }

    private void OnEnable()
    {
        IncrementalContactPipelineCsvRecorderRuntime.SessionCompleted += OnSessionCompleted;
        ApplyPreset(_preset);
    }

    private void OnDisable()
    {
        IncrementalContactPipelineCsvRecorderRuntime.SessionCompleted -= OnSessionCompleted;
    }

    private void OnGUI()
    {
        EditorGUILayout.HelpBox(
            "Compare Timestep Swept (A off, B on) with Incremental (A on, B on). " +
            "Keep the same settled unit state, destinations and random seed. Performance presets disable the O(N²) oracle; run correctness validation separately.",
            MessageType.Info);

        EditorGUI.BeginChangeCheck();
        TrialPreset selectedPreset = (TrialPreset)EditorGUILayout.EnumPopup("Preset", _preset);
        if (EditorGUI.EndChangeCheck())
        {
            _preset = selectedPreset;
            ApplyPreset(_preset);
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Trial identity", EditorStyles.boldLabel);
        _experimentId = EditorGUILayout.TextField("Experiment ID", _experimentId);
        _scenario = EditorGUILayout.TextField("Scenario", _scenario);
        _configurationLabel = EditorGUILayout.TextField("Configuration label", _configurationLabel);
        _outputDirectory = EditorGUILayout.TextField("Output directory", _outputDirectory);
        _warmupFrames = Mathf.Max(0, EditorGUILayout.IntField("Warmup timesteps", _warmupFrames));
        _measuredFrames = Mathf.Max(1, EditorGUILayout.IntField("Measured timesteps", _measuredFrames));

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Effective overrides", EditorStyles.boldLabel);
        using (new EditorGUI.DisabledScope(_preset != TrialPreset.Custom))
        {
            _timestepCacheEnabled = EditorGUILayout.Toggle("Cross-substep contact set (B)", _timestepCacheEnabled);
            _crossFrameTopologyEnabled = EditorGUILayout.Toggle("Cross-frame topology (A)", _crossFrameTopologyEnabled);
            if (_crossFrameTopologyEnabled && !_timestepCacheEnabled)
            {
                _timestepCacheEnabled = true;
                EditorGUILayout.HelpBox("A requires B; B was enabled automatically.", MessageType.Info);
            }
            _predictiveContactsEnabled = EditorGUILayout.Toggle("Predictive contacts", _predictiveContactsEnabled);
            _substeps = Mathf.Max(1, EditorGUILayout.IntField("Substeps", _substeps));
            _iterations = Mathf.Max(1, EditorGUILayout.IntField("Iterations", _iterations));
            _guardMargin = Mathf.Max(0f, EditorGUILayout.FloatField("Guard envelope margin", _guardMargin));
            _predictiveSkin = Mathf.Max(0f, EditorGUILayout.FloatField("Predictive skin", _predictiveSkin));
            _contactMargin = Mathf.Max(0f, EditorGUILayout.FloatField("Contact margin", _contactMargin));
        }

        EditorGUILayout.Space();
        DrawPresetMeaning();

        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("Apply overrides"))
                ApplyRuntimeOverrides();
            if (GUILayout.Button("Disable overrides"))
                IncrementalContactPipelineExperimentRuntime.OverrideEnabled = false;
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Recording", EditorStyles.boldLabel);
        EditorGUILayout.LabelField("State",
            IncrementalContactPipelineCsvRecorderRuntime.IsRecording ? "Recording" : "Idle");
        EditorGUILayout.LabelField("Progress",
            $"{IncrementalContactPipelineCsvRecorderRuntime.RecordedFrames}/" +
            $"{IncrementalContactPipelineCsvRecorderRuntime.TargetFrames}");

        using (new EditorGUI.DisabledScope(
                   IncrementalContactPipelineCsvRecorderRuntime.IsRecording || !EditorApplication.isPlaying))
        {
            if (GUILayout.Button("Apply and start trial"))
                StartTrial();
        }

        using (new EditorGUI.DisabledScope(!IncrementalContactPipelineCsvRecorderRuntime.IsRecording))
        {
            if (GUILayout.Button("Stop and write summary"))
                IncrementalContactPipelineCsvRecorderRuntime.Stop(writeSummary: true);
        }

        if (!EditorApplication.isPlaying)
        {
            EditorGUILayout.HelpBox(
                "Enter Play Mode before starting a trial. The recorder consumes one completed ECS snapshot per timestep.",
                MessageType.Warning);
        }

        string rawPath = IncrementalContactPipelineCsvRecorderRuntime.RawPath;
        if (!string.IsNullOrEmpty(rawPath))
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Raw CSV", rawPath, EditorStyles.wordWrappedLabel);
            EditorGUILayout.LabelField("Summary CSV",
                IncrementalContactPipelineCsvRecorderRuntime.SummaryPath,
                EditorStyles.wordWrappedLabel);
        }
    }

    private void StartTrial()
    {
        ApplyRuntimeOverrides();
        string directory = ResolveOutputDirectory(_outputDirectory);
        string safeExperiment = SanitizeFileName(_experimentId);
        string safeScenario = SanitizeFileName(_scenario);
        string safeConfiguration = SanitizeFileName(_configurationLabel);
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName =
            $"{safeExperiment}_{safeScenario}_{safeConfiguration}_{timestamp}.csv";
        string path = Path.Combine(directory, fileName);
        IncrementalContactPipelineCsvRecorderRuntime.Start(
            path,
            _warmupFrames,
            _measuredFrames);
    }

    private void ApplyRuntimeOverrides()
    {
        IncrementalContactPipelineExperimentRuntime.OverrideEnabled = true;
        IncrementalContactPipelineExperimentRuntime.TimestepCacheEnabled =
            _timestepCacheEnabled;
        IncrementalContactPipelineExperimentRuntime.CrossFrameContactCacheEnabled =
            _crossFrameTopologyEnabled && _timestepCacheEnabled;
        IncrementalContactPipelineExperimentRuntime.PredictiveContactsEnabled =
            _predictiveContactsEnabled;
        IncrementalContactPipelineExperimentRuntime.SubstepCount = _substeps;
        IncrementalContactPipelineExperimentRuntime.IterationCount = _iterations;
        IncrementalContactPipelineExperimentRuntime.GuardEnvelopeMargin = _guardMargin;
        IncrementalContactPipelineExperimentRuntime.PredictiveSkin = _predictiveSkin;
        IncrementalContactPipelineExperimentRuntime.TimestepContactMargin = _contactMargin;
        IncrementalContactPipelineExperimentRuntime.ExperimentId = _experimentId;
        IncrementalContactPipelineExperimentRuntime.Scenario = _scenario;
        IncrementalContactPipelineExperimentRuntime.ConfigurationLabel =
            _configurationLabel;
    }

    private void ApplyPreset(TrialPreset preset)
    {
        _predictiveContactsEnabled = true;
        // 诊断/oracle 不再由预设控制：EnableDiagnostics 是 Simulation Debugger 面板
        // 和 FlowFieldManager 组件上对用户可见的开关。如果基准测试需要纯净的 O(N²) 关闭基线，
        // 请在那边切换。
        _substeps = 4;
        _iterations = 4;
        _predictiveSkin = 0.05f;
        _contactMargin = 0.02f;

        switch (preset)
        {
            case TrialPreset.FullSweptPerSubstep:
                _configurationLabel = "full_swept_per_substep";
                _timestepCacheEnabled = false;
                _crossFrameTopologyEnabled = false;
                _guardMargin = 0f;
                break;
            case TrialPreset.TimestepSweptNoFrameReuse:
                _configurationLabel = "timestep_swept_no_frame_reuse";
                _timestepCacheEnabled = true;
                _crossFrameTopologyEnabled = false;
                _guardMargin = 0.5f;
                break;
            case TrialPreset.IncrementalTightGuard:
                _configurationLabel = "incremental_guard_0.05";
                _timestepCacheEnabled = true;
                _crossFrameTopologyEnabled = true;
                _guardMargin = 0.05f;
                break;
            case TrialPreset.IncrementalDefault:
                _configurationLabel = "incremental_guard_0.5";
                _timestepCacheEnabled = true;
                _crossFrameTopologyEnabled = true;
                _guardMargin = 0.5f;
                break;
            case TrialPreset.IncrementalWideGuard:
                _configurationLabel = "incremental_guard_2.0";
                _timestepCacheEnabled = true;
                _crossFrameTopologyEnabled = true;
                _guardMargin = 2f;
                break;
            case TrialPreset.Custom:
                _configurationLabel = "custom";
                break;
        }
    }

    private void DrawPresetMeaning()
    {
        string message;
        switch (_preset)
        {
            case TrialPreset.FullSweptPerSubstep:
                message = "Disables the timestep cache. This is the available full-swept baseline after the legacy Fat/Adaptive path was retired.";
                break;
            case TrialPreset.TimestepSweptNoFrameReuse:
                message = "Primary A/B baseline: one swept contact-set build per timestep (B on), with cross-frame topology disabled (A off).";
                break;
            case TrialPreset.IncrementalTightGuard:
                message = "Tests low candidate inflation with more frequent topology dirtiness and repairs.";
                break;
            case TrialPreset.IncrementalDefault:
                message = "Default incremental topology, predictive lifecycle and dormant scheduling configuration.";
                break;
            case TrialPreset.IncrementalWideGuard:
                message = "Tests high topology reuse against neighbor-pair inflation and low candidate utilization.";
                break;
            default:
                message = "Custom values are applied directly to the solver after legacy debugger overrides.";
                break;
        }
        EditorGUILayout.HelpBox(message, MessageType.None);
    }

    private static string ResolveOutputDirectory(string configured)
    {
        string value = string.IsNullOrWhiteSpace(configured)
            ? "Diagnostics/IncrementalContact"
            : configured.Trim();
        if (Path.IsPathRooted(value))
            return value;
        return Path.GetFullPath(Path.Combine(Application.dataPath, "..", value));
    }

    private static string SanitizeFileName(string value)
    {
        string resolved = string.IsNullOrWhiteSpace(value) ? "unnamed" : value.Trim();
        foreach (char invalid in Path.GetInvalidFileNameChars())
            resolved = resolved.Replace(invalid, '_');
        return resolved.Replace(' ', '_');
    }

    private void OnSessionCompleted(string rawPath, string summaryPath)
    {
        EditorApplication.delayCall += () =>
        {
            Repaint();
            Debug.Log($"Incremental contact benchmark complete.\nRaw: {rawPath}\nSummary: {summaryPath}");
            AssetDatabase.Refresh();
        };
    }
}
}
#endif
