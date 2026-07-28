#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEngine;

namespace RTS.Unit.FlowField.Editor
{
public enum SimulationDiagnosticsBuildProfile : byte
{
    Custom,
    EditorDiagnostics,
    DevelopmentDiagnostics,
    ReleaseGameplayOnly
}

/// <summary>
/// 仅编辑器的资源，管理 RTS_CONTACT_DIAGNOSTICS 脚本宏。应用配置会更新 PlayerSettings 并触发 Unity 脚本重编译。该资源本身不会打包进玩家程序集。
/// </summary>
[CreateAssetMenu(
    fileName = "SimulationDiagnosticsBuildSettings",
    menuName = "RTS/Diagnostics/Build Settings")]
public sealed class SimulationDiagnosticsBuildSettings : ScriptableObject
{
    public const string DiagnosticsDefine = "RTS_CONTACT_DIAGNOSTICS";

    [Tooltip("通过该资源最后应用的配置。")]
    public SimulationDiagnosticsBuildProfile Profile =
        SimulationDiagnosticsBuildProfile.EditorDiagnostics;

    [Header("Custom / Development Targets")]
    public bool Standalone = true;
    public bool Android;
    public bool IOS;
    public bool WebGL;

    [Tooltip(
        "编辑器诊断为当前激活的构建目标启用脚本宏。Unity 在编译 Play Mode 时使用激活目标的脚本宏。")]
    public bool IncludeActiveTargetForEditor = true;
}

[CustomEditor(typeof(SimulationDiagnosticsBuildSettings))]
public sealed class SimulationDiagnosticsBuildSettingsEditor : UnityEditor.Editor
{
    [MenuItem("RTS/Diagnostics/Select Build Settings")]
    private static void SelectBuildSettings()
    {
        string[] guids = AssetDatabase.FindAssets(
            "t:SimulationDiagnosticsBuildSettings");
        if (guids.Length == 0)
        {
            Debug.LogWarning(
                "SimulationDiagnosticsBuildSettings asset is missing.");
            return;
        }

        string path = AssetDatabase.GUIDToAssetPath(guids[0]);
        Selection.activeObject =
            AssetDatabase.LoadAssetAtPath<SimulationDiagnosticsBuildSettings>(path);
        EditorGUIUtility.PingObject(Selection.activeObject);
    }

    private static readonly BuildTargetGroup[] SupportedGroups =
    {
        BuildTargetGroup.Standalone,
        BuildTargetGroup.Android,
        BuildTargetGroup.iOS,
        BuildTargetGroup.WebGL
    };

    public override void OnInspectorGUI()
    {
        var settings = (SimulationDiagnosticsBuildSettings)target;
        serializedObject.Update();

        DrawCompileStatus();
        EditorGUILayout.LabelField("Last applied profile", settings.Profile.ToString());
        EditorGUILayout.Space(8f);
        DrawProfileButtons(settings);
        EditorGUILayout.Space(8f);

        EditorGUILayout.LabelField("Custom / Development Targets", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(settings.Standalone)));
        EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(settings.Android)));
        EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(settings.IOS)));
        EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(settings.WebGL)));
        EditorGUILayout.PropertyField(
            serializedObject.FindProperty(nameof(settings.IncludeActiveTargetForEditor)));

        serializedObject.ApplyModifiedProperties();

        using (new EditorGUI.DisabledScope(EditorApplication.isCompiling))
        {
            if (GUILayout.Button("Apply Custom Targets And Recompile", GUILayout.Height(28f)))
            {
                settings.Profile = SimulationDiagnosticsBuildProfile.Custom;
                ApplyCustom(settings);
            }
        }

        EditorGUILayout.Space(8f);
        DrawTargetStatus();
        EditorGUILayout.Space(4f);
        EditorGUILayout.HelpBox(
            "Scripting Define Symbols are compile-time settings. Applying a profile " +
            "causes Unity to recompile. “Editor Diagnostics” uses the active build " +
            "target because Unity does not expose an independent Editor-only symbol set. " +
            "Apply “Release Gameplay Only” before producing a release build.",
            MessageType.Info);
    }

    private static void DrawCompileStatus()
    {
        EditorGUILayout.LabelField("RTS Contact Diagnostics", EditorStyles.boldLabel);
        EditorGUILayout.LabelField(
            "Active build target",
            $"{EditorUserBuildSettings.activeBuildTarget} / " +
            $"{EditorUserBuildSettings.selectedBuildTargetGroup}");
#if RTS_CONTACT_DIAGNOSTICS
        EditorGUILayout.HelpBox(
            "Current editor compilation: Enabled",
            MessageType.Info);
#else
        EditorGUILayout.HelpBox(
            "Current editor compilation: Disabled",
            MessageType.None);
#endif
        if (EditorApplication.isCompiling)
            EditorGUILayout.HelpBox("Unity is recompiling scripts…", MessageType.Warning);
    }

    private static void DrawProfileButtons(
        SimulationDiagnosticsBuildSettings settings)
    {
        EditorGUILayout.LabelField("Presets", EditorStyles.boldLabel);
        using (new EditorGUI.DisabledScope(EditorApplication.isCompiling))
        {
            if (GUILayout.Button("Editor Diagnostics", GUILayout.Height(25f)))
            {
                settings.Profile = SimulationDiagnosticsBuildProfile.EditorDiagnostics;
                ApplyEditorDiagnostics(settings);
            }

            if (GUILayout.Button("Development Diagnostics", GUILayout.Height(25f)))
            {
                settings.Profile = SimulationDiagnosticsBuildProfile.DevelopmentDiagnostics;
                ApplyDevelopmentDiagnostics(settings);
            }

            if (GUILayout.Button("Release Gameplay Only", GUILayout.Height(25f)))
            {
                settings.Profile = SimulationDiagnosticsBuildProfile.ReleaseGameplayOnly;
                ApplyReleaseGameplayOnly(settings);
            }
        }
    }

    private static void ApplyEditorDiagnostics(
        SimulationDiagnosticsBuildSettings settings)
    {
        DisableSupportedTargets();
        if (settings.IncludeActiveTargetForEditor)
            SetDiagnosticsDefine(ActiveNamedBuildTarget(), true);
        SaveAndRefresh(settings);
    }

    private static void ApplyDevelopmentDiagnostics(
        SimulationDiagnosticsBuildSettings settings)
    {
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.Standalone),
            settings.Standalone);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.Android),
            settings.Android);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.iOS),
            settings.IOS);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.WebGL),
            settings.WebGL);

        if (settings.IncludeActiveTargetForEditor)
            SetDiagnosticsDefine(ActiveNamedBuildTarget(), true);
        SaveAndRefresh(settings);
    }

    private static void ApplyReleaseGameplayOnly(
        SimulationDiagnosticsBuildSettings settings)
    {
        DisableSupportedTargets();
        SetDiagnosticsDefine(ActiveNamedBuildTarget(), false);
        SaveAndRefresh(settings);
    }

    private static void ApplyCustom(SimulationDiagnosticsBuildSettings settings)
    {
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.Standalone),
            settings.Standalone);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.Android),
            settings.Android);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.iOS),
            settings.IOS);
        SetDiagnosticsDefine(
            NamedBuildTarget.FromBuildTargetGroup(BuildTargetGroup.WebGL),
            settings.WebGL);
        SaveAndRefresh(settings);
    }

    private static void DisableSupportedTargets()
    {
        foreach (BuildTargetGroup group in SupportedGroups)
            SetDiagnosticsDefine(NamedBuildTarget.FromBuildTargetGroup(group), false);
    }

    private static NamedBuildTarget ActiveNamedBuildTarget()
    {
        return NamedBuildTarget.FromBuildTargetGroup(
            EditorUserBuildSettings.selectedBuildTargetGroup);
    }

    private static void SetDiagnosticsDefine(
        NamedBuildTarget target,
        bool enabled)
    {
        if (target == NamedBuildTarget.Unknown)
            return;

        string current = PlayerSettings.GetScriptingDefineSymbols(target);
        var symbols = new HashSet<string>(
            current.Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries)
                .Select(symbol => symbol.Trim())
                .Where(symbol => !string.IsNullOrEmpty(symbol)),
            StringComparer.Ordinal);

        bool changed = enabled
            ? symbols.Add(SimulationDiagnosticsBuildSettings.DiagnosticsDefine)
            : symbols.Remove(SimulationDiagnosticsBuildSettings.DiagnosticsDefine);
        if (!changed)
            return;

        PlayerSettings.SetScriptingDefineSymbols(
            target,
            string.Join(";", symbols.OrderBy(symbol => symbol, StringComparer.Ordinal)));
    }

    private static bool HasDiagnosticsDefine(NamedBuildTarget target)
    {
        if (target == NamedBuildTarget.Unknown)
            return false;
        string current = PlayerSettings.GetScriptingDefineSymbols(target);
        return current.Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries)
            .Any(symbol =>
                string.Equals(
                    symbol.Trim(),
                    SimulationDiagnosticsBuildSettings.DiagnosticsDefine,
                    StringComparison.Ordinal));
    }

    private static void DrawTargetStatus()
    {
        EditorGUILayout.LabelField("Current Target Symbols", EditorStyles.boldLabel);
        DrawTargetStatus("Standalone", BuildTargetGroup.Standalone);
        DrawTargetStatus("Android", BuildTargetGroup.Android);
        DrawTargetStatus("iOS", BuildTargetGroup.iOS);
        DrawTargetStatus("WebGL", BuildTargetGroup.WebGL);
    }

    private static void DrawTargetStatus(string label, BuildTargetGroup group)
    {
        NamedBuildTarget target = NamedBuildTarget.FromBuildTargetGroup(group);
        EditorGUILayout.LabelField(
            label,
            HasDiagnosticsDefine(target) ? "Enabled" : "Disabled");
    }

    private static void SaveAndRefresh(
        SimulationDiagnosticsBuildSettings settings)
    {
        EditorUtility.SetDirty(settings);
        AssetDatabase.SaveAssets();
        RepaintAllInspectors();
    }

    private static void RepaintAllInspectors()
    {
        foreach (UnityEditor.Editor editor in Resources.FindObjectsOfTypeAll<UnityEditor.Editor>())
            editor.Repaint();
    }
}
}
#endif
