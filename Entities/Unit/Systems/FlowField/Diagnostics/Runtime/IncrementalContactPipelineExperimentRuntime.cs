using Unity.Collections;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Managed experiment overrides used by the migrated benchmark tuner. They are
/// applied after the legacy debugger overrides so every recorded snapshot
/// carries the settings that actually reached the solver.
/// </summary>
public static class IncrementalContactPipelineExperimentRuntime
{
#if RTS_CONTACT_DIAGNOSTICS
    public static bool OverrideEnabled;
    public static bool TimestepCacheEnabled = true;
    public static bool CrossFrameContactCacheEnabled = true;
    public static bool PredictiveContactsEnabled = true;
    public static bool DiagnosticsEnabled = true;

    public static int SubstepCount = 4;
    public static int IterationCount = 4;
    public static ContactPositionSolverMode ContactPositionSolver =
        ContactPositionSolverMode.GaussSeidel;
    public static float GuardEnvelopeMargin = 0.5f;
    public static float PredictiveSkin = 0.05f;
    public static float TimestepContactMargin = 0.02f;

    public static string ExperimentId = "manual";
    public static string Scenario = "unspecified";
    public static string ConfigurationLabel = "runtime";
#else
    public static bool OverrideEnabled { get => false; set { } }
    public static bool TimestepCacheEnabled { get => true; set { } }
    public static bool CrossFrameContactCacheEnabled { get => true; set { } }
    public static bool PredictiveContactsEnabled { get => true; set { } }
    public static bool DiagnosticsEnabled { get => false; set { } }
    public static int SubstepCount { get => 4; set { } }
    public static int IterationCount { get => 4; set { } }
    public static ContactPositionSolverMode ContactPositionSolver
    {
        get => ContactPositionSolverMode.GaussSeidel;
        set { }
    }
    public static float GuardEnvelopeMargin { get => 0.5f; set { } }
    public static float PredictiveSkin { get => 0.05f; set { } }
    public static float TimestepContactMargin { get => 0.02f; set { } }
    public static string ExperimentId { get => "disabled"; set { } }
    public static string Scenario { get => "disabled"; set { } }
    public static string ConfigurationLabel { get => "gameplay"; set { } }
#endif

    public static void Apply(ref UnitContactSolverSettings settings)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (!OverrideEnabled)
            return;

        settings.SubstepCount = SubstepCount < 1 ? 1 : SubstepCount;
        settings.IterationCount = IterationCount < 1 ? 1 : IterationCount;
        settings.ContactPositionSolver = ContactPositionSolver;
        settings.FatAabbCacheMargin = GuardEnvelopeMargin < 0f ? 0f : GuardEnvelopeMargin;
        settings.PredictiveSkin = PredictiveSkin < 0f ? 0f : PredictiveSkin;
        settings.TimestepContactMargin = TimestepContactMargin < 0f ? 0f : TimestepContactMargin;
        settings.EnablePredictiveContacts = PredictiveContactsEnabled;
        settings.EnableFatAabbCache = CrossFrameContactCacheEnabled && TimestepCacheEnabled;
        settings.EnableDiagnostics = DiagnosticsEnabled;
#endif
    }

    public static IncrementalContactPipelineConfiguration CaptureConfiguration(
        int unitCount,
        float deltaTime,
        float softAvoidanceShell,
        UnitContactSolverSettings settings,
        bool effectiveTimestepCacheEnabled,
        bool effectiveCrossFrameTopologyEnabled)
    {
#if RTS_CONTACT_DIAGNOSTICS
        return new IncrementalContactPipelineConfiguration
        {
            ExperimentId = ToFixedString(ExperimentId, "manual"),
            Scenario = ToFixedString(Scenario, "unspecified"),
            ConfigurationLabel = ToFixedString(ConfigurationLabel, "runtime"),
            UnitCount = unitCount,
            SubstepCount = settings.SubstepCount,
            IterationCount = settings.IterationCount,
            ContactPositionSolver = (byte)settings.ContactPositionSolver,
            DeltaTime = deltaTime,
            GuardEnvelopeMargin = settings.FatAabbCacheMargin,
            PredictiveSkin = settings.PredictiveSkin,
            TimestepContactMargin = settings.TimestepContactMargin,
            SoftAvoidanceShell = softAvoidanceShell,
            TimestepCacheEnabled = (byte)(effectiveTimestepCacheEnabled ? 1 : 0),
            CrossFrameTopologyEnabled = (byte)(effectiveCrossFrameTopologyEnabled ? 1 : 0),
            PredictiveContactsEnabled = (byte)(settings.EnablePredictiveContacts ? 1 : 0),
            DiagnosticsEnabled = (byte)(settings.EnableDiagnostics ? 1 : 0)
        };
#else
        return default;
#endif
    }

#if RTS_CONTACT_DIAGNOSTICS
    private static FixedString64Bytes ToFixedString(string value, string fallback)
    {
        string resolved = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return new FixedString64Bytes(resolved);
    }
#endif
}
}
