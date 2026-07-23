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
    public static bool OverrideEnabled;
    public static bool TimestepCacheEnabled = true;
    public static bool CrossFrameContactCacheEnabled = true;
    public static bool PredictiveContactsEnabled = true;
    public static bool DiagnosticsEnabled = true;

    public static int SubstepCount = 4;
    public static int IterationCount = 4;
    public static float GuardEnvelopeMargin = 0.5f;
    public static float PredictiveSkin = 0.05f;
    public static float TimestepContactMargin = 0.02f;

    public static string ExperimentId = "manual";
    public static string Scenario = "unspecified";
    public static string ConfigurationLabel = "runtime";

    public static void Apply(ref UnitContactSolverSettings settings)
    {
        if (!OverrideEnabled)
            return;

        settings.SubstepCount = SubstepCount < 1 ? 1 : SubstepCount;
        settings.IterationCount = IterationCount < 1 ? 1 : IterationCount;
        settings.FatAabbCacheMargin = GuardEnvelopeMargin < 0f ? 0f : GuardEnvelopeMargin;
        settings.PredictiveSkin = PredictiveSkin < 0f ? 0f : PredictiveSkin;
        settings.TimestepContactMargin = TimestepContactMargin < 0f ? 0f : TimestepContactMargin;
        settings.EnablePredictiveContacts = PredictiveContactsEnabled;
        settings.EnableFatAabbCache = CrossFrameContactCacheEnabled && TimestepCacheEnabled;
        settings.EnableDiagnostics = DiagnosticsEnabled;
    }

    public static IncrementalContactPipelineConfiguration CaptureConfiguration(
        int unitCount,
        float deltaTime,
        float softAvoidanceShell,
        UnitContactSolverSettings settings,
        bool effectiveTimestepCacheEnabled,
        bool effectiveCrossFrameTopologyEnabled)
    {
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
    }

    private static FixedString64Bytes ToFixedString(string value, string fallback)
    {
        string resolved = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return new FixedString64Bytes(resolved);
    }
}
}
