using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private void ApplySimulationDebuggerRuntimeOverrides(
        ref FlowFieldSettings flowSettings,
        ref UnitContactSolverSettings solverSettings)
    {
        SimulationDebuggerEffectiveSettings current = BuildEffectiveSettings(
            flowSettings,
            solverSettings,
            AdaptiveFatAabbSettings.Default);
        SimulationDebuggerRuntime.CaptureBaselineSettings(current);

        if (!SimulationDebuggerRuntime.TryConsumeSettingsRequest(
                out SimulationDebuggerEffectiveSettings requested,
                out _))
            return;

        solverSettings.SubstepCount = math.max(1, requested.SubstepCount);
        solverSettings.IterationCount = math.max(1, requested.IterationCount);
        solverSettings.ContactPositionSolver =
            (ContactPositionSolverMode)math.clamp(requested.ContactPositionSolver, 0, 1);
        solverSettings.Compliance = math.max(0f, requested.Compliance);
        solverSettings.PredictiveSkin = math.max(0f, requested.PredictiveSkin);
        solverSettings.EnablePredictivePairGeneration = requested.EnablePredictivePairGeneration != 0;
        solverSettings.EnablePredictiveContacts = requested.EnablePredictiveContacts != 0;
        solverSettings.EnableFatAabbCache = requested.EnableFatAabbCache != 0;
        SimulationDebuggerRuntime.TimestepContactSetCacheEnabled =
            requested.EnableTimestepContactSetCache != 0;
        solverSettings.FatAabbCacheMargin = math.max(0f, requested.FatAabbCacheMargin);
        solverSettings.EnableDiagnostics = requested.EnableDiagnostics != 0;

        flowSettings.SoftAvoidanceResponseRate = math.max(0f, requested.SoftAvoidanceResponseRate);
        flowSettings.SoftAvoidanceShell = math.max(0f, requested.SoftAvoidanceShell);
        flowSettings.SettledSoftAvoidanceMultiplier = math.max(
            0f,
            requested.SettledSoftAvoidanceMultiplier);
        flowSettings.SoftAvoidanceVelocitySolver =
            (SoftAvoidanceVelocitySolverMode)math.clamp(requested.SoftAvoidanceVelocitySolver, 0, 1);
        flowSettings.RvoTimeHorizon = math.max(0.01f, requested.RvoTimeHorizon);

        SystemAPI.SetSingleton(flowSettings);
        SystemAPI.SetSingleton(solverSettings);
    }

    private static SimulationDebuggerEffectiveSettings BuildEffectiveSettings(
        FlowFieldSettings flowSettings,
        UnitContactSolverSettings solverSettings,
        AdaptiveFatAabbSettings adaptiveSettings)
    {
        bool timestepCacheEnabled = SimulationDebuggerRuntime.TimestepContactSetCacheEnabled;
        return new SimulationDebuggerEffectiveSettings
        {
            SubstepCount = solverSettings.SubstepCount,
            IterationCount = solverSettings.IterationCount,
            ContactPositionSolver = (int)solverSettings.ContactPositionSolver,
            Compliance = solverSettings.Compliance,
            PredictiveSkin = solverSettings.PredictiveSkin,
            EnablePredictivePairGeneration =
                (byte)(solverSettings.EnablePredictivePairGeneration ? 1 : 0),
            EnablePredictiveContacts =
                (byte)(solverSettings.EnablePredictiveContacts ? 1 : 0),
            EnableFatAabbCache =
                (byte)(solverSettings.EnableFatAabbCache && timestepCacheEnabled ? 1 : 0),
            EnableTimestepContactSetCache =
                (byte)(timestepCacheEnabled ? 1 : 0),
            FatAabbCacheMargin = solverSettings.FatAabbCacheMargin,
            EnableDiagnostics =
                (byte)(solverSettings.EnableDiagnostics ? 1 : 0),
            SoftAvoidanceResponseRate = flowSettings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = flowSettings.SoftAvoidanceShell,
            SettledSoftAvoidanceMultiplier = flowSettings.SettledSoftAvoidanceMultiplier,
            SoftAvoidanceVelocitySolver = (int)flowSettings.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = flowSettings.RvoTimeHorizon,
            EnableAdaptiveFatAabb = adaptiveSettings.Enabled,
            AdaptiveDetectionCellSpan = adaptiveSettings.DetectionCellSpan,
            AdaptiveMinimumUnitsPerCell = adaptiveSettings.MinimumUnitsPerCell,
            AdaptiveMinimumUnitsPerRegion = adaptiveSettings.MinimumUnitsPerRegion,
            AdaptiveEnableScore = adaptiveSettings.EnableScore,
            AdaptiveDisableScore = adaptiveSettings.DisableScore
        };
    }
}
}
