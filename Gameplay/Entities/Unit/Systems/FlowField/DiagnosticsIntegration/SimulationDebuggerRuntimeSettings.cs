using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private void ApplySimulationDebuggerRuntimeOverrides(
        ref UnitContactSolverSettings solverSettings)
    {
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        SimulationDebuggerEffectiveSettings current = BuildEffectiveSettings(
            solverSettings,
            AdaptiveFatAabbSettings.Default);
        SimulationDebuggerRuntime.CaptureBaselineSettings(worldId, current);

        if (!SimulationDebuggerRuntime.TryConsumeSettingsRequest(
                worldId,
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
        solverSettings.EnablePersistentContactCache = requested.EnablePersistentContactCache != 0;
        solverSettings.EnableTimestepContactSetCache =
            requested.EnableTimestepContactSetCache != 0;
        solverSettings.PersistentGuardEnvelopeMargin = math.max(0f, requested.PersistentGuardEnvelopeMargin);
        solverSettings.TimestepContactMargin = math.max(0f, requested.TimestepContactMargin);
        solverSettings.EnableDiagnostics = requested.EnableDiagnostics != 0;

        solverSettings.SoftAvoidanceResponseRate =
            math.max(0f, requested.SoftAvoidanceResponseRate);
        solverSettings.SoftAvoidanceShell =
            math.max(0f, requested.SoftAvoidanceShell);
        solverSettings.SettledSoftAvoidanceMultiplier = math.max(
            0f,
            requested.SettledSoftAvoidanceMultiplier);
        solverSettings.SoftAvoidanceVelocitySolver =
            (SoftAvoidanceVelocitySolverMode)math.clamp(requested.SoftAvoidanceVelocitySolver, 0, 1);
        solverSettings.RvoTimeHorizon =
            math.max(0.01f, requested.RvoTimeHorizon);

        SystemAPI.SetSingleton(solverSettings);
    }

    private static SimulationDebuggerEffectiveSettings BuildEffectiveSettings(
        UnitContactSolverSettings solverSettings,
        AdaptiveFatAabbSettings adaptiveSettings)
    {
        bool timestepCacheEnabled = solverSettings.EnableTimestepContactSetCache;
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
            EnablePersistentContactCache =
                (byte)(solverSettings.EnablePersistentContactCache && timestepCacheEnabled ? 1 : 0),
            EnableTimestepContactSetCache =
                (byte)(timestepCacheEnabled ? 1 : 0),
            PersistentGuardEnvelopeMargin = solverSettings.PersistentGuardEnvelopeMargin,
            TimestepContactMargin = solverSettings.TimestepContactMargin,
            EnableDiagnostics =
                (byte)(solverSettings.EnableDiagnostics ? 1 : 0),
            SoftAvoidanceResponseRate =
                solverSettings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = solverSettings.SoftAvoidanceShell,
            SettledSoftAvoidanceMultiplier =
                solverSettings.SettledSoftAvoidanceMultiplier,
            SoftAvoidanceVelocitySolver =
                (int)solverSettings.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = solverSettings.RvoTimeHorizon,
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
