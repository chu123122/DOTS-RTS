using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{
    private void ExecuteInitializeSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentProxyIndexByBody.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            IncrementalCacheState.Value = default;
        }
    }

    private void ExecuteBuildInitialSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0 || !EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        PrepareTimestepContactPrediction(DeltaTime, false);
        if (EnablePersistentContactCache)
            PrepareInitialPersistentDirtyBodySet();
        long start = ProfilerUnsafeUtility.Timestamp;
        BuildOrRefreshTimestepContactViews(
            ref statistics,
            ref incremental,
            false,
            false);
        statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - start);
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteBuildSubstepInteractionSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0 || EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        float dt = DeltaTime / math.max(1, SubstepCount);
        PrepareTimestepContactPrediction(dt, true);
        long start = ProfilerUnsafeUtility.Timestamp;
        BuildSubstepInteractionAndSoftViews(ref statistics, ref incremental);
        statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - start);
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteValidateBaseMotionSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0 || !EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float dt = DeltaTime / substepCount;
        if (!ValidateBaseMotionInteractionEnvelope(
                SubstepIndex,
                ref statistics,
                ref incremental))
        {
            RepairOrRebuildContactViewForRemainingTime(
                SubstepIndex,
                substepCount,
                dt,
                true,
                ref statistics,
                ref incremental,
                false);
        }
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteClampSoftOutputSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        ClampSoftOutputToInteractionEnvelope(
            DeltaTime / math.max(1, SubstepCount),
            ref incremental);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteValidatePredictedAndActivateSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float dt = DeltaTime / substepCount;
        bool rebuilt = false;
        if (!ValidatePredictedContactEnvelope(
                SubstepIndex,
                ref statistics,
                ref incremental))
        {
            RepairOrRebuildContactViewForRemainingTime(
                SubstepIndex,
                substepCount,
                dt,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incremental,
                false);
            rebuilt = true;
        }
        if (!EnableTimestepContactSetCache && !rebuilt)
        {
            PrepareSubstepContactPrediction();
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incremental);
            statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }
        ActivateScheduledPredictiveContactsForSubstep(
            EnableTimestepContactSetCache ? SubstepIndex : 0,
            EnableTimestepContactSetCache ? substepCount : 1,
            ref incremental);
        ResetTimestepContactSetForSubstep();
        ActiveIncidentIndexState.Value = default;
        EnsureActiveConstraintIncidentIndexP1P6();
        statistics.TimestepContactSetSubstepUseCount++;
        control.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
#if RTS_CONTACT_DIAGNOSTICS
        control.IterationAccountedStartNanoseconds =
            AccountedCandidateNanoseconds(incremental);
#endif
        SerialControl.Value = control;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteValidateSolverCorrectionSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, SubstepCount);
        float dt = DeltaTime / substepCount;
        if (!ValidateSolverCorrectionContactEnvelope(
                SubstepIndex,
                ref statistics,
                ref incremental))
        {
            RepairOrRebuildContactViewForRemainingTime(
                SubstepIndex,
                substepCount,
                dt,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incremental);
            if (AfterContact == 0)
                ResetTimestepContactSetForSubstep();
            ActiveIncidentIndexState.Value = default;
            EnsureActiveConstraintIncidentIndexP1P6();
            if (AfterContact != 0 && IsLastIteration != 0)
                control.RecoveryRequired = 1;
        }
        SerialControl.Value = control;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }
}
}
