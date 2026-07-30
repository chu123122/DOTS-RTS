using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct ValidateConsumerViewsJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<ContactConstraint> TimestepContactPairs;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry>
        PredictiveContactSchedule;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation>
        InteractionCertificateViolations;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int SubstepIndex;
    public byte RequireDirtyBodies;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
#endif

    public void Execute()
    {
        if (RequireDirtyBodies != 0 &&
            DirtyBodies.Length == 0)
            return;

        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0)
            return;
        ContactSolverSkipReason failure =
            InteractionCertificateKernel.GetConsumerCertificateFailure(
            SubstepIndex,
            Configuration,
            Bodies,
            SoftAvoidancePairs,
            TimestepContactPairs,
            PredictiveContactSchedule,
            InteractionCertificate);
        if (failure == ContactSolverSkipReason.None)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        statistics.SolverSkipReason = failure;
        statistics.SolverSkippedSubstepCount++;
        Statistics.Value = statistics;
#endif
        if (InteractionCertificate.IsCreated)
        {
            InteractionCertificate certificate =
                InteractionCertificate.Value;
            certificate.Flags &= ~InteractionCertificationFlags.Issued;
            InteractionCertificate.Value = certificate;
        }
        if (InteractionCertificateViolations.IsCreated)
        {
            InteractionCertificateViolations.Add(
                new InteractionCertificateViolation
                {
                    BodyIndex = -1,
                    FirstInvalidSubstep =
                        (ushort)Unity.Mathematics.math.max(0, SubstepIndex),
                    Reason = InteractionCertificateViolationReason
                        .CommittedViewMismatch
                });
        }
        runtime.IsValid = 0;
        runtime.RecoveryRequired = 1;
        RuntimeState.Value = runtime;
    }
}
}
