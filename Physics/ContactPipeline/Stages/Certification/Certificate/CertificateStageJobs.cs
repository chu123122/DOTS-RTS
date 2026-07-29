using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
    [BurstCompile]
    public struct ValidateConsumerViewsJob : IJob
    {
        public ContactPipelineConfiguration Configuration;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
        [ReadOnly] public NativeList<ContactConstraint> TimestepContactPairs;
        [ReadOnly] public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
        public NativeReference<InteractionCertificate> InteractionCertificate;
        public NativeList<InteractionCertificateViolation> InteractionCertificateViolations;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
#if RTS_CONTACT_DIAGNOSTICS
        public NativeReference<PredictiveDiscContactStatistics> Statistics;
#endif
        public void Execute()
        {
            var kernel = new CertificationStageKernel
            {
                Configuration = Configuration,
                Bodies = Bodies,
                SoftAvoidancePairs = SoftAvoidancePairs,
                TimestepContactPairs = TimestepContactPairs,
                PredictiveContactSchedule = PredictiveContactSchedule,
                InteractionCertificate = InteractionCertificate,
                InteractionCertificateViolations = InteractionCertificateViolations,
#if RTS_CONTACT_DIAGNOSTICS
                Statistics = Statistics
#endif
            };
            kernel.ValidateConsumerViews(SubstepIndex, RuntimeState);
        }
    }
}
}
