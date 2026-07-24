using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// The single certification boundary for frame-local interaction products.
/// Persistent containers remain candidate data; lower stages consume only the
/// compact views whose scope is described by InteractionCertificate.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionCertificateViolations;

    private uint CalculateBodySetFingerprint()
    {
        uint fingerprint = 2166136261u;
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            Entity entity = States[bodyIndex].Entity;
            fingerprint = math.hash(new uint3(
                fingerprint,
                unchecked((uint)entity.Index),
                unchecked((uint)entity.Version)));
        }
        return fingerprint == 0u ? 1u : fingerprint;
    }

    private static CertifiedInteractionSourceMode ToCertifiedSourceMode(
        ContactInteractionSourceMode sourceMode)
    {
        switch (sourceMode)
        {
            case ContactInteractionSourceMode.PersistentReuse:
                return CertifiedInteractionSourceMode.PersistentReuse;
            case ContactInteractionSourceMode.PersistentRepair:
                return CertifiedInteractionSourceMode.PersistentRepair;
            case ContactInteractionSourceMode.PersistentFullRebuild:
                return CertifiedInteractionSourceMode.PersistentFullRebuild;
            default:
                return CertifiedInteractionSourceMode.FullSweep;
        }
    }

    private void IssueInteractionCertificate(
        ContactViewBuildResult result,
        int scheduleStartSubstep)
    {
        if (!InteractionCertificate.IsCreated)
            return;

        int substepCount = math.max(1, SubstepCount);
        int start = EnableTimestepContactSetCache
            ? math.clamp(scheduleStartSubstep, 0, substepCount - 1)
            : 0;
        int end = EnableTimestepContactSetCache
            ? substepCount
            : 1;
        IncrementalContactCacheState cacheState = IncrementalCacheState.IsCreated
            ? IncrementalCacheState.Value
            : default;

        InteractionCertificationFlags flags =
            InteractionCertificationFlags.StructureVerified |
            InteractionCertificationFlags.EntityMappingVerified |
            InteractionCertificationFlags.ConfigurationVerified |
            InteractionCertificationFlags.TopologyCoverageVerified |
            InteractionCertificationFlags.ClassificationVerified |
            InteractionCertificationFlags.ConsumerViewsCommitted |
            InteractionCertificationFlags.Issued;

        InteractionCertificate.Value = new InteractionCertificate
        {
            WorldId = Configuration.WorldId,
            SimulationStepId = Configuration.SimulationStepId,
            BodySetFingerprint = CalculateBodySetFingerprint(),
            ConfigurationFingerprint =
                Configuration.CalculateCertificationFingerprint(),
            TopologyEpoch = cacheState.TopologyEpoch,
            ClassificationFingerprint = CalculateClassificationEpoch(),
            StartSubstep = (ushort)start,
            EndSubstepExclusive = (ushort)end,
            HorizonDuration = DeltaTime * (end - start) / substepCount,
            SourceMode = ToCertifiedSourceMode(result.SourceMode),
            Flags = flags,
            InteractionPairCount = math.max(0, result.InteractionPairCount),
            SoftPairCount = SoftAvoidancePairs.Length,
            ContactConstraintCount = TimestepContactPairs.Length,
            DormantScheduleCount = PredictiveContactSchedule.Length
        };
        if (InteractionCertificateViolations.IsCreated)
            InteractionCertificateViolations.Clear();
    }

    private void IssueFullSweepSubstepCertificate()
    {
        IssueInteractionCertificate(
            new ContactViewBuildResult
            {
                SourceMode = ContactInteractionSourceMode.FullSweep,
                PersistentViewReady = 0,
                UsedFullRebuild = 0,
                RepairedBodyCount = 0,
                InteractionPairCount = TimestepInteractionPairs.Length
            },
            0);
    }

    private void RevokeInteractionCertificate(
        int bodyIndex,
        int substepIndex,
        InteractionCertificateViolationReason reason,
        float2 observedMin,
        float2 observedMax)
    {
        if (InteractionCertificate.IsCreated)
        {
            InteractionCertificate certificate = InteractionCertificate.Value;
            certificate.Flags &= ~InteractionCertificationFlags.Issued;
            InteractionCertificate.Value = certificate;
        }

        if (!InteractionCertificateViolations.IsCreated)
            return;
        InteractionCertificateViolations.Add(new InteractionCertificateViolation
        {
            BodyIndex = bodyIndex,
            FirstInvalidSubstep = (ushort)math.max(0, substepIndex),
            Reason = reason,
            ObservedMin = observedMin,
            ObservedMax = observedMax
        });
    }
}
}
