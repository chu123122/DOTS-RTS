using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

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

    private ContactViewBuildResult InferCommittedViewBuildResult(
        IncrementalContactPipelineStatistics incrementalStatistics)
    {
        ContactInteractionSourceMode sourceMode =
            ContactInteractionSourceMode.FullSweep;
        bool persistent = EnablePersistentContactCache &&
                          IncrementalCacheState.IsCreated &&
                          IncrementalCacheState.Value.IsValid != 0;
        if (persistent)
        {
            if (incrementalStatistics.UsedFullRebuild != 0)
                sourceMode = ContactInteractionSourceMode.PersistentFullRebuild;
            else if (incrementalStatistics.IncrementalRepairCount > 0)
                sourceMode = ContactInteractionSourceMode.PersistentRepair;
            else
                sourceMode = ContactInteractionSourceMode.PersistentReuse;
        }

        int interactionPairCount =
            incrementalStatistics.CurrentInteractionPairCount > 0
                ? incrementalStatistics.CurrentInteractionPairCount
                : persistent
                    ? PersistentNeighborPairs.Length
                    : TimestepInteractionPairs.Length;
        return new ContactViewBuildResult
        {
            SourceMode = sourceMode,
            PersistentViewReady = (byte)(persistent ? 1 : 0),
            UsedFullRebuild = (byte)(
                incrementalStatistics.UsedFullRebuild != 0 ? 1 : 0),
            RepairedBodyCount = sourceMode ==
                                ContactInteractionSourceMode.PersistentRepair
                ? math.max(0, incrementalStatistics.TopologyDirtyBodyCount)
                : 0,
            InteractionPairCount = interactionPairCount
        };
    }

    /// <summary>
    /// Common commit hook used by both the serial reference path and the staged
    /// P1-P6 Jacobi path. Every consumer-visible compact view therefore receives
    /// an explicit certificate even when the caller bypasses the serial source
    /// resolver.
    /// </summary>
    private void IssueCertificateForCommittedViews(
        IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep = 0)
    {
        IssueInteractionCertificate(
            InferCommittedViewBuildResult(incrementalStatistics),
            scheduleStartSubstep);
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
