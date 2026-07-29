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
public partial struct InteractionCertificationJob
{
    private uint CalculateBodySetFingerprint()
    {
        uint fingerprint = 2166136261u;
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            Entity entity = Bodies[bodyIndex].Entity;
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

    private InteractionCertificationEvidence BuildCertificationEvidence(
        int startSubstep,
        int endSubstepExclusive)
    {
        IncrementalContactCacheState cacheState = IncrementalCacheState.IsCreated
            ? IncrementalCacheState.Value
            : default;
        int substepCount = math.max(1, SubstepCount);
        return new InteractionCertificationEvidence
        {
            WorldId = Configuration.WorldId,
            SimulationStepId = Configuration.SimulationStepId,
            BodySetFingerprint = CalculateBodySetFingerprint(),
            ConfigurationFingerprint =
                Configuration.CalculateCertificationFingerprint(),
            TopologyEpoch = cacheState.TopologyEpoch,
            ClassificationFingerprint = CalculateClassificationEpoch(),
            StartSubstep = (ushort)startSubstep,
            EndSubstepExclusive = (ushort)endSubstepExclusive,
            HorizonDuration = DeltaTime *
                              (endSubstepExclusive - startSubstep) /
                              substepCount,
            BodyCount = Bodies.Length
        };
    }

    private bool HasValidConsumerViewStructure()
    {
        return GetConsumerViewStructureFailure() == ContactSolverSkipReason.None;
    }

    private ContactSolverSkipReason GetConsumerViewStructureFailure()
    {
        if (!TimestepInteractionPairs.IsCreated ||
            !SoftAvoidancePairs.IsCreated ||
            !TimestepContactPairs.IsCreated ||
            !PredictiveContactSchedule.IsCreated)
            return ContactSolverSkipReason.CertificateViewUnavailable;

        int bodyCount = Bodies.Length;
        for (int i = 0; i < TimestepInteractionPairs.Length; i++)
        {
            BodyPair pair = TimestepInteractionPairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason.CertificateInteractionPairInvalid;
        }
        for (int i = 0; i < SoftAvoidancePairs.Length; i++)
        {
            BodyPair pair = SoftAvoidancePairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason.CertificateSoftPairInvalid;
        }
        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            ContactConstraint pair = TimestepContactPairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason.CertificateContactPairInvalid;
        }

        int substepCount = math.max(1, SubstepCount);
        for (int i = 0; i < PredictiveContactSchedule.Length; i++)
        {
            ushort scheduledSubstep = PredictiveContactSchedule[i].Substep;
            // ushort.MaxValue is the explicit "do not wake this timestep"
            // sentinel for dormant pairs without relative motion. It is valid
            // schedule state, not an out-of-range consumer view.
            if (scheduledSubstep != ushort.MaxValue &&
                scheduledSubstep >= substepCount)
                return ContactSolverSkipReason.CertificateScheduleInvalid;
        }
        return ContactSolverSkipReason.None;
    }

    private bool HasValidPersistentEntityMapping()
    {
        if (!CurrentBodyIndexByEntity.IsCreated)
            return false;
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            if (!CurrentBodyIndexByEntity.TryGetValue(
                    Bodies[bodyIndex].Entity,
                    out int mappedBodyIndex) ||
                mappedBodyIndex != bodyIndex)
                return false;
        }
        return true;
    }

    private InteractionCertificationFlags BuildCertificationFlags(
        ContactViewBuildResult result,
        InteractionCertificationEvidence evidence)
    {
        InteractionCertificationFlags flags = InteractionCertificationFlags.None;
        bool structureValid =
            result.InteractionPairCount >= 0 && HasValidConsumerViewStructure();
        if (structureValid)
            flags |= InteractionCertificationFlags.StructureVerified;

        bool persistentSource =
            result.SourceMode != ContactInteractionSourceMode.FullSweep;
        if (!persistentSource || HasValidPersistentEntityMapping())
            flags |= InteractionCertificationFlags.EntityMappingVerified;
        if (evidence.ConfigurationFingerprint != 0u)
            flags |= InteractionCertificationFlags.ConfigurationVerified;
        if (!persistentSource || result.PersistentViewReady != 0)
            flags |= InteractionCertificationFlags.TopologyCoverageVerified;
        if (!persistentSource ||
            (IncrementalCacheState.IsCreated &&
             IncrementalCacheState.Value.IsValid != 0))
            flags |= InteractionCertificationFlags.ClassificationVerified;
        if (structureValid)
            flags |= InteractionCertificationFlags.ConsumerViewsCommitted;
        return flags;
    }

    /// <summary>
    /// Common commit hook used by both XPBD solver backends. Every
    /// consumer-visible compact view therefore receives
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
        int end = EnableTimestepContactSetCache ? substepCount : 1;
        int start = EnableTimestepContactSetCache
            ? math.clamp(scheduleStartSubstep, 0, end)
            : 0;
        InteractionCertificationEvidence evidence =
            BuildCertificationEvidence(start, end);

        InteractionCertificationFlags flags =
            BuildCertificationFlags(result, evidence);
        ContactSolverSkipReason structureFailure =
            result.InteractionPairCount < 0
                ? ContactSolverSkipReason.CertificateInteractionCountInvalid
                : GetConsumerViewStructureFailure();
        const InteractionCertificationFlags required =
            InteractionCertificationFlags.StructureVerified |
            InteractionCertificationFlags.EntityMappingVerified |
            InteractionCertificationFlags.ConfigurationVerified |
            InteractionCertificationFlags.TopologyCoverageVerified |
            InteractionCertificationFlags.ClassificationVerified |
            InteractionCertificationFlags.ConsumerViewsCommitted;
        if ((flags & required) == required)
            flags |= InteractionCertificationFlags.Issued;

        InteractionCertificate.Value = new InteractionCertificate
        {
            WorldId = evidence.WorldId,
            SimulationStepId = evidence.SimulationStepId,
            BodySetFingerprint = evidence.BodySetFingerprint,
            ConfigurationFingerprint = evidence.ConfigurationFingerprint,
            TopologyEpoch = evidence.TopologyEpoch,
            ClassificationFingerprint = evidence.ClassificationFingerprint,
            StartSubstep = evidence.StartSubstep,
            EndSubstepExclusive = evidence.EndSubstepExclusive,
            HorizonDuration = evidence.HorizonDuration,
            SourceMode = ToCertifiedSourceMode(result.SourceMode),
            Flags = flags,
            StructureFailure = structureFailure,
            InteractionPairCount = math.max(0, result.InteractionPairCount),
            SoftPairCount = SoftAvoidancePairs.Length,
            ContactConstraintCount = TimestepContactPairs.Length,
            DormantScheduleCount = PredictiveContactSchedule.Length
        };
        if (InteractionCertificateViolations.IsCreated)
            InteractionCertificateViolations.Clear();
    }

    private ContactSolverSkipReason GetConsumerCertificateFailure(
        int substepIndex)
    {
        if (!InteractionCertificate.IsCreated)
            return ContactSolverSkipReason.CertificateUnavailable;

        InteractionCertificate certificate = InteractionCertificate.Value;
        int certificateSubstep = EnableTimestepContactSetCache
            ? substepIndex
            : 0;
        if (!certificate.IsIssued)
        {
            if ((certificate.Flags &
                 InteractionCertificationFlags.StructureVerified) == 0)
            {
                return certificate.StructureFailure != ContactSolverSkipReason.None
                    ? certificate.StructureFailure
                    : ContactSolverSkipReason.CertificateStructureNotVerified;
            }
            if ((certificate.Flags &
                 InteractionCertificationFlags.EntityMappingVerified) == 0)
                return ContactSolverSkipReason.CertificateEntityMappingNotVerified;
            if ((certificate.Flags &
                 InteractionCertificationFlags.ConfigurationVerified) == 0)
                return ContactSolverSkipReason.CertificateConfigurationNotVerified;
            if ((certificate.Flags &
                 InteractionCertificationFlags.TopologyCoverageVerified) == 0)
                return ContactSolverSkipReason.CertificateTopologyNotVerified;
            if ((certificate.Flags &
                 InteractionCertificationFlags.ClassificationVerified) == 0)
                return ContactSolverSkipReason.CertificateClassificationNotVerified;
            if ((certificate.Flags &
                 InteractionCertificationFlags.ConsumerViewsCommitted) == 0)
                return ContactSolverSkipReason.CertificateConsumerViewsNotCommitted;
            return ContactSolverSkipReason.CertificateNotIssued;
        }
        if (!certificate.Covers(
                Configuration.WorldId,
                Configuration.SimulationStepId,
                certificateSubstep))
            return ContactSolverSkipReason.CertificateScopeMismatch;
        if (certificate.BodySetFingerprint != CalculateBodySetFingerprint())
            return ContactSolverSkipReason.BodySetMismatch;
        if (certificate.ConfigurationFingerprint !=
            Configuration.CalculateCertificationFingerprint())
            return ContactSolverSkipReason.ConfigurationMismatch;
        if (certificate.SoftPairCount != SoftAvoidancePairs.Length)
            return ContactSolverSkipReason.SoftPairCountMismatch;
        if (certificate.ContactConstraintCount != TimestepContactPairs.Length)
            return ContactSolverSkipReason.ContactConstraintCountMismatch;
        if (certificate.DormantScheduleCount != PredictiveContactSchedule.Length)
            return ContactSolverSkipReason.DormantScheduleCountMismatch;
        return ContactSolverSkipReason.None;
    }

    private void RecordSolverSkip(ContactSolverSkipReason reason)
    {
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        statistics.SolverSkipReason = reason;
        statistics.SolverSkippedSubstepCount++;
        StoreContactStatistics(statistics);
#endif
    }

    private void ValidateConsumerViews()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0)
            return;
        ContactSolverSkipReason failure =
            GetConsumerCertificateFailure(SubstepIndex);
        if (failure == ContactSolverSkipReason.None)
            return;

        RecordSolverSkip(failure);
        RevokeInteractionCertificate(
            -1,
            SubstepIndex,
            InteractionCertificateViolationReason.CommittedViewMismatch,
            default,
            default);
        runtime.IsValid = 0;
        runtime.RecoveryRequired = 1;
        RuntimeState.Value = runtime;
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
