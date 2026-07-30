using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal enum ContactInteractionSourceMode : byte
{
    FullSweep,
    PersistentReuse,
    PersistentRepair,
    PersistentFullRebuild
}

/// <summary>
/// The single certification boundary for frame-local interaction products.
/// Persistent containers remain candidate data; lower stages consume only the
/// compact views whose scope is described by InteractionCertificate.
/// </summary>
internal static class InteractionCertificateKernel
{
    internal static uint CalculateBodySetFingerprint(
        NativeArray<CrowdBodySnapshot> bodies)
    {
        uint fingerprint = 2166136261u;
        for (int bodyIndex = 0; bodyIndex < bodies.Length; bodyIndex++)
        {
            Entity entity = bodies[bodyIndex].Entity;
            fingerprint = math.hash(new uint3(
                fingerprint,
                unchecked((uint)entity.Index),
                unchecked((uint)entity.Version)));
        }
        return fingerprint == 0u ? 1u : fingerprint;
    }

    internal static CertifiedInteractionSourceMode ToCertifiedSourceMode(
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













    /// <summary>
    /// Common commit hook used by both XPBD solver backends. Every
    /// consumer-visible compact view therefore receives
    /// an explicit certificate even when the caller bypasses the serial source
    /// resolver.
    /// </summary>


    internal static void IssueCertificateForCommittedViews(
        IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeList<BodyPair> timestepInteractionPairs,
        NativeList<BodyPair> softAvoidancePairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity,
        NativeList<PersistentNeighborPair> persistentNeighborPairs,
        NativeList<PredictiveContactScheduleEntry> predictiveContactSchedule,
        NativeReference<IncrementalContactCacheState> incrementalCacheState,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations,
        int scheduleStartSubstep = 0)
    {
        if (!interactionCertificate.IsCreated)
            return;

        IncrementalContactCacheState cacheState =
            incrementalCacheState.IsCreated
                ? incrementalCacheState.Value
                : default;
        bool persistent =
            configuration.EnablePersistentContactCache &&
            incrementalCacheState.IsCreated &&
            cacheState.IsValid != 0;
        ContactInteractionSourceMode sourceMode =
            ContactInteractionSourceMode.FullSweep;
        if (persistent)
        {
            sourceMode = incrementalStatistics.UsedFullRebuild != 0
                ? ContactInteractionSourceMode.PersistentFullRebuild
                : incrementalStatistics.IncrementalRepairCount > 0
                    ? ContactInteractionSourceMode.PersistentRepair
                    : ContactInteractionSourceMode.PersistentReuse;
        }

        int interactionPairCount =
            incrementalStatistics.CurrentInteractionPairCount > 0
                ? incrementalStatistics.CurrentInteractionPairCount
                : persistent
                    ? persistentNeighborPairs.Length
                    : timestepInteractionPairs.Length;
        int substepCount = math.max(1, configuration.SubstepCount);
        int end = configuration.EnableTimestepContactSetCache
            ? substepCount
            : 1;
        int start = configuration.EnableTimestepContactSetCache
            ? math.clamp(scheduleStartSubstep, 0, end)
            : 0;
        InteractionCertificationEvidence evidence =
            new InteractionCertificationEvidence
            {
                WorldId = configuration.WorldId,
                SimulationStepId = configuration.SimulationStepId,
                BodySetFingerprint = CalculateBodySetFingerprint(bodies),
                ConfigurationFingerprint =
                    configuration.CalculateCertificationFingerprint(),
                TopologyEpoch = cacheState.TopologyEpoch,
                ClassificationFingerprint =
                    ContactClassificationEpoch.Calculate(configuration),
                StartSubstep = (ushort)start,
                EndSubstepExclusive = (ushort)end,
                HorizonDuration = configuration.DeltaTime *
                                  (end - start) / substepCount,
                BodyCount = bodies.Length
            };

        ContactSolverSkipReason structureFailure =
            GetConsumerViewStructureFailure(
                bodies.Length,
                substepCount,
                timestepInteractionPairs,
                softAvoidancePairs,
                timestepContactPairs,
                predictiveContactSchedule);
        bool structureValid =
            interactionPairCount >= 0 &&
            structureFailure == ContactSolverSkipReason.None;
        bool persistentSource =
            sourceMode != ContactInteractionSourceMode.FullSweep;
        bool mappingValid =
            !persistentSource ||
            HasValidPersistentEntityMapping(
                bodies,
                currentBodyIndexByEntity);

        InteractionCertificationFlags flags =
            InteractionCertificationFlags.None;
        if (structureValid)
        {
            flags |= InteractionCertificationFlags.StructureVerified |
                     InteractionCertificationFlags.ConsumerViewsCommitted;
        }
        if (mappingValid)
            flags |= InteractionCertificationFlags.EntityMappingVerified;
        if (evidence.ConfigurationFingerprint != 0u)
            flags |= InteractionCertificationFlags.ConfigurationVerified;
        if (!persistentSource || persistent)
        {
            flags |=
                InteractionCertificationFlags.TopologyCoverageVerified;
        }
        if (!persistentSource || cacheState.IsValid != 0)
            flags |= InteractionCertificationFlags.ClassificationVerified;

        const InteractionCertificationFlags required =
            InteractionCertificationFlags.StructureVerified |
            InteractionCertificationFlags.EntityMappingVerified |
            InteractionCertificationFlags.ConfigurationVerified |
            InteractionCertificationFlags.TopologyCoverageVerified |
            InteractionCertificationFlags.ClassificationVerified |
            InteractionCertificationFlags.ConsumerViewsCommitted;
        if ((flags & required) == required)
            flags |= InteractionCertificationFlags.Issued;

        interactionCertificate.Value = new InteractionCertificate
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
            SourceMode = ToCertifiedSourceMode(sourceMode),
            Flags = flags,
            StructureFailure = interactionPairCount < 0
                ? ContactSolverSkipReason.CertificateInteractionCountInvalid
                : structureFailure,
            InteractionPairCount = math.max(0, interactionPairCount),
            SoftPairCount = softAvoidancePairs.Length,
            ContactConstraintCount = timestepContactPairs.Length,
            DormantScheduleCount = predictiveContactSchedule.Length
        };
        if (certificateViolations.IsCreated)
            certificateViolations.Clear();
    }

    private static ContactSolverSkipReason GetConsumerViewStructureFailure(
        int bodyCount,
        int substepCount,
        NativeList<BodyPair> timestepInteractionPairs,
        NativeList<BodyPair> softAvoidancePairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<PredictiveContactScheduleEntry> predictiveContactSchedule)
    {
        if (!timestepInteractionPairs.IsCreated ||
            !softAvoidancePairs.IsCreated ||
            !timestepContactPairs.IsCreated ||
            !predictiveContactSchedule.IsCreated)
            return ContactSolverSkipReason.CertificateViewUnavailable;

        for (int i = 0; i < timestepInteractionPairs.Length; i++)
        {
            BodyPair pair = timestepInteractionPairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason
                    .CertificateInteractionPairInvalid;
        }
        for (int i = 0; i < softAvoidancePairs.Length; i++)
        {
            BodyPair pair = softAvoidancePairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason.CertificateSoftPairInvalid;
        }
        for (int i = 0; i < timestepContactPairs.Length; i++)
        {
            ContactConstraint pair = timestepContactPairs[i];
            if (pair.BodyA < 0 || pair.BodyB <= pair.BodyA ||
                pair.BodyB >= bodyCount)
                return ContactSolverSkipReason.CertificateContactPairInvalid;
        }
        for (int i = 0; i < predictiveContactSchedule.Length; i++)
        {
            ushort scheduledSubstep =
                predictiveContactSchedule[i].Substep;
            if (scheduledSubstep != ushort.MaxValue &&
                scheduledSubstep >= substepCount)
                return ContactSolverSkipReason.CertificateScheduleInvalid;
        }
        return ContactSolverSkipReason.None;
    }

    private static bool HasValidPersistentEntityMapping(
        NativeArray<CrowdBodySnapshot> bodies,
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity)
    {
        if (!currentBodyIndexByEntity.IsCreated)
            return false;
        for (int bodyIndex = 0; bodyIndex < bodies.Length; bodyIndex++)
        {
            if (!currentBodyIndexByEntity.TryGetValue(
                    bodies[bodyIndex].Entity,
                    out int mappedBodyIndex) ||
                mappedBodyIndex != bodyIndex)
                return false;
        }
        return true;
    }



    internal static ContactSolverSkipReason GetConsumerCertificateFailure(
        int substepIndex,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeList<BodyPair> softAvoidancePairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<PredictiveContactScheduleEntry> predictiveContactSchedule,
        NativeReference<InteractionCertificate> interactionCertificate)
    {
        if (!interactionCertificate.IsCreated)
            return ContactSolverSkipReason.CertificateUnavailable;

        InteractionCertificate certificate = interactionCertificate.Value;
        int certificateSubstep = configuration.EnableTimestepContactSetCache
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
                configuration.WorldId,
                configuration.SimulationStepId,
                certificateSubstep))
            return ContactSolverSkipReason.CertificateScopeMismatch;
        if (certificate.BodySetFingerprint != CalculateBodySetFingerprint(bodies))
            return ContactSolverSkipReason.BodySetMismatch;
        if (certificate.ConfigurationFingerprint !=
            configuration.CalculateCertificationFingerprint())
            return ContactSolverSkipReason.ConfigurationMismatch;
        if (certificate.SoftPairCount != softAvoidancePairs.Length)
            return ContactSolverSkipReason.SoftPairCountMismatch;
        if (certificate.ContactConstraintCount != timestepContactPairs.Length)
            return ContactSolverSkipReason.ContactConstraintCountMismatch;
        if (certificate.DormantScheduleCount != predictiveContactSchedule.Length)
            return ContactSolverSkipReason.DormantScheduleCountMismatch;
        return ContactSolverSkipReason.None;
    }




}
}
