using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// Cross-timestep candidate contact state owned by one movement-system World.
/// Only the interaction certifier may interpret or mutate these containers.
/// They are never authoritative lower-stage inputs until certified frame views
/// have been committed.
/// </summary>
internal struct ContactPersistentState
{
    public NativeList<PersistentSweptProxy> SweptProxies;
    public NativeList<int> ProxyIndexByBody;
    public NativeList<PersistentNeighborPair> NeighborPairs;
    public NativeList<PersistentPredictiveContact> PredictiveContacts;
    public NativeList<StableEntityPairKey> ActiveContactKeys;
    public NativeList<StableEntityPairKey> SoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> DormantContactSchedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    public NativeParallelMultiHashMap<Entity, int> IncidentPairLookup;
    public NativeReference<uint> IncidentLookupEpoch;
    public NativeParallelMultiHashMap<int, int> SpatialMembership;
    public NativeReference<uint> SpatialMembershipEpoch;

    public static ContactPersistentState Create()
    {
        return new ContactPersistentState
        {
            SweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent),
            ProxyIndexByBody = new NativeList<int>(Allocator.Persistent),
            NeighborPairs = new NativeList<PersistentNeighborPair>(Allocator.Persistent),
            PredictiveContacts = new NativeList<PersistentPredictiveContact>(Allocator.Persistent),
            ActiveContactKeys = new NativeList<StableEntityPairKey>(Allocator.Persistent),
            SoftAvoidancePairKeys = new NativeList<StableEntityPairKey>(Allocator.Persistent),
            DormantContactSchedule = new NativeList<PredictiveContactScheduleEntry>(Allocator.Persistent),
            CacheState = new NativeReference<IncrementalContactCacheState>(Allocator.Persistent),
            IncidentPairLookup = new NativeParallelMultiHashMap<Entity, int>(1, Allocator.Persistent),
            IncidentLookupEpoch = new NativeReference<uint>(Allocator.Persistent),
            SpatialMembership = new NativeParallelMultiHashMap<int, int>(1, Allocator.Persistent),
            SpatialMembershipEpoch = new NativeReference<uint>(Allocator.Persistent)
        };
    }

    public bool RequiresCapacity(int unitCount)
    {
        int incidentRequired = math.max(1, unitCount * 64);
        int spatialRequired = math.max(1, unitCount * 128);
        return ProxyIndexByBody.Capacity < unitCount ||
               IncidentPairLookup.Capacity < incidentRequired ||
               SpatialMembership.Capacity < spatialRequired;
    }

    public void EnsureCapacity(int unitCount)
    {
        if (ProxyIndexByBody.Capacity < unitCount)
            ProxyIndexByBody.Capacity = unitCount;
        int incidentRequired = math.max(1, unitCount * 64);
        int spatialRequired = math.max(1, unitCount * 128);
        if (IncidentPairLookup.Capacity < incidentRequired)
            IncidentPairLookup.Capacity = incidentRequired;
        if (SpatialMembership.Capacity < spatialRequired)
            SpatialMembership.Capacity = spatialRequired;
    }

    public void Reset()
    {
        SweptProxies.Clear();
        ProxyIndexByBody.Clear();
        NeighborPairs.Clear();
        PredictiveContacts.Clear();
        ActiveContactKeys.Clear();
        SoftAvoidancePairKeys.Clear();
        DormantContactSchedule.Clear();
        CacheState.Value = default;
        IncidentPairLookup.Clear();
        IncidentLookupEpoch.Value = 0;
        SpatialMembership.Clear();
        SpatialMembershipEpoch.Value = 0;
    }

    public void Dispose()
    {
        if (SweptProxies.IsCreated) SweptProxies.Dispose();
        if (ProxyIndexByBody.IsCreated) ProxyIndexByBody.Dispose();
        if (NeighborPairs.IsCreated) NeighborPairs.Dispose();
        if (PredictiveContacts.IsCreated) PredictiveContacts.Dispose();
        if (ActiveContactKeys.IsCreated) ActiveContactKeys.Dispose();
        if (SoftAvoidancePairKeys.IsCreated) SoftAvoidancePairKeys.Dispose();
        if (DormantContactSchedule.IsCreated) DormantContactSchedule.Dispose();
        if (CacheState.IsCreated) CacheState.Dispose();
        if (IncidentPairLookup.IsCreated) IncidentPairLookup.Dispose();
        if (IncidentLookupEpoch.IsCreated) IncidentLookupEpoch.Dispose();
        if (SpatialMembership.IsCreated) SpatialMembership.Dispose();
        if (SpatialMembershipEpoch.IsCreated) SpatialMembershipEpoch.Dispose();
    }
}

/// <summary>
/// Runtime-only containers for one scheduled contact timestep. Creation and
/// disposal are centralized here; diagnostics resources remain in the separate
/// ContactDiagnosticsFrameResources owner.
/// </summary>
internal struct ContactFrameResources
{
    public NativeArray<FlowMovementFrameState> States;
    public NativeArray<float2> CollisionFootprints;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<UnitCollisionPair> CollisionPairs;
    public NativeList<UnitCollisionPair> TimestepContactPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;
    public NativeList<UnitCollisionPair> TimestepInteractionPairs;
    public NativeList<UnitCollisionPair> SoftAvoidancePairs;
    public NativeList<UnitCollisionPair> PreviousTimestepContactPairs;
    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor;

    // The compact consumer views above are authoritative only while this
    // certificate remains issued for their exact step/substep scope.
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionViolations;

    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
    public NativeArray<byte> EnvelopeEscapeFlags;
    public NativeArray<ParallelBodyStageResult> ParallelBodyResults;
    public NativeArray<int> DirtyBodyBlockOffsets;
    public NativeArray<int> SoftIncidentOffsets;
    public NativeArray<int> SoftIncidentWriteCursors;
    public NativeList<int> SoftIncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> SoftPairContributions;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeReference<ParallelJacobiExecutionState> ParallelJacobiRuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
    public NativeReference<ParallelJacobiIterationTelemetry> ParallelJacobiIterationState;
    public NativeList<JacobiBlockTelemetry> ParallelJacobiBlockTelemetry;
#endif

    public static ContactFrameResources Create(
        int unitCount,
        bool usesJacobiScratch,
        bool useParallelJacobi)
    {
        int one = math.max(unitCount, 1);
        return new ContactFrameResources
        {
            States = new NativeArray<FlowMovementFrameState>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            CollisionFootprints = new NativeArray<float2>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            SweptCellEntries = new NativeList<SweptDiscCellEntry>(math.max(unitCount * 4, 1), Allocator.TempJob),
            CollisionPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 4, 1), Allocator.TempJob),
            TimestepContactPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 4, 1), Allocator.TempJob),
            CurrentBodyIndexByEntity = new NativeParallelHashMap<Entity, int>(one, Allocator.TempJob),
            TimestepInteractionPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            SoftAvoidancePairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 4, 1), Allocator.TempJob),
            PreviousTimestepContactPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            CurrentIncrementalProxies = new NativeList<PersistentSweptProxy>(one, Allocator.TempJob),
            IncrementalDirtyBodies = new NativeList<IncrementalDirtyBody>(one, Allocator.TempJob),
            IncrementalDirtyFlagsByBody = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            PredictiveContactScratch = new NativeList<PersistentPredictiveContact>(math.max(unitCount * 4, 1), Allocator.TempJob),
            IncrementalNeighborPairScratch = new NativeList<PersistentNeighborPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            PredictiveContactSchedule = new NativeList<PredictiveContactScheduleEntry>(math.max(unitCount * 2, 1), Allocator.TempJob),
            PredictiveContactScheduleScratch = new NativeList<PredictiveContactScheduleEntry>(one, Allocator.TempJob),
            PredictiveContactScheduleCursor = new NativeReference<int>(Allocator.TempJob),
            InteractionCertificate = new NativeReference<InteractionCertificate>(Allocator.TempJob),
            InteractionViolations = new NativeList<InteractionCertificateViolation>(one, Allocator.TempJob),
            CorrectedBodyFlags = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            CorrectedBodyIndices = new NativeList<int>(one, Allocator.TempJob),
            ActiveIncidentOffsets = usesJacobiScratch ? new NativeArray<int>(unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            ActiveIncidentWriteCursors = usesJacobiScratch ? new NativeArray<int>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            ActiveIncidentPairIndices = usesJacobiScratch ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob) : default,
            JacobiPairCorrections = usesJacobiScratch ? new NativeList<JacobiPairCorrection>(math.max(unitCount * 4, 1), Allocator.TempJob) : default,
            EnvelopeEscapeFlags = useParallelJacobi ? new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            ParallelBodyResults = useParallelJacobi ? new NativeArray<ParallelBodyStageResult>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            DirtyBodyBlockOffsets = useParallelJacobi ? new NativeArray<int>(one, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            SoftIncidentOffsets = useParallelJacobi ? new NativeArray<int>(unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            SoftIncidentWriteCursors = useParallelJacobi ? new NativeArray<int>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory) : default,
            SoftIncidentPairIndices = useParallelJacobi ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob) : default,
            SoftPairContributions = useParallelJacobi ? new NativeList<SoftAvoidancePairContribution>(math.max(unitCount * 4, 1), Allocator.TempJob) : default,
            ActiveIncidentIndexState = usesJacobiScratch ? new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob) : default,
            PersistentClassificationResults = useParallelJacobi ? new NativeList<PersistentPairClassificationResult>(math.max(unitCount * 8, 1), Allocator.TempJob) : default,
            PersistentClassificationState = useParallelJacobi ? new NativeReference<PersistentClassificationPhaseState>(Allocator.TempJob) : default,
            PersistentSpatialVisitStampByProxy = new NativeArray<uint>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            PersistentSpatialVisitStamp = new NativeReference<uint>(Allocator.TempJob),
            ParallelJacobiRuntimeState = useParallelJacobi ? new NativeReference<ParallelJacobiExecutionState>(Allocator.TempJob) : default,
#if RTS_CONTACT_DIAGNOSTICS
            PersistentClassificationTelemetry = useParallelJacobi
                ? new NativeReference<PersistentClassificationTelemetryState>(Allocator.TempJob)
                : default,
            ParallelJacobiIterationState = useParallelJacobi ? new NativeReference<ParallelJacobiIterationTelemetry>(Allocator.TempJob) : default,
            ParallelJacobiBlockTelemetry = useParallelJacobi ? new NativeList<JacobiBlockTelemetry>(math.max((unitCount * 4 + 63) / 64, 1), Allocator.TempJob) : default,
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = Combine(combined, States.Dispose(finalReader));
        combined = Combine(combined, CollisionFootprints.Dispose(finalReader));
        combined = Combine(combined, SweptCellEntries.Dispose(finalReader));
        combined = Combine(combined, CollisionPairs.Dispose(finalReader));
        combined = Combine(combined, TimestepContactPairs.Dispose(finalReader));
        combined = Combine(combined, CurrentBodyIndexByEntity.Dispose(finalReader));
        combined = Combine(combined, TimestepInteractionPairs.Dispose(finalReader));
        combined = Combine(combined, SoftAvoidancePairs.Dispose(finalReader));
        combined = Combine(combined, PreviousTimestepContactPairs.Dispose(finalReader));
        combined = Combine(combined, CurrentIncrementalProxies.Dispose(finalReader));
        combined = Combine(combined, IncrementalDirtyBodies.Dispose(finalReader));
        combined = Combine(combined, IncrementalDirtyFlagsByBody.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScratch.Dispose(finalReader));
        combined = Combine(combined, IncrementalNeighborPairScratch.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactSchedule.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScheduleScratch.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScheduleCursor.Dispose(finalReader));
        combined = Combine(combined, InteractionCertificate.Dispose(finalReader));
        combined = Combine(combined, InteractionViolations.Dispose(finalReader));
        combined = Combine(combined, CorrectedBodyFlags.Dispose(finalReader));
        combined = Combine(combined, CorrectedBodyIndices.Dispose(finalReader));
        if (ActiveIncidentOffsets.IsCreated) combined = Combine(combined, ActiveIncidentOffsets.Dispose(finalReader));
        if (ActiveIncidentWriteCursors.IsCreated) combined = Combine(combined, ActiveIncidentWriteCursors.Dispose(finalReader));
        if (ActiveIncidentPairIndices.IsCreated) combined = Combine(combined, ActiveIncidentPairIndices.Dispose(finalReader));
        if (JacobiPairCorrections.IsCreated) combined = Combine(combined, JacobiPairCorrections.Dispose(finalReader));
        if (EnvelopeEscapeFlags.IsCreated) combined = Combine(combined, EnvelopeEscapeFlags.Dispose(finalReader));
        if (ParallelBodyResults.IsCreated) combined = Combine(combined, ParallelBodyResults.Dispose(finalReader));
        if (DirtyBodyBlockOffsets.IsCreated) combined = Combine(combined, DirtyBodyBlockOffsets.Dispose(finalReader));
        if (SoftIncidentOffsets.IsCreated) combined = Combine(combined, SoftIncidentOffsets.Dispose(finalReader));
        if (SoftIncidentWriteCursors.IsCreated) combined = Combine(combined, SoftIncidentWriteCursors.Dispose(finalReader));
        if (SoftIncidentPairIndices.IsCreated) combined = Combine(combined, SoftIncidentPairIndices.Dispose(finalReader));
        if (SoftPairContributions.IsCreated) combined = Combine(combined, SoftPairContributions.Dispose(finalReader));
        if (ActiveIncidentIndexState.IsCreated) combined = Combine(combined, ActiveIncidentIndexState.Dispose(finalReader));
        if (PersistentClassificationResults.IsCreated) combined = Combine(combined, PersistentClassificationResults.Dispose(finalReader));
        if (PersistentClassificationState.IsCreated) combined = Combine(combined, PersistentClassificationState.Dispose(finalReader));
        combined = Combine(combined, PersistentSpatialVisitStampByProxy.Dispose(finalReader));
        combined = Combine(combined, PersistentSpatialVisitStamp.Dispose(finalReader));
        if (ParallelJacobiRuntimeState.IsCreated) combined = Combine(combined, ParallelJacobiRuntimeState.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        if (PersistentClassificationTelemetry.IsCreated) combined = Combine(combined, PersistentClassificationTelemetry.Dispose(finalReader));
        if (ParallelJacobiIterationState.IsCreated) combined = Combine(combined, ParallelJacobiIterationState.Dispose(finalReader));
        if (ParallelJacobiBlockTelemetry.IsCreated) combined = Combine(combined, ParallelJacobiBlockTelemetry.Dispose(finalReader));
#endif
        return combined;
    }

    private static JobHandle Combine(JobHandle a, JobHandle b) =>
        JobHandle.CombineDependencies(a, b);
}
}
