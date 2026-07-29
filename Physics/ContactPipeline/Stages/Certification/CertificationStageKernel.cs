using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct CertificationEnvironmentResources
{
    public ContactPipelineConfiguration Configuration;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
}

public struct CertificationBodyResources
{
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates;
}

public struct CertificationViewResources
{
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeArray<int> BodyCellCounts;
    public NativeArray<int> BodyCellOffsets;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> Pairs;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<BodyPair> ClassificationBodyPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;
}

public struct PersistentCertificationResources
{
    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies;
    public NativeList<int> PersistentProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentPredictiveContacts;
    public NativeHashMap<StableEntityPairKey, PersistentPredictiveContact> PersistentContactIndex;
    public NativeList<StableEntityPairKey> PersistentActiveContactKeys;
    public NativeList<StableEntityPairKey> PersistentSoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> PersistentDormantContactSchedule;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionCertificateViolations;
    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState;
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;
    public NativeArray<DirtyBodyRefreshResult> DirtyBodyRefreshResults;
    public NativeReference<DirtyBodyRefreshSummary> DirtyBodyRefreshSummary;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockCounts;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockOffsets;
}

public struct CertificationSolverResources
{
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
    public NativeArray<byte> EnvelopeEscapeFlags;
    public NativeArray<int> DirtyBodyBlockOffsets;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
}

/// <summary>
/// 只供明确 Stage Job 调用的算法内核。字段是实际 NativeContainer，
/// 不再通过聚合门面转发资源，也不参与调度。
/// </summary>
internal partial struct CertificationStageKernel
{
    public ContactPipelineConfiguration Configuration;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;

    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates;

    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeArray<int> BodyCellCounts;
    public NativeArray<int> BodyCellOffsets;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> Pairs;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<BodyPair> ClassificationBodyPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;

    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies;
    public NativeList<int> PersistentProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentPredictiveContacts;
    public NativeHashMap<StableEntityPairKey, PersistentPredictiveContact> PersistentContactIndex;
    public NativeList<StableEntityPairKey> PersistentActiveContactKeys;
    public NativeList<StableEntityPairKey> PersistentSoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> PersistentDormantContactSchedule;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionCertificateViolations;
    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState;
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;
    public NativeArray<DirtyBodyRefreshResult> DirtyBodyRefreshResults;
    public NativeReference<DirtyBodyRefreshSummary> DirtyBodyRefreshSummary;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockCounts;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockOffsets;

    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
    public NativeArray<byte> EnvelopeEscapeFlags;
    public NativeArray<int> DirtyBodyBlockOffsets;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;

#if RTS_CONTACT_DIAGNOSTICS
    private static long AccountedCandidateNanoseconds(
        IncrementalContactPipelineStatistics statistics) =>
        statistics.ProxyValidationNanoseconds +
        statistics.FullSweepSourceNanoseconds +
        statistics.PersistentPairMappingNanoseconds +
        statistics.LocalBroadPhaseNanoseconds +
        statistics.PairDiffNanoseconds +
        statistics.FallbackNanoseconds +
        statistics.SweptClassificationNanoseconds +
        statistics.ContactActivationNanoseconds;
#endif
}
}
