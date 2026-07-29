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
/// 认证算法与资源切片的非调度门面。具体 IJob 位于 InteractionCertificationStageJobs.cs。
/// 本类型不实现 IJob，也不通过 operation switch 携带全部阶段参数。
/// </summary>
public partial struct InteractionCertificationAlgorithms
{
    public CertificationEnvironmentResources Environment;
    public CertificationBodyResources Body;
    public CertificationViewResources Views;
    public PersistentCertificationResources Persistent;
    public CertificationSolverResources Solver;

    public ContactPipelineConfiguration Configuration => Environment.Configuration;
    public float3 GridOrigin => Environment.GridOrigin;
    public int2 GridDimensions => Environment.GridDimensions;
    public float CellRadius => Environment.CellRadius;
    public NativeArray<FlowFieldCell> Grid => Environment.Grid;

    public NativeArray<CrowdBodySnapshot> Bodies => Body.Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates => Body.NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents => Body.MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence => Body.MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates => Body.StepStates;

    public NativeList<SweptDiscCellEntry> SweptCellEntries => Views.SweptCellEntries;
    public NativeList<ContactConstraint> Pairs => Views.Pairs;
    public NativeList<ContactConstraint> TimestepContactPairs => Views.TimestepContactPairs;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs => Views.PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs => Views.TimestepInteractionPairs;
    public NativeList<BodyPair> SoftAvoidancePairs => Views.SoftAvoidancePairs;
    public NativeList<BodyPair> ClassificationBodyPairs => Views.ClassificationBodyPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity => Views.CurrentBodyIndexByEntity;

    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies => Persistent.CurrentIncrementalProxies;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies => Persistent.PersistentSweptProxies;
    public NativeList<int> PersistentProxyIndexByBody => Persistent.PersistentProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentNeighborPairs => Persistent.PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentPredictiveContacts => Persistent.PersistentPredictiveContacts;
    public NativeHashMap<StableEntityPairKey, PersistentPredictiveContact> PersistentContactIndex => Persistent.PersistentContactIndex;
    public NativeList<StableEntityPairKey> PersistentActiveContactKeys => Persistent.PersistentActiveContactKeys;
    public NativeList<StableEntityPairKey> PersistentSoftAvoidancePairKeys => Persistent.PersistentSoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> PersistentDormantContactSchedule => Persistent.PersistentDormantContactSchedule;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch => Persistent.PredictiveContactScratch;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies => Persistent.IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody => Persistent.IncrementalDirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch => Persistent.IncrementalNeighborPairScratch;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule => Persistent.PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch => Persistent.PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor => Persistent.PredictiveContactScheduleCursor;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState => Persistent.IncrementalCacheState;
    public NativeReference<InteractionCertificate> InteractionCertificate => Persistent.InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionCertificateViolations => Persistent.InteractionCertificateViolations;
    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults => Persistent.PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState => Persistent.PersistentClassificationState;
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership => Persistent.PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch => Persistent.PersistentSpatialMembershipEpoch;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy => Persistent.PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp => Persistent.PersistentSpatialVisitStamp;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup => Persistent.PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch => Persistent.PersistentIncidentLookupEpoch;

    public NativeArray<byte> CorrectedBodyFlags => Solver.CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices => Solver.CorrectedBodyIndices;
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics => Solver.ParallelBodyStatistics;
    public NativeArray<byte> EnvelopeEscapeFlags => Solver.EnvelopeEscapeFlags;
    public NativeArray<int> DirtyBodyBlockOffsets => Solver.DirtyBodyBlockOffsets;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState => Solver.ActiveIncidentIndexState;
    public NativeArray<int> ActiveIncidentOffsets => Solver.ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors => Solver.ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices => Solver.ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections => Solver.JacobiPairCorrections;

    private float DeltaTime => Configuration.DeltaTime;
    private int SubstepCount => Configuration.SubstepCount;
    private int IterationCount => Configuration.IterationCount;
    private float PredictiveSkin => Configuration.PredictiveSkin;
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SoftAvoidanceShell => Configuration.SoftAvoidanceShell;
    private SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver => Configuration.SoftAvoidanceVelocitySolver;
    private float RvoTimeHorizon => Configuration.RvoTimeHorizon;
    private bool EnablePredictivePairGeneration => Configuration.EnablePredictivePairGeneration;
    private bool EnablePredictiveContacts => Configuration.EnablePredictiveContacts;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private bool EnableTimestepContactSetCache => Configuration.EnableTimestepContactSetCache;
    private float GuardEnvelopeMargin => Configuration.GuardEnvelopeMargin;
    private float TimestepContactMargin => Configuration.TimestepContactMargin;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private ContactPositionSolverMode ContactPositionSolver => Configuration.ContactPositionSolver;

    private FlowGridGeometry EnvironmentGeometry =>
        new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);

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
