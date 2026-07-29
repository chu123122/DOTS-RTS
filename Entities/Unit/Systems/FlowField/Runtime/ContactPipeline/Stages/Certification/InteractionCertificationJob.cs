using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public enum InteractionCertificationOperation : byte
{
    None,
    PreparePersistentClassification,
    CommitPersistentClassification,
    BuildInitial,
    FinalizeEnvelopeEscapes,
    PrepareSubstepRepair,
    CommitSubstepRepair,
    FinalizePreparedSubstep,
    ValidateConsumerViews,
    FinalizeWallIteration,
    FinalizeContactIteration
}

[BurstCompile]
public partial struct InteractionCertificationJob : IJob
{
    public InteractionCertificationOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public int SubstepIndex;
    public int IterationIndex;
    public byte IsLastIteration;
    public byte AfterContact;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int BodyBlockCount;

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
    // O(1) 持久接触查找索引。与 PersistentPredictiveContacts 同步：
    // 每次全量重建后从列表回填；增量 patch 路径就地更新，无需重排序。
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

    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState;
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;


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

    public void Execute()
    {
        switch (Operation)
        {
            case InteractionCertificationOperation.PreparePersistentClassification:
                PreparePersistentClassification(RuntimeState);
                break;
            case InteractionCertificationOperation.CommitPersistentClassification:
                CommitPersistentClassification(RuntimeState);
                break;
            case InteractionCertificationOperation.BuildInitial:
                BuildInitialContactSet(RuntimeState);
                break;
            case InteractionCertificationOperation.FinalizeEnvelopeEscapes:
                FinalizeEnvelopeEscapes(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.PrepareSubstepRepair:
                PrepareSubstepRepairClassification(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.CommitSubstepRepair:
                CommitSubstepRepairClassification(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.FinalizePreparedSubstep:
                FinalizePreparedSubstep(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.ValidateConsumerViews:
                ValidateConsumerViews();
                break;
            case InteractionCertificationOperation.FinalizeWallIteration:
                FinalizeWallIteration(SubstepIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState, BlockStatistics
#endif
                    , BodyBlockCount);
                break;
            case InteractionCertificationOperation.FinalizeContactIteration:
                FinalizeContactIteration(
                    SubstepIndex,
                    IterationIndex,
                    RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState, BlockStatistics
#endif
                );
                break;
        }
    }

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
