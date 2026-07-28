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
public struct SerialContactPipelineControlState
{
    public byte IsValid;
    public byte RecoveryRequired;
    public float PenetrationSum;
    public long SolverStartTimestamp;
    public long IterationStartTimestamp;
    public long IterationAccountedStartNanoseconds;
    public float MaxViolationBeforeSolve;
    public float AverageViolationBeforeSolve;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
    public float TotalContactPositionCorrection;
    public float MaxContactPositionCorrection;
}

public enum InteractionCertificationOperation : byte
{
    None,
    InitializeSerial,
    BuildInitialSerial,
    BuildSubstepInteractionSerial,
    ValidateBaseMotionSerial,
    ClampSoftOutputSerial,
    ValidatePredictedAndActivateSerial,
    ValidateSolverCorrectionSerial,
    ValidateConsumerViewsSerial,
    PreparePersistentClassificationP1P6,
    CommitPersistentClassificationP1P6,
    BuildInitialP1P6,
    FinalizeEnvelopeEscapesP1P6,
    PrepareSubstepRepairP1P6,
    CommitSubstepRepairP1P6,
    FinalizePreparedSubstepP1P6,
    ValidateConsumerViewsP1P6,
    FinalizeWallIterationP1P6,
    FinalizeContactIterationP1P6
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
    public NativeReference<ParallelJacobiExecutionState> RuntimeState;
    public NativeReference<SerialContactPipelineControlState> SerialControl;
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
#if RTS_CONTACT_DIAGNOSTICS
        bool measureSerialValidation = IsSerialValidationTimingOperation(Operation);
        long timingStart = measureSerialValidation
            ? ProfilerUnsafeUtility.Timestamp
            : 0L;
        long accountedStart = measureSerialValidation
            ? AccountedCandidateNanoseconds(LoadIncrementalStatistics())
            : 0L;
#endif
        switch (Operation)
        {
            case InteractionCertificationOperation.InitializeSerial:
                ExecuteInitializeSerial();
                break;
            case InteractionCertificationOperation.BuildInitialSerial:
                ExecuteBuildInitialSerial();
                break;
            case InteractionCertificationOperation.BuildSubstepInteractionSerial:
                ExecuteBuildSubstepInteractionSerial();
                break;
            case InteractionCertificationOperation.ValidateBaseMotionSerial:
                ExecuteValidateBaseMotionSerial();
                break;
            case InteractionCertificationOperation.ClampSoftOutputSerial:
                ExecuteClampSoftOutputSerial();
                break;
            case InteractionCertificationOperation.ValidatePredictedAndActivateSerial:
                ExecuteValidatePredictedAndActivateSerial();
                break;
            case InteractionCertificationOperation.ValidateSolverCorrectionSerial:
                ExecuteValidateSolverCorrectionSerial();
                break;
            case InteractionCertificationOperation.ValidateConsumerViewsSerial:
                ValidateConsumerViewsSerial();
                break;
            case InteractionCertificationOperation.PreparePersistentClassificationP1P6:
                PreparePersistentClassificationP1P6(RuntimeState);
                break;
            case InteractionCertificationOperation.CommitPersistentClassificationP1P6:
                CommitPersistentClassificationP1P6(RuntimeState);
                break;
            case InteractionCertificationOperation.BuildInitialP1P6:
                BuildInitialP1P6ContactSet(RuntimeState);
                break;
            case InteractionCertificationOperation.FinalizeEnvelopeEscapesP1P6:
                FinalizeP1P6EnvelopeEscapes(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.PrepareSubstepRepairP1P6:
                PrepareP1P6SubstepRepairClassification(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.CommitSubstepRepairP1P6:
                CommitP1P6SubstepRepairClassification(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.FinalizePreparedSubstepP1P6:
                FinalizeP1P6PreparedSubstep(SubstepIndex, RuntimeState);
                break;
            case InteractionCertificationOperation.ValidateConsumerViewsP1P6:
                ValidateConsumerViewsP1P6();
                break;
            case InteractionCertificationOperation.FinalizeWallIterationP1P6:
                FinalizeP1P6WallIteration(SubstepIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState, BlockStatistics
#endif
                    , BodyBlockCount);
                break;
            case InteractionCertificationOperation.FinalizeContactIterationP1P6:
                FinalizeParallelJacobiIteration(
                    SubstepIndex,
                    IterationIndex,
                    RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState, BlockStatistics
#endif
                );
                break;
        }
#if RTS_CONTACT_DIAGNOSTICS
        if (measureSerialValidation)
        {
            long elapsed = ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - timingStart);
            long nestedCandidateNanoseconds =
                AccountedCandidateNanoseconds(LoadIncrementalStatistics()) -
                accountedStart;
            PredictiveDiscContactStatistics statistics =
                LoadContactStatistics();
            statistics.ValidationRepairNanoseconds += math.max(
                0L,
                elapsed - math.max(0L, nestedCandidateNanoseconds));
            StoreContactStatistics(statistics);
        }
#endif
    }

#if RTS_CONTACT_DIAGNOSTICS
    private static bool IsSerialValidationTimingOperation(
        InteractionCertificationOperation operation) =>
        operation == InteractionCertificationOperation.InitializeSerial ||
        operation == InteractionCertificationOperation.BuildInitialSerial ||
        operation == InteractionCertificationOperation.BuildSubstepInteractionSerial ||
        operation == InteractionCertificationOperation.ValidateBaseMotionSerial ||
        operation == InteractionCertificationOperation.ClampSoftOutputSerial ||
        operation == InteractionCertificationOperation.ValidatePredictedAndActivateSerial;

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
