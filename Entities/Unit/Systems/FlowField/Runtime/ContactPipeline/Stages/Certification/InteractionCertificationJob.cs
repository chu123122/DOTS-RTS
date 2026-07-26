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
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
#endif

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
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
#endif
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;

#if RTS_CONTACT_DIAGNOSTICS
    public Entity DiagnosticSelectedEntity;
    public NativeList<BodyPair> IncrementalOracleContactPairs;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeArray<ContactHeatSample> HeatSamples;
#endif

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

    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? IncrementalStatistics.Value : default;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? Statistics.Value : default;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) Statistics.Value = value;
#endif
    }

    public void Execute()
    {
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
    }
}
}
