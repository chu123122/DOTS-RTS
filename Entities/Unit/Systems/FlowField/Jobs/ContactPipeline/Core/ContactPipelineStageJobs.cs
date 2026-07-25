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
    PreparePersistentClassificationP1P6,
    CommitPersistentClassificationP1P6,
    BuildInitialP1P6,
    FinalizeEnvelopeEscapesP1P6,
    PrepareSubstepRepairP1P6,
    CommitSubstepRepairP1P6,
    FinalizePreparedSubstepP1P6,
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
    public NativeArray<FlowFieldCell> Grid;

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
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeArray<Stage3ContactHeatSample> HeatSamples;
#else
    public Entity DiagnosticSelectedEntity { get => Entity.Null; set { } }
    public NativeList<BodyPair> IncrementalOracleContactPairs { get => default; set { } }
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> Statistics { get => default; set { } }
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics { get => default; set { } }
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics { get => default; set { } }
    public NativeArray<Stage3ContactHeatSample> HeatSamples { get => default; set { } }
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
    private FlowGridGeometry EnvironmentGeometry =>
        new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private ContactPositionSolverMode ContactPositionSolver => Configuration.ContactPositionSolver;

    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = value;
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

public enum MotionIntegrationOperation : byte
{
    None,
    PrepareBaseVelocity,
    PredictUnconstrained,
    ReconstructVelocity
}

[BurstCompile]
public partial struct MotionIntegrationJob : IJob
{
    public MotionIntegrationOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdBodyStepState> StepStates;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
#else
    public NativeReference<PredictiveDiscContactStatistics> Statistics { get => default; set { } }
#endif
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private FlowGridGeometry EnvironmentGeometry => new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);
    public void Execute()
    {
        float dt = Configuration.DeltaTime / math.max(1, Configuration.SubstepCount);
        switch (Operation)
        {
            case MotionIntegrationOperation.PrepareBaseVelocity:
                PrepareBaseVelocitiesForSubstep(dt);
                break;
            case MotionIntegrationOperation.PredictUnconstrained:
                PredictUnconstrainedPositions(dt);
                break;
            case MotionIntegrationOperation.ReconstructVelocity:
#if RTS_CONTACT_DIAGNOSTICS
                PredictiveDiscContactStatistics statistics = Statistics.Value;
                ReconstructVelocities(dt, ref statistics);
                Statistics.Value = statistics;
#else
                PredictiveDiscContactStatistics statistics = default;
                ReconstructVelocities(dt, ref statistics);
#endif
                break;
        }
    }
}

public enum SoftAvoidanceOperation : byte
{
    None,
    SolveSerial,
    PrepareParallelWorkset,
    FinalizeParallel
}

[BurstCompile]
public partial struct SoftAvoidanceJob : IJob
{
    public SoftAvoidanceOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public NativeReference<ParallelJacobiExecutionState> RuntimeState;
    public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdBodyStepState> StepStates;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeArray<int> SoftIncidentOffsets;
    public NativeArray<int> SoftIncidentWriteCursors;
    public NativeList<int> SoftIncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> SoftPairContributions;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeArray<int> EscapeCountsByBlock;
    public int EscapeBlockCount;
#else
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> Statistics { get => default; set { } }
#endif
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SoftAvoidanceShell => Configuration.SoftAvoidanceShell;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver => Configuration.SoftAvoidanceVelocitySolver;
    private float RvoTimeHorizon => Configuration.RvoTimeHorizon;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private FlowGridGeometry EnvironmentGeometry => new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);
    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = value;
#endif
    }
    public void Execute()
    {
        switch (Operation)
        {
            case SoftAvoidanceOperation.SolveSerial:
            {
                float dt = Configuration.DeltaTime / math.max(1, Configuration.SubstepCount);
#if RTS_CONTACT_DIAGNOSTICS
                PredictiveDiscContactStatistics statistics = Statistics.Value;
                IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
#else
                PredictiveDiscContactStatistics statistics = default;
                IncrementalContactPipelineStatistics incremental = default;
#endif
                long start = ProfilerUnsafeUtility.Timestamp;
                CalculateSoftAvoidanceForSubstep(dt, ref statistics, ref incremental);
                statistics.SoftAvoidanceEvaluationCount++;
                statistics.SoftAvoidanceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp - start);
#if RTS_CONTACT_DIAGNOSTICS
                Statistics.Value = statistics;
                IncrementalStatistics.Value = incremental;
#endif
                break;
            }
            case SoftAvoidanceOperation.PrepareParallelWorkset:
                PrepareP1P6SoftWorkset(RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , BlockStatistics
#endif
                );
                break;
            case SoftAvoidanceOperation.FinalizeParallel:
#if RTS_CONTACT_DIAGNOSTICS
                FinalizeP1P6SoftAvoidance(RuntimeState, BlockStatistics, EscapeCountsByBlock, EscapeBlockCount);
#endif
                break;
        }
    }
}

public enum ConstraintSolverOperation : byte
{
    None,
    SolveWallSerial,
    SolveContactSerial,
    SolveRecoverySerial,
    SolveParallelRecovery,
    FinalizeSerialSubstep,
    FinalizeSerialPipeline,
    ResetAndBuildIncidentSerial,
    BeginParallelIteration,
    BeginParallelFinalizeSubstep,
    FinalizeParallelVelocity,
    MergeParallelDebuggerPairs,
    FinalizeParallelPipeline
}

[BurstCompile]
public partial struct ConstraintSolverJob : IJob
{
    public ConstraintSolverOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public NativeReference<SerialContactPipelineControlState> SerialControl;
    public int SubstepIndex;
    public int IterationIndex;
    public int BodyBlockCount;
    public int BlockCount;
    public NativeReference<ParallelJacobiExecutionState> RuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
#endif
    public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
#if RTS_CONTACT_DIAGNOSTICS
    public Entity DiagnosticSelectedEntity;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;
    public NativeArray<Stage3ContactHeatSample> HeatSamples;
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask;
    public int SimulationDebuggerMaximumPairs;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
    public NativeList<SimulationDebuggerPairSample> ParallelSimulationDebuggerPairScratch;
    public NativeReference<SimulationDebuggerUnitSample> SimulationDebuggerSelectedUnit;
    public NativeReference<byte> SimulationDebuggerSelectedUnitValid;
#else
    public Entity DiagnosticSelectedEntity { get => Entity.Null; set { } }
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> Statistics { get => default; set { } }
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics { get => default; set { } }
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics { get => default; set { } }
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic { get => default; set { } }
    public NativeArray<Stage3ContactHeatSample> HeatSamples { get => default; set { } }
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask { get => default; set { } }
    public int SimulationDebuggerMaximumPairs { get => 0; set { } }
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs { get => default; set { } }
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates { get => default; set { } }
    public NativeList<SimulationDebuggerPairSample> ParallelSimulationDebuggerPairScratch { get => default; set { } }
    public NativeReference<SimulationDebuggerUnitSample> SimulationDebuggerSelectedUnit { get => default; set { } }
    public NativeReference<byte> SimulationDebuggerSelectedUnitValid { get => default; set { } }
#endif
    private float DeltaTime => Configuration.DeltaTime;
    private int SubstepCount => Configuration.SubstepCount;
    private int IterationCount => Configuration.IterationCount;
    private ContactPositionSolverMode ContactPositionSolver => Configuration.ContactPositionSolver;
    private float Compliance => Configuration.Compliance;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    private bool EnableTimestepContactSetCache => Configuration.EnableTimestepContactSetCache;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private bool EnablePredictiveContacts => Configuration.EnablePredictiveContacts;
    private float PredictiveSkin => Configuration.PredictiveSkin;
    private FlowGridGeometry EnvironmentGeometry => new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);
    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = value;
#endif
    }
    public void Execute()
    {
        switch (Operation)
        {
            case ConstraintSolverOperation.SolveWallSerial:
                ExecuteSolveWallSerial();
                break;
            case ConstraintSolverOperation.SolveContactSerial:
                ExecuteSolveContactSerial(false);
                break;
            case ConstraintSolverOperation.SolveRecoverySerial:
                ExecuteSolveContactSerial(true);
                break;
            case ConstraintSolverOperation.SolveParallelRecovery:
                ExecuteParallelRecovery();
                break;
            case ConstraintSolverOperation.FinalizeSerialSubstep:
                ExecuteFinalizeSerialSubstep();
                break;
            case ConstraintSolverOperation.FinalizeSerialPipeline:
                ExecuteFinalizeSerialPipeline();
                break;
            case ConstraintSolverOperation.ResetAndBuildIncidentSerial:
                ResetTimestepContactSetForSubstep();
                RebuildActiveConstraintIncidentIndexIfNeeded();
                break;
            case ConstraintSolverOperation.BeginParallelIteration:
                BeginP1P6Iteration(SubstepIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState
#endif
                );
                break;
            case ConstraintSolverOperation.BeginParallelFinalizeSubstep:
#if RTS_CONTACT_DIAGNOSTICS
                BeginP1P6FinalizeSubstep(RuntimeState);
#endif
                break;
            case ConstraintSolverOperation.FinalizeParallelVelocity:
#if RTS_CONTACT_DIAGNOSTICS
                FinalizeP1P6VelocityStatistics(RuntimeState, BlockCount);
#endif
                break;
            case ConstraintSolverOperation.MergeParallelDebuggerPairs:
#if RTS_CONTACT_DIAGNOSTICS
                MergeParallelSimulationDebuggerPairScratch();
#endif
                break;
            case ConstraintSolverOperation.FinalizeParallelPipeline:
                FinalizeParallelJacobiPipeline(RuntimeState);
                break;
        }
    }
}
}
