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
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
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
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<SelectedBodyContactDiagnostic> SelectedBodyDiagnostic;
    public NativeArray<ContactHeatSample> HeatSamples;
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask;
    public int SimulationDebuggerMaximumPairs;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
    public NativeList<SimulationDebuggerPairSample> ParallelSimulationDebuggerPairScratch;
    public NativeReference<SimulationDebuggerUnitSample> SimulationDebuggerSelectedUnit;
    public NativeReference<byte> SimulationDebuggerSelectedUnitValid;
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
