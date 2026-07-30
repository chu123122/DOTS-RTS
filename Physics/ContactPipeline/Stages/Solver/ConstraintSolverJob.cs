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
    SolveGaussSeidelContact,
    SolveGaussSeidelRecovery,
    PrepareJacobiRecovery,
    FinalizeJacobiRecovery,
    InitializeContactIteration,
    FinalizeSubstepTelemetry,
    FinalizeVelocity,
    MergeParallelDebuggerPairs,
    FinalizePipeline
}

[BurstCompile]
public partial struct ConstraintSolverJob : IJob
{
    public ConstraintSolverOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public int SubstepIndex;
    public int IterationIndex;
    public int BodyBlockCount;
    public int BlockCount;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    [ReadOnly] public NativeArray<CrowdObstacleCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdAvoidanceState> AvoidanceStates;
    public NativeArray<CrowdSolverBodyState> StepStates;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
    public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
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
    public void Execute()
    {
        switch (Operation)
        {
            case ConstraintSolverOperation.SolveGaussSeidelContact:
                ExecuteSolveGaussSeidelContact();
                break;
            case ConstraintSolverOperation.SolveGaussSeidelRecovery:
                ExecuteGaussSeidelRecovery();
                break;
            case ConstraintSolverOperation.PrepareJacobiRecovery:
                PrepareJacobiRecovery();
                break;
            case ConstraintSolverOperation.FinalizeJacobiRecovery:
                FinalizeJacobiRecovery();
                break;
            case ConstraintSolverOperation.InitializeContactIteration:
                InitializeContactIteration(SubstepIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , IterationState
#endif
                );
                break;
            case ConstraintSolverOperation.FinalizeSubstepTelemetry:
#if RTS_CONTACT_DIAGNOSTICS
                FinalizeSubstepTelemetry(RuntimeState);
#endif
                break;
            case ConstraintSolverOperation.FinalizeVelocity:
#if RTS_CONTACT_DIAGNOSTICS
                FinalizeVelocityStatistics(RuntimeState, BlockCount);
#endif
                break;
            case ConstraintSolverOperation.MergeParallelDebuggerPairs:
#if RTS_CONTACT_DIAGNOSTICS
                MergeParallelSimulationDebuggerPairScratch();
#endif
                break;
            case ConstraintSolverOperation.FinalizePipeline:
                FinalizeContactPipeline(RuntimeState);
                break;
        }
    }
}
}
