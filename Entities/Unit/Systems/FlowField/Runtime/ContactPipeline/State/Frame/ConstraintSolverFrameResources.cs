using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
internal struct ConstraintSolverFrameResources
{
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
    public NativeArray<byte> EnvelopeEscapeFlags;
    public NativeArray<ParallelBodyStageResult> ParallelBodyResults;
    public NativeArray<int> DirtyBodyBlockOffsets;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;

    public static ConstraintSolverFrameResources Create(
        int unitCount,
        bool usesJacobiScratch,
        bool useParallelJacobi)
    {
        int one = math.max(unitCount, 1);
        return new ConstraintSolverFrameResources
        {
            CorrectedBodyFlags = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            CorrectedBodyIndices = new NativeList<int>(one, Allocator.TempJob),
            ActiveIncidentOffsets = usesJacobiScratch
                ? new NativeArray<int>(unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            ActiveIncidentWriteCursors = usesJacobiScratch
                ? new NativeArray<int>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            ActiveIncidentPairIndices = usesJacobiScratch
                ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob)
                : default,
            JacobiPairCorrections = usesJacobiScratch
                ? new NativeList<JacobiPairCorrection>(math.max(unitCount * 4, 1), Allocator.TempJob)
                : default,
            EnvelopeEscapeFlags = useParallelJacobi
                ? new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            ParallelBodyResults = useParallelJacobi
                ? new NativeArray<ParallelBodyStageResult>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            DirtyBodyBlockOffsets = useParallelJacobi
                ? new NativeArray<int>(one, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            ActiveIncidentIndexState = usesJacobiScratch
                ? new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob)
                : default
        };
    }

    public ConstraintSolverJob CreateJob(
        ContactPipelineConfiguration configuration,
        FlowFieldGrid grid,
        CrowdStepBodyResources body,
        InteractionCertificationFrameResources certification,
        ContactPipelineExecutionResources execution,
        ContactDiagnosticsFrameResources diagnostics,
        Entity diagnosticSelectedEntity,
        SimulationDebuggerCaptureMask captureMask,
        int maximumVisualizedPairs,
        NativeList<SimulationDebuggerPairSample> debuggerSelectedPairs,
        NativeReference<SimulationDebuggerUnitSample> debuggerSelectedUnit,
        NativeReference<byte> debuggerSelectedUnitValid)
    {
        return new ConstraintSolverJob
        {
            Configuration = configuration,
            SerialControl = execution.SerialControlState,
            Grid = grid.Grid,
            GridOrigin = grid.GridOrigin,
            GridDimensions = grid.GridDimensions,
            CellRadius = grid.CellRadius,
            Bodies = body.Bodies,
            NavigationStates = body.NavigationStates,
            MotionIntents = body.MotionIntents,
            MotionEvidence = body.MotionEvidence,
            StepStates = body.StepStates,
            TimestepContactPairs = certification.TimestepContactPairs,
            CorrectedBodyFlags = CorrectedBodyFlags,
            CorrectedBodyIndices = CorrectedBodyIndices,
            ActiveIncidentOffsets = ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = ActiveIncidentPairIndices,
            JacobiPairCorrections = JacobiPairCorrections,
            ActiveIncidentIndexState = ActiveIncidentIndexState,
            ParallelBodyStatistics = ParallelBodyResults,
#if RTS_CONTACT_DIAGNOSTICS
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            SelectedBodyDiagnostic = diagnostics.SelectedBody,
            HeatSamples = diagnostics.HeatSamples,
            SimulationDebuggerCaptureMask = captureMask,
            SimulationDebuggerMaximumPairs = maximumVisualizedPairs,
            SimulationDebuggerSelectedPairs = debuggerSelectedPairs,
            ParallelSimulationDebuggerPairCandidates = diagnostics.ParallelPairCandidates,
            ParallelSimulationDebuggerPairScratch = diagnostics.ParallelPairScratch,
            SimulationDebuggerSelectedUnit = debuggerSelectedUnit,
            SimulationDebuggerSelectedUnitValid = debuggerSelectedUnitValid,
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = JobHandle.CombineDependencies(combined, CorrectedBodyFlags.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, CorrectedBodyIndices.Dispose(finalReader));
        if (ActiveIncidentOffsets.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ActiveIncidentOffsets.Dispose(finalReader));
        if (ActiveIncidentWriteCursors.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ActiveIncidentWriteCursors.Dispose(finalReader));
        if (ActiveIncidentPairIndices.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ActiveIncidentPairIndices.Dispose(finalReader));
        if (JacobiPairCorrections.IsCreated)
            combined = JobHandle.CombineDependencies(combined, JacobiPairCorrections.Dispose(finalReader));
        if (EnvelopeEscapeFlags.IsCreated)
            combined = JobHandle.CombineDependencies(combined, EnvelopeEscapeFlags.Dispose(finalReader));
        if (ParallelBodyResults.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelBodyResults.Dispose(finalReader));
        if (DirtyBodyBlockOffsets.IsCreated)
            combined = JobHandle.CombineDependencies(combined, DirtyBodyBlockOffsets.Dispose(finalReader));
        if (ActiveIncidentIndexState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ActiveIncidentIndexState.Dispose(finalReader));
        return combined;
    }
}
}
