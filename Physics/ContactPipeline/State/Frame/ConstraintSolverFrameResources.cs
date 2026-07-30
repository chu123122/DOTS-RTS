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
    public NativeList<byte> ActiveIncidentBodyWorkset;
    public NativeList<byte> ActiveIncidentPairWorkset;

    public static ConstraintSolverFrameResources Create(int unitCount)
    {
        int one = math.max(unitCount, 1);
        return new ConstraintSolverFrameResources
        {
            CorrectedBodyFlags = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            CorrectedBodyIndices = new NativeList<int>(one, Allocator.TempJob),
            ActiveIncidentOffsets = new NativeArray<int>(unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            ActiveIncidentWriteCursors = new NativeArray<int>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            ActiveIncidentPairIndices = new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob),
            JacobiPairCorrections = new NativeList<JacobiPairCorrection>(math.max(unitCount * 4, 1), Allocator.TempJob),
            EnvelopeEscapeFlags = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            ParallelBodyResults = new NativeArray<ParallelBodyStageResult>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            DirtyBodyBlockOffsets = new NativeArray<int>(one, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            ActiveIncidentIndexState =
                new NativeReference<ActiveIncidentIndexState>(
                    Allocator.TempJob),
            ActiveIncidentBodyWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            ActiveIncidentPairWorkset =
                new NativeList<byte>(one, Allocator.TempJob)
        };
    }

    public ConstraintSolverJob CreateJob(
        ContactPipelineConfiguration configuration,
        CrowdObstacleSnapshot obstacles,
        CrowdStepBodyResources body,
        NarrowPhaseConstraintBatch constraints,
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
            RuntimeState = execution.PipelineRuntimeState,
            Grid = obstacles.Cells,
            GridOrigin = obstacles.Geometry.Origin,
            GridDimensions = obstacles.Geometry.Dimensions,
            CellRadius = obstacles.Geometry.CellRadius,
            Bodies = body.Bodies,
            MotionEvidence = body.MotionEvidence,
            AvoidanceStates = body.AvoidanceStates,
            StepStates = body.StepStates,
            TimestepContactPairs = constraints.HardContacts,
            CorrectedBodyFlags = CorrectedBodyFlags,
            CorrectedBodyIndices = CorrectedBodyIndices,
            ActiveIncidentOffsets = ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = ActiveIncidentPairIndices,
            JacobiPairCorrections = JacobiPairCorrections,
            ActiveIncidentIndexState = ActiveIncidentIndexState,
            ParallelBodyStatistics = ParallelBodyResults,
#if RTS_CONTACT_DIAGNOSTICS
            IterationState = execution.SolverIterationState,
            BlockStatistics = execution.JacobiBlockStatistics,
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
        if (ActiveIncidentBodyWorkset.IsCreated)
            combined = JobHandle.CombineDependencies(
                combined, ActiveIncidentBodyWorkset.Dispose(finalReader));
        if (ActiveIncidentPairWorkset.IsCreated)
            combined = JobHandle.CombineDependencies(
                combined, ActiveIncidentPairWorkset.Dispose(finalReader));
        return combined;
    }
}
}
