using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
internal struct SoftAvoidanceFrameResources
{
    public NativeArray<int> IncidentOffsets;
    public NativeArray<int> IncidentWriteCursors;
    public NativeList<int> IncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> PairContributions;

    public static SoftAvoidanceFrameResources Create(int unitCount, bool useParallelJacobi)
    {
        return new SoftAvoidanceFrameResources
        {
            IncidentOffsets = useParallelJacobi
                ? new NativeArray<int>(unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            IncidentWriteCursors = useParallelJacobi
                ? new NativeArray<int>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory)
                : default,
            IncidentPairIndices = useParallelJacobi
                ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob)
                : default,
            PairContributions = useParallelJacobi
                ? new NativeList<SoftAvoidancePairContribution>(math.max(unitCount * 4, 1), Allocator.TempJob)
                : default
        };
    }

    public SoftAvoidanceJob CreateJob(
        ContactPipelineConfiguration configuration,
        FlowFieldGrid grid,
        CrowdStepBodyResources body,
        InteractionCertificationFrameResources certification,
        ConstraintSolverFrameResources solver,
        ContactPipelineExecutionResources execution,
        ContactDiagnosticsFrameResources diagnostics)
    {
        return new SoftAvoidanceJob
        {
            Configuration = configuration,
            Grid = grid.Grid,
            GridOrigin = grid.GridOrigin,
            GridDimensions = grid.GridDimensions,
            CellRadius = grid.CellRadius,
            Bodies = body.Bodies,
            StepStates = body.StepStates,
            SoftAvoidancePairs = certification.SoftAvoidancePairs,
            SoftIncidentOffsets = IncidentOffsets,
            SoftIncidentWriteCursors = IncidentWriteCursors,
            SoftIncidentPairIndices = IncidentPairIndices,
            SoftPairContributions = PairContributions,
            ActiveIncidentIndexState = solver.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            BlockStatistics = execution.ParallelJacobiBlockTelemetry,
            EscapeCountsByBlock = solver.DirtyBodyBlockOffsets,
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        if (IncidentOffsets.IsCreated)
            combined = JobHandle.CombineDependencies(combined, IncidentOffsets.Dispose(finalReader));
        if (IncidentWriteCursors.IsCreated)
            combined = JobHandle.CombineDependencies(combined, IncidentWriteCursors.Dispose(finalReader));
        if (IncidentPairIndices.IsCreated)
            combined = JobHandle.CombineDependencies(combined, IncidentPairIndices.Dispose(finalReader));
        if (PairContributions.IsCreated)
            combined = JobHandle.CombineDependencies(combined, PairContributions.Dispose(finalReader));
        return combined;
    }
}
}
