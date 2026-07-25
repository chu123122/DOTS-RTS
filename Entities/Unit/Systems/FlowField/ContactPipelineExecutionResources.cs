using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Frame-local coordination state, not gameplay or candidate data.</summary>
internal struct ContactPipelineExecutionResources
{
    public NativeReference<ParallelJacobiExecutionState> ParallelJacobiRuntimeState;
    public NativeReference<SerialContactPipelineControlState> SerialControlState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ParallelJacobiIterationTelemetry> ParallelJacobiIterationState;
    public NativeList<JacobiBlockTelemetry> ParallelJacobiBlockTelemetry;
#endif

    public static ContactPipelineExecutionResources Create(int unitCount, bool useParallelJacobi)
    {
        return new ContactPipelineExecutionResources
        {
            ParallelJacobiRuntimeState = useParallelJacobi
                ? new NativeReference<ParallelJacobiExecutionState>(Allocator.TempJob)
                : default,
            SerialControlState = new NativeReference<SerialContactPipelineControlState>(Allocator.TempJob),
#if RTS_CONTACT_DIAGNOSTICS
            ParallelJacobiIterationState = useParallelJacobi
                ? new NativeReference<ParallelJacobiIterationTelemetry>(Allocator.TempJob)
                : default,
            ParallelJacobiBlockTelemetry = useParallelJacobi
                ? new NativeList<JacobiBlockTelemetry>(math.max((unitCount * 4 + 63) / 64, 1), Allocator.TempJob)
                : default,
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        if (ParallelJacobiRuntimeState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiRuntimeState.Dispose(finalReader));
        if (SerialControlState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, SerialControlState.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        if (ParallelJacobiIterationState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiIterationState.Dispose(finalReader));
        if (ParallelJacobiBlockTelemetry.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiBlockTelemetry.Dispose(finalReader));
#endif
        return combined;
    }
}
}
