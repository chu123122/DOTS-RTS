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
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdBodyStepState> StepStates;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
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
}
