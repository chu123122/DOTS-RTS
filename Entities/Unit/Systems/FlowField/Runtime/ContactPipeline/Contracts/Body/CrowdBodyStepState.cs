using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Mutable state owned by the current timestep/substep solver execution.
/// Predicted positions and XPBD corrections never enter persistent World state.
/// </summary>
public struct CrowdBodyStepState
{
    public float3 SoftAvoidanceVelocity;
    public float3 WallAvoidanceVelocity;
    public int SoftAvoidanceNeighborCount;

    public float3 BaseVelocity;
    public float3 IntegratedVelocity;
    public float3 SubstepStartPosition;
    public float3 UnconstrainedPosition;
    public float3 VelocityBeforeContact;
    public float3 SolvedPosition;
    public float3 PreviousSubstepPosition;
    public float3 ContactCorrection;
    public float3 WallCorrection;
}
}
