using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 当前 timestep/substep 求解器执行拥有的可变状态。预测位置与 XPBD 修正永不进入持久 World 状态。
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
