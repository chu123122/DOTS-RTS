using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 当前 timestep/substep 求解器拥有的位置、速度积分与 XPBD 修正状态。
/// 软避让中间量由独立的 CrowdAvoidanceState 承载。
/// </summary>
public struct CrowdSolverBodyState
{
    public float3 BaseVelocity;
    public float3 IntegratedVelocity;
    public float3 SubstepStartPosition;
    public float3 UnconstrainedPosition;
    public float3 VelocityBeforeContact;
    public float3 SolvedPosition;
    public float3 PreviousSubstepPosition;
    // 当前 substep 的修正量；每个 substep 开始时重置。
    public float3 ContactCorrection;
    public float3 WallCorrection;
    // 当前 timestep 的累计修正量；仅在 step 初始化时重置。
    public float3 TimestepContactCorrection;
    public float3 TimestepWallCorrection;
}
}
