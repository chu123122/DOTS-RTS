using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 仅导航产物，仅承载运动意图所需的路径语义。
/// </summary>
public struct CrowdNavigationState
{
    public int2 Cell;
    public int BestDirectionIndex;
    public ushort IntegrationValue;
    public byte IsReachable;
    public byte IsBlocked;
    public byte IsSettled;
}
}
