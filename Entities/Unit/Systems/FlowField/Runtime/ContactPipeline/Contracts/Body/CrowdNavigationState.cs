using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Navigation-only product. It carries only path semantics needed by movement intent.
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
