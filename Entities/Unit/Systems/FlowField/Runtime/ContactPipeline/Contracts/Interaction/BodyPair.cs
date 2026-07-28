using System.Collections.Generic;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 帧级无序 body 关系。仅作为发现/交互数据；不含求解器模式、lambda、激活状态或诊断历史。
/// </summary>
public struct BodyPair
{
    public int BodyA;
    public int BodyB;

    public BodyPair(int bodyA, int bodyB)
    {
        if (bodyA <= bodyB)
        {
            BodyA = bodyA;
            BodyB = bodyB;
        }
        else
        {
            BodyA = bodyB;
            BodyB = bodyA;
        }
    }
}

public struct BodyPairComparer : IComparer<BodyPair>
{
    public int Compare(BodyPair x, BodyPair y)
    {
        int bodyAComparison = x.BodyA.CompareTo(y.BodyA);
        return bodyAComparison != 0
            ? bodyAComparison
            : x.BodyB.CompareTo(y.BodyB);
    }
}
}
