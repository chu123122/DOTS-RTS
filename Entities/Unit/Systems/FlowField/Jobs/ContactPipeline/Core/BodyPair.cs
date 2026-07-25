using System.Collections.Generic;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Frame-local unordered body relationship. This is discovery/interaction data only;
/// it carries no solver mode, lambda, activation state, or diagnostics history.
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
