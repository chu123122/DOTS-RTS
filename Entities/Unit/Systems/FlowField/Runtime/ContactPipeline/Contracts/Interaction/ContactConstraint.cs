using System.Collections.Generic;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 帧级 disc 接触约束的数学模式。
/// </summary>
public enum ContactConstraintMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// 求解器拥有的帧级接触记录。字段为直接存储，而非兼容转发属性。交互发现使用 BodyPair。
/// </summary>
public struct ContactConstraint
{
    // 定义部分：求解器消费前装填。
    public int BodyA;
    public int BodyB;
    public float3 PredictiveNormal;
    public ContactConstraintMode ContactMode;
    public byte PredictiveNormalOriented;
    public byte IsDormant;

    // 可变求解器状态。
    public float Lambda;
    public byte WasActivated;

    // timestep 利用/来源状态。
    public byte WasActivatedThisTimestep;
    public byte WasCorrectedThisTimestep;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
}

public struct ContactConstraintComparer : IComparer<ContactConstraint>
{
    public int Compare(ContactConstraint x, ContactConstraint y)
    {
        int bodyAComparison = x.BodyA.CompareTo(y.BodyA);
        return bodyAComparison != 0
            ? bodyAComparison
            : x.BodyB.CompareTo(y.BodyB);
    }
}
}
