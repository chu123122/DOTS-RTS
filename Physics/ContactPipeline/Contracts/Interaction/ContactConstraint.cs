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
/// NarrowPhase 产出的不可变约束定义。
/// </summary>
public struct ContactConstraintDefinition
{
    public int BodyA;
    public int BodyB;
    public float3 PredictiveNormal;
    public ContactConstraintMode ContactMode;
    public byte IsDormant;
}

/// <summary>
/// TimestepCache/Solver 独占的可变约束状态；不得进入 BroadPhase 或跨帧缓存。
/// </summary>
public struct ContactConstraintRuntime
{
    public float Lambda;
    public float3 OrientedPredictiveNormal;
    public byte PredictiveNormalOriented;
    public byte WasActivated;
    public byte WasActivatedThisTimestep;
    public byte WasCorrectedThisTimestep;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
}

/// <summary>
/// NarrowPhase 定义与 Solver runtime 的显式组合记录。定义只能通过
/// Definition 字段在 NarrowPhase 装配；Solver 只能更新 Runtime。
/// </summary>
public struct ContactConstraint
{
    public ContactConstraintDefinition Definition;
    public ContactConstraintRuntime Runtime;

    public readonly int BodyA => Definition.BodyA;
    public readonly int BodyB => Definition.BodyB;
    public readonly float3 PredictiveNormal => Definition.PredictiveNormal;
    public readonly ContactConstraintMode ContactMode => Definition.ContactMode;
    public readonly byte PredictiveNormalOriented =>
        Runtime.PredictiveNormalOriented;
    public readonly byte IsDormant => Definition.IsDormant;

    public float Lambda
    {
        readonly get => Runtime.Lambda;
        set => Runtime.Lambda = value;
    }

    public byte WasActivated
    {
        readonly get => Runtime.WasActivated;
        set => Runtime.WasActivated = value;
    }

    public byte WasActivatedThisTimestep
    {
        readonly get => Runtime.WasActivatedThisTimestep;
        set => Runtime.WasActivatedThisTimestep = value;
    }

    public byte WasCorrectedThisTimestep
    {
        readonly get => Runtime.WasCorrectedThisTimestep;
        set => Runtime.WasCorrectedThisTimestep = value;
    }

    public byte WasAddedByFallback
    {
        readonly get => Runtime.WasAddedByFallback;
        set => Runtime.WasAddedByFallback = value;
    }

    public int FirstActivatedSubstep
    {
        readonly get => Runtime.FirstActivatedSubstep;
        set => Runtime.FirstActivatedSubstep = value;
    }

    public int ActivatedSubstepCount
    {
        readonly get => Runtime.ActivatedSubstepCount;
        set => Runtime.ActivatedSubstepCount = value;
    }
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
