using System.Collections.Generic;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Mathematical mode of one frame-local disc contact constraint.
/// </summary>
public enum ContactConstraintMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// Solver-owned frame-local contact record. Fields are direct storage rather than
/// compatibility forwarding properties. Interaction discovery uses BodyPair instead.
/// </summary>
public struct ContactConstraint
{
    // Definition, assembled before solver consumption.
    public int BodyA;
    public int BodyB;
    public float3 PredictiveNormal;
    public ContactConstraintMode ContactMode;
    public byte PredictiveNormalOriented;
    public byte IsDormant;

    // Mutable solver state.
    public float Lambda;
    public byte WasActivated;

    // Timestep utilization/provenance state.
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
