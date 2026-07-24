using System.Collections.Generic;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Solver-facing contact semantics for the current timestep view.
/// Persistent lifecycle state lives in ContactPipeline/Persistent and must not
/// be added to the frame-local constraint definition.
/// </summary>
public enum UnitContactMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// Immutable-by-convention data assembled before XPBD evaluation. Dormant is a
/// classification/schedule fact and must be removed before the active solver view
/// is consumed.
/// </summary>
public struct ContactConstraintDefinition
{
    public int BodyA;
    public int BodyB;
    public float3 PredictiveNormal;
    public UnitContactMode ContactMode;
    public byte PredictiveNormalOriented;
    public byte IsDormant;
}

/// <summary>
/// Mutable state reset for each substep/iteration as required by the solver.
/// </summary>
public struct ContactConstraintRuntime
{
    public float Lambda;
    public byte WasActivated;
}

/// <summary>
/// Timestep provenance and utilization history. It is kept separate from the
/// mathematical definition so future gameplay-only builds can move observation-
/// only fields behind the diagnostics boundary.
/// </summary>
public struct ContactConstraintHistory
{
    public byte WasActivatedThisTimestep;
    public byte WasCorrectedThisTimestep;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
}

/// <summary>
/// Transitional frame-local contact value. Existing algorithms keep source
/// compatibility through forwarding properties while storage is physically split
/// into definition, runtime and history lifetimes.
/// </summary>
public struct UnitCollisionPair
{
    public ContactConstraintDefinition Definition;
    public ContactConstraintRuntime Runtime;
    public ContactConstraintHistory History;

    public int BodyA
    {
        get => Definition.BodyA;
        set => Definition.BodyA = value;
    }

    public int BodyB
    {
        get => Definition.BodyB;
        set => Definition.BodyB = value;
    }

    public float3 PredictiveNormal
    {
        get => Definition.PredictiveNormal;
        set => Definition.PredictiveNormal = value;
    }

    public UnitContactMode ContactMode
    {
        get => Definition.ContactMode;
        set => Definition.ContactMode = value;
    }

    public byte PredictiveNormalOriented
    {
        get => Definition.PredictiveNormalOriented;
        set => Definition.PredictiveNormalOriented = value;
    }

    public byte IsDormant
    {
        get => Definition.IsDormant;
        set => Definition.IsDormant = value;
    }

    public float Lambda
    {
        get => Runtime.Lambda;
        set => Runtime.Lambda = value;
    }

    public byte WasActivated
    {
        get => Runtime.WasActivated;
        set => Runtime.WasActivated = value;
    }

    public byte WasActivatedThisTimestep
    {
        get => History.WasActivatedThisTimestep;
        set => History.WasActivatedThisTimestep = value;
    }

    public byte WasCorrectedThisTimestep
    {
        get => History.WasCorrectedThisTimestep;
        set => History.WasCorrectedThisTimestep = value;
    }

    public byte WasAddedByFallback
    {
        get => History.WasAddedByFallback;
        set => History.WasAddedByFallback = value;
    }

    public int FirstActivatedSubstep
    {
        get => History.FirstActivatedSubstep;
        set => History.FirstActivatedSubstep = value;
    }

    public int ActivatedSubstepCount
    {
        get => History.ActivatedSubstepCount;
        set => History.ActivatedSubstepCount = value;
    }
}

public struct UnitCollisionPairComparer : IComparer<UnitCollisionPair>
{
    public int Compare(UnitCollisionPair x, UnitCollisionPair y)
    {
        int bodyAComparison = x.BodyA.CompareTo(y.BodyA);
        return bodyAComparison != 0
            ? bodyAComparison
            : x.BodyB.CompareTo(y.BodyB);
    }
}
}
