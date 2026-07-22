using System.Collections.Generic;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Solver-facing contact semantics for the current timestep view.
/// Persistent lifecycle state lives in ContactPipeline/Persistent and must not
/// be added to this frame-local constraint type.
/// </summary>
public enum UnitContactMode : byte
{
    Regular,
    Predictive
}

/// <summary>
/// Frame-local XPBD contact constraint. Body indices and lambda are never
/// persisted across frames. PredictiveNormal is oriented once per timestep.
/// </summary>
public struct UnitCollisionPair
{
    public int BodyA;
    public int BodyB;
    public float Lambda;
    public float3 PredictiveNormal;
    public UnitContactMode ContactMode;
    public byte PredictiveNormalOriented;
    public byte WasActivated;
    public byte WasActivatedThisTimestep;
    public byte WasCorrectedThisTimestep;
    public byte IsDormant;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
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
