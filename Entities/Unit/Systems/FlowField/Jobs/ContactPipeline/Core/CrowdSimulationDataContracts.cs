using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Immutable body facts captured from ECS at the beginning of one crowd step.
/// This type deliberately contains no navigation-cell, contact-cache, solver or
/// diagnostics state.
/// </summary>
public struct CrowdBodySnapshot
{
    public Entity Entity;
    public float3 Position;
    public quaternion Rotation;
    public float3 Velocity;
    public float MoveSpeed;
    public float MaxAcceleration;
    public float InverseMass;
    public float Radius;
    public byte IsInsideSimulationDomain;
}

/// <summary>
/// Navigation-only result for one body. It describes path availability and
/// arrival policy without exposing the FlowField cell to contact or solver code.
/// </summary>
public struct CrowdNavigationState
{
    public int2 Cell;
    public int BestDirectionIndex;
    public ushort IntegrationValue;
    public byte IsReachable;
    public byte IsSettled;
}

/// <summary>
/// Motion policy produced by navigation. PreferredVelocity is the target motion;
/// SteeringVelocityError preserves the current controller semantics while the
/// historical IndependentForce name is migrated.
/// </summary>
public struct CrowdMotionIntent
{
    public float3 PreferredVelocity;
    public float3 SteeringVelocityError;
}

/// <summary>
/// Authoritative evidence used by the interaction certifier for one horizon.
/// It is not persistent cache state and must not be mutated by lower consumers.
/// </summary>
public struct CrowdMotionEvidence
{
    public float3 TrajectoryStart;
    public float3 BaselineEnd;
    public float2 ContactEnvelopeMin;
    public float2 ContactEnvelopeMax;
    public float2 InteractionEnvelopeMin;
    public float2 InteractionEnvelopeMax;
    public uint MotionVersion;
}

/// <summary>
/// Mutable body state owned by the current substep/solver execution.
/// Predicted positions never belong to persistent world state.
/// </summary>
public struct CrowdBodyStepState
{
    public float3 PreviousSubstepPosition;
    public float3 UnconstrainedPosition;
    public float3 SolvedPosition;
    public float3 VelocityBeforeContact;
    public float3 IntegratedVelocity;
    public float3 ContactCorrection;
    public float3 WallCorrection;
}

/// <summary>
/// Final detached result consumed by ECS writeback and completed-step observers.
/// </summary>
public struct CrowdBodyResult
{
    public float3 Position;
    public float3 Velocity;
    public quaternion Rotation;
    public byte IsSettled;
}
}
