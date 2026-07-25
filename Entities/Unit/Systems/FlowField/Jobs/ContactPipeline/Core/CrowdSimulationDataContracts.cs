using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Immutable body facts captured from ECS at the beginning of one crowd step.
/// Navigation, interaction certification and solver state live in separate arrays.
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

/// <summary>
/// Navigation movement policy. SteeringVelocityError preserves the current controller
/// math: the motion integrator clamps it by Body.MaxAcceleration before integration.
/// </summary>
public struct CrowdMotionIntent
{
    public float3 PreferredVelocity;
    public float3 SteeringVelocityError;
}

/// <summary>
/// Timestep-scoped authoritative motion evidence consumed by the interaction certifier.
/// This is not persistent candidate state and lower consumers may only report escapes.
/// </summary>
public struct CrowdMotionEvidence
{
    public float3 TrajectoryStart;
    public float3 BaselineEnd;
    public float2 ContactEnvelopeMin;
    public float2 ContactEnvelopeMax;
    public float2 InteractionEnvelopeMin;
    public float2 InteractionEnvelopeMax;
    public float3 ContactCorrection;
    public float3 WallCorrection;
    public uint MotionVersion;
    public byte EnvelopeEscaped;
}

/// <summary>
/// Mutable state owned by the current timestep/substep solver execution.
/// Predicted positions and XPBD corrections never enter persistent World state.
/// </summary>
public struct CrowdBodyStepState
{
    public float3 SoftAvoidanceVelocity;
    public float3 WallAvoidanceVelocity;
    public int SoftAvoidanceNeighborCount;

    public float3 BaseVelocity;
    public float3 IntegratedVelocity;
    public float3 SubstepStartPosition;
    public float3 UnconstrainedPosition;
    public float3 VelocityBeforeContact;
    public float3 SolvedPosition;
    public float3 PreviousSubstepPosition;
    public float3 ContactCorrection;
    public float3 WallCorrection;
}

/// <summary>
/// Detached post-solve product consumed by ECS writeback and completed-step observers.
/// </summary>
public struct CrowdBodyResult
{
    public float3 Position;
    public float3 Velocity;
    public quaternion Rotation;
}
}
