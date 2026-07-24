using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Transitional host for one crowd step. Stable input/navigation/intent semantics
/// are physically composed from explicit contracts; timestep/substep solver fields
/// remain flat temporarily because several Burst jobs pass vector fields by ref/out.
/// </summary>
public struct FlowMovementFrameState
{
    public CrowdBodySnapshot Body;
    public CrowdNavigationState Navigation;
    public CrowdMotionIntent MotionIntent;

    public Entity Entity
    {
        get => Body.Entity;
        set => Body.Entity = value;
    }

    public float3 CurrentPosition
    {
        get => Body.Position;
        set => Body.Position = value;
    }

    public quaternion CurrentRotation
    {
        get => Body.Rotation;
        set => Body.Rotation = value;
    }

    public float3 CurrentVelocity
    {
        get => Body.Velocity;
        set => Body.Velocity = value;
    }

    public float MoveSpeed
    {
        get => Body.MoveSpeed;
        set => Body.MoveSpeed = value;
    }

    // Serialized/authoring compatibility name. The controller integrates this as
    // a velocity-change rate rather than a Newtonian force.
    public float MaxForce
    {
        get => Body.MaxAcceleration;
        set => Body.MaxAcceleration = value;
    }

    public float InverseMass
    {
        get => Body.InverseMass;
        set => Body.InverseMass = value;
    }

    public float Radius
    {
        get => Body.Radius;
        set => Body.Radius = value;
    }

    public int2 CellPosition
    {
        get => Navigation.Cell;
        set => Navigation.Cell = value;
    }

    public bool IsSettled
    {
        get => Navigation.IsSettled != 0;
        set => Navigation.IsSettled = (byte)(value ? 1 : 0);
    }

    public bool IsInsideGrid
    {
        get => Body.IsInsideSimulationDomain != 0;
        set => Body.IsInsideSimulationDomain = (byte)(value ? 1 : 0);
    }

    public float3 IndependentForce
    {
        get => MotionIntent.SteeringVelocityError;
        set => MotionIntent.SteeringVelocityError = value;
    }

    // Current timestep certification evidence. Substeps consume views certified
    // against this complete baseline trajectory.
    public float3 TimestepStartPosition;
    public float3 TimestepPredictedPosition;
    public float2 TimestepEnvelopeMin;
    public float2 TimestepEnvelopeMax;
    public float2 TimestepInteractionEnvelopeMin;
    public float2 TimestepInteractionEnvelopeMax;
    public byte TimestepEscaped;
    public float3 TimestepContactCorrection;
    public float3 TimestepWallCorrection;

    // Substep motion and constraint state.
    public float3 SoftAvoidanceVelocity;
    public float3 WallAvoidanceVelocity;
    public int SoftAvoidanceNeighborCount;
    public float3 BasePredictedVelocity;
    public float3 IntegratedVelocity;
    public float3 StartPosition;
    public float3 UnconstrainedPredictedPosition;
    public float3 VelocityBeforeContact;
    public float3 PredictedPosition;
    public float3 PreviousSubstepPosition;
    public float3 ContactPositionCorrection;
    public float3 WallPositionCorrection;
}
}
