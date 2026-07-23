using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// One constraint result produced from the shared Jacobi iteration snapshot.
/// Body corrections are gathered through ActiveConstraintIncidentPairIndices.
/// </summary>
public struct JacobiContactProjection
{
    public float3 Normal;
    public float AppliedLambda;
    public float ConstraintValue;
    public float PairCorrection;
}
}
