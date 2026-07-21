using Unity.Mathematics;

/// <summary>
/// Soft avoidance math shared by the per-substep contact solver.
/// Neighbor discovery and accumulation live in SolveXpbdUnitContactsJob so every
/// substep can use the latest constrained positions.
/// </summary>
public static class SoftAvoidanceMath
{
    public static float3 CalculateUnitForce(
        float3 position,
        float3 neighborPosition,
        float radius,
        float neighborRadius,
        float moveSpeed,
        float softShell)
    {
        softShell = math.max(0f, softShell);
        if (softShell <= 0f)
            return float3.zero;

        float3 difference = position - neighborPosition;
        difference.y = 0;
        float distanceSq = math.lengthsq(difference);
        float activationDistance = math.max(0f, radius) +
                                   math.max(0f, neighborRadius) +
                                   softShell;
        if (distanceSq >= activationDistance * activationDistance ||
            distanceSq <= 0.00001f)
            return float3.zero;

        float distance = math.sqrt(distanceSq);
        float surfaceGap = distance - math.max(0f, radius) - math.max(0f, neighborRadius);
        float softFactor = math.saturate((softShell - surfaceGap) / softShell);
        return difference / distance * softFactor * moveSpeed;
    }

    public static float3 CalculateWallForce(
        float3 position,
        float3 wallPosition,
        float moveSpeed,
        float wallCheckRadius)
    {
        float3 difference = position - wallPosition;
        difference.y = 0;
        float distanceSq = math.lengthsq(difference);
        if (distanceSq >= wallCheckRadius * wallCheckRadius || distanceSq <= 0.0001f)
            return float3.zero;

        float distance = math.sqrt(distanceSq);
        float repelStrength = (wallCheckRadius - distance) / distance * 10f;
        return difference / distance * repelStrength * moveSpeed;
    }
}
