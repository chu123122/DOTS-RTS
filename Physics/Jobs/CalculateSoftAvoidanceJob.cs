using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{

/// <summary>
/// 子步接触求解器共用的软避让速度算法。邻居发现由校验阶段负责；
/// SoftAvoidanceJob 基于最新子步位置累加此处的成对修正。
/// </summary>
public static class SoftAvoidanceMath
{
    public static bool TryCalculatePairVelocities(
        SoftAvoidanceVelocitySolverMode solverMode,
        float3 positionA,
        float3 positionB,
        float3 velocityA,
        float3 velocityB,
        float radiusA,
        float radiusB,
        float inverseMassA,
        float inverseMassB,
        float moveSpeedA,
        float moveSpeedB,
        float softShell,
        float timeHorizon,
        float minimumCorrectionTime,
        float3 fallbackNormal,
        out float3 correctionA,
        out float3 correctionB)
    {
        switch (solverMode)
        {
            case SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle:
                return TryCalculateRvoVelocities(
                    positionA,
                    positionB,
                    velocityA,
                    velocityB,
                    radiusA,
                    radiusB,
                    inverseMassA,
                    inverseMassB,
                    softShell,
                    timeHorizon,
                    minimumCorrectionTime,
                    fallbackNormal,
                    out correctionA,
                    out correctionB);
            default:
                correctionA = CalculateUnitVelocity(
                    positionA,
                    positionB,
                    radiusA,
                    radiusB,
                    moveSpeedA,
                    softShell);
                correctionB = CalculateUnitVelocity(
                    positionB,
                    positionA,
                    radiusB,
                    radiusA,
                    moveSpeedB,
                    softShell);
                return math.lengthsq(correctionA) > 0f || math.lengthsq(correctionB) > 0f;
        }
    }

    public static float3 CalculateUnitVelocity(
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

    public static float3 CalculateWallVelocity(
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

    private static bool TryCalculateRvoVelocities(
        float3 positionA,
        float3 positionB,
        float3 velocityA,
        float3 velocityB,
        float radiusA,
        float radiusB,
        float inverseMassA,
        float inverseMassB,
        float softShell,
        float timeHorizon,
        float minimumCorrectionTime,
        float3 fallbackNormal,
        out float3 correctionA,
        out float3 correctionB)
    {
        correctionA = float3.zero;
        correctionB = float3.zero;

        float inverseMassSum = math.max(0f, inverseMassA) + math.max(0f, inverseMassB);
        if (inverseMassSum <= 0f)
            return false;

        float3 relativePosition = positionA - positionB;
        relativePosition.y = 0f;
        float3 relativeVelocity = velocityA - velocityB;
        relativeVelocity.y = 0f;
        float relativeSpeedSq = math.lengthsq(relativeVelocity);
        if (relativeSpeedSq <= 0.0000001f ||
            math.dot(relativePosition, relativeVelocity) >= 0f)
            return false;

        float horizon = math.max(0.0001f, timeHorizon);
        float closestTime = math.clamp(
            -math.dot(relativePosition, relativeVelocity) / relativeSpeedSq,
            0f,
            horizon);
        if (closestTime <= 0f)
            return false;

        float3 closestDelta = relativePosition + relativeVelocity * closestTime;
        float closestDistance = math.length(closestDelta);
        float safetyDistance = math.max(0f, radiusA) +
                               math.max(0f, radiusB) +
                               math.max(0f, softShell);
        if (closestDistance >= safetyDistance)
            return false;

        float3 normal = math.normalizesafe(
            closestDelta,
            math.normalizesafe(relativePosition, fallbackNormal));
        float correctionTime = math.max(closestTime, math.max(0.0001f, minimumCorrectionTime));
        float3 relativeCorrection = normal *
                                    ((safetyDistance - closestDistance) / correctionTime);
        float weightA = math.max(0f, inverseMassA) / inverseMassSum;
        float weightB = math.max(0f, inverseMassB) / inverseMassSum;
        correctionA = relativeCorrection * weightA;
        correctionB = -relativeCorrection * weightB;
        return true;
    }

    /// <summary>
    /// 把速度缓冲按每秒响应率转换为与 substep 数无关的指数响应比例。
    /// </summary>
    public static float CalculateBufferAlpha(float responseRate, float deltaTime)
    {
        return 1f - math.exp(-math.max(0f, responseRate) * math.max(0f, deltaTime));
    }

    public static float3 ApplyVelocityBuffer(
        float3 baseVelocity,
        float3 avoidanceVelocity,
        float responseRate,
        float deltaTime,
        float maxSpeed)
    {
        float3 velocity = baseVelocity +
                          avoidanceVelocity * CalculateBufferAlpha(responseRate, deltaTime);
        float maxSpeedSq = math.max(0f, maxSpeed) * math.max(0f, maxSpeed);
        if (maxSpeedSq > 0f && math.lengthsq(velocity) > maxSpeedSq)
            velocity = math.normalizesafe(velocity) * math.sqrt(maxSpeedSq);
        return velocity;
    }
}
}
