using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 管线各阶段共享的简洁确定性数学辅助函数。
/// </summary>
public static class ContactPipelineMath
{
    public static float3 DeterministicFallbackNormal(int bodyA, int bodyB)
    {
        uint hash = math.hash(new int2(bodyA, bodyB));
        return (hash & 1u) == 0u
            ? new float3(1, 0, 0)
            : new float3(0, 0, 1);
    }

    internal static long TimestampToNanoseconds(long timestampDelta)
    {
        long denominator = ProfilerUnsafeUtility.TimestampToNanosecondsConversionRatio.Denominator;
        long numerator = ProfilerUnsafeUtility.TimestampToNanosecondsConversionRatio.Numerator;
        return denominator > 0 ? timestampDelta * numerator / denominator : 0L;
    }

    internal static float3 CalculateBaseVelocityForSubstep(
        NativeArray<FlowFieldCell> grid,
        FlowGridGeometry geometry,
        CrowdBodySnapshot body,
        CrowdNavigationState navigation,
        CrowdMotionIntent intent,
        CrowdBodyStepState step,
        float substepDeltaTime)
    {
        float3 steeringVelocityError = intent.SteeringVelocityError;
        if (GridObstacleView.IsBlocked(grid, geometry, navigation.Cell) &&
            math.lengthsq(steeringVelocityError) < 0.1f)
        {
            float3 cellCenter = GridObstacleView.CellCenter(
                geometry,
                navigation.Cell,
                body.Position.y);
            float3 escapeDirection = step.SolvedPosition - cellCenter;
            escapeDirection.y = 0f;
            escapeDirection = math.normalizesafe(
                escapeDirection,
                new float3(1f, 0f, 0f));
            steeringVelocityError += escapeDirection * body.MoveSpeed * 5f;
        }

        if (math.lengthsq(steeringVelocityError) >
            body.MaxAcceleration * body.MaxAcceleration)
        {
            steeringVelocityError = math.normalizesafe(steeringVelocityError) *
                                    body.MaxAcceleration;
        }

        return step.IntegratedVelocity +
               steeringVelocityError * substepDeltaTime;
    }
    internal static float3 CalculateBaseVelocity(
        CrowdBodySnapshot snapshot,
        CrowdNavigationState navigation,
        CrowdMotionIntent intent,
        CrowdBodyStepState step,
        float deltaTime,
        float3 gridOrigin,
        float cellRadius)
    {
        float3 totalForce = intent.SteeringVelocityError;
        if (navigation.IsBlocked != 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 center = gridOrigin + new float3(
                navigation.Cell.x * cellRadius * 2f + cellRadius,
                snapshot.Position.y,
                navigation.Cell.y * cellRadius * 2f + cellRadius);
            float3 escape = step.SolvedPosition - center;
            escape.y = 0f;
            totalForce += math.normalizesafe(escape, new float3(1f, 0f, 0f)) *
                          snapshot.MoveSpeed * 5f;
        }
        if (math.lengthsq(totalForce) > snapshot.MaxAcceleration * snapshot.MaxAcceleration)
            totalForce = math.normalizesafe(totalForce) * snapshot.MaxAcceleration;
        return step.IntegratedVelocity + totalForce * deltaTime;
    }

    internal static bool Contains(float2 outerMin, float2 outerMax, float2 innerMin, float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    internal static float3 DeterministicPairNormal(int a, int b)
    {
        return ContactPipelineMath.DeterministicFallbackNormal(a, b);
    }

    internal static void CalculateInteractionBounds(
        CrowdBodySnapshot snapshot,
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        float predictiveSkin,
        float margin,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        ContactPipelineMath.CalculatePathBounds(
            evidence,
            step,
            softShell,
            softResponseRate,
            softSolverMode,
            rvoTimeHorizon,
            out float2 pathMin,
            out float2 pathMax);
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, margin) * 2f;
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, snapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        min = pathMin - extent;
        max = pathMax + extent;
    }

    internal static void CalculateValidationBounds(
        CrowdBodySnapshot snapshot,
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        float predictiveSkin,
        float margin,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        ContactPipelineMath.CalculatePathBounds(
            evidence,
            step,
            softShell,
            softResponseRate,
            softSolverMode,
            rvoTimeHorizon,
            out float2 pathMin,
            out float2 pathMax);
        float contactPadding = math.max(0f, predictiveSkin) + math.max(0f, margin);
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, snapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        min = pathMin - extent;
        max = pathMax + extent;
    }

    internal static void CalculatePathBounds(
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        float softShell,
        float softResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon,
        out float2 min,
        out float2 max)
    {
        min = math.min(
            evidence.TrajectoryStart.xz,
            math.min(
                evidence.BaselineEnd.xz,
                math.min(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        max = math.max(
            evidence.TrajectoryStart.xz,
            math.max(
                evidence.BaselineEnd.xz,
                math.max(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        if (softSolverMode != SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            softShell <= 0f || softResponseRate <= 0f)
            return;
        float2 horizonEnd = step.SolvedPosition.xz +
                            step.BaseVelocity.xz * math.max(0f, rvoTimeHorizon);
        min = math.min(min, horizonEnd);
        max = math.max(max, horizonEnd);
    }

    internal static bool SoftOutputInsideEnvelope(
        CrowdBodySnapshot snapshot,
        CrowdNavigationState navigation,
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        float3 avoidance,
        float responseRate,
        float settledMultiplier,
        float deltaTime,
        float predictiveSkin,
        float margin,
        float softShell)
    {
        float response = math.max(0f, responseRate);
        if ((navigation.IsSettled != 0))
            response *= math.max(0f, settledMultiplier);
        float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
            step.BaseVelocity,
            avoidance,
            response,
            deltaTime,
            snapshot.MoveSpeed);
        if ((navigation.IsSettled != 0))
            velocity *= math.pow(0.8f, deltaTime * 60f);
        if (math.lengthsq(velocity) > snapshot.MoveSpeed * snapshot.MoveSpeed)
            velocity = math.normalizesafe(velocity) * snapshot.MoveSpeed;
        float3 end = step.SolvedPosition + velocity * deltaTime;
        float contactPadding = math.max(0f, predictiveSkin) + math.max(0f, margin);
        float avoidancePadding = math.max(0f, softShell) * 0.5f;
        float extent = math.max(0f, snapshot.Radius) + math.max(contactPadding, avoidancePadding);
        return ContactPipelineMath.Contains(
            evidence.InteractionEnvelopeMin,
            evidence.InteractionEnvelopeMax,
            end.xz - extent,
            end.xz + extent);
    }
}
}
