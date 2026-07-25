using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Frame-local motion prediction and post-solve velocity reconstruction.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void PredictUnconstrainedPositions(float substepDeltaTime)
    {
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot body = Bodies[bodyIndex];
            if (body.IsInsideSimulationDomain == 0)
                continue;

            CrowdNavigationState navigation = NavigationStates[bodyIndex];
            CrowdBodyStepState step = StepStates[bodyIndex];

            step.SubstepStartPosition = step.SolvedPosition;
            step.PreviousSubstepPosition = step.SubstepStartPosition;
            step.ContactCorrection = float3.zero;
            step.WallCorrection = float3.zero;

            float3 velocity = step.BaseVelocity;
            float responseRate = math.max(0f, SoftAvoidanceResponseRate);
            if (navigation.IsSettled != 0)
                responseRate *= math.max(0f, SettledSoftAvoidanceMultiplier);
            velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                velocity,
                step.SoftAvoidanceVelocity,
                responseRate,
                substepDeltaTime,
                body.MoveSpeed);
            if (navigation.IsSettled != 0)
                velocity *= math.pow(0.8f, substepDeltaTime * 60f);

            if (math.lengthsq(velocity) > body.MoveSpeed * body.MoveSpeed)
                velocity = math.normalizesafe(velocity) * body.MoveSpeed;

            step.SolvedPosition = step.SubstepStartPosition + velocity * substepDeltaTime;
            step.SolvedPosition.y = body.Position.y;
            step.UnconstrainedPosition = step.SolvedPosition;
            step.VelocityBeforeContact = velocity;
            step.IntegratedVelocity = velocity;
            StepStates[bodyIndex] = step;
        }
    }

    private void PrepareBaseVelocitiesForSubstep(float substepDeltaTime)
    {
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot body = Bodies[bodyIndex];
            if (body.IsInsideSimulationDomain == 0)
                continue;

            CrowdNavigationState navigation = NavigationStates[bodyIndex];
            CrowdMotionIntent intent = MotionIntents[bodyIndex];
            CrowdBodyStepState step = StepStates[bodyIndex];
            step.BaseVelocity = CalculateBaseVelocityForSubstep(
                body,
                navigation,
                intent,
                step,
                substepDeltaTime);
            StepStates[bodyIndex] = step;
        }
    }

    private float3 CalculateBaseVelocityForSubstep(
        CrowdBodySnapshot body,
        CrowdNavigationState navigation,
        CrowdMotionIntent intent,
        CrowdBodyStepState step,
        float substepDeltaTime)
    {
        float3 steeringVelocityError = intent.SteeringVelocityError;
        if (IsObstacleCell(navigation.Cell) &&
            math.lengthsq(steeringVelocityError) < 0.1f)
        {
            float3 cellCenter = ObstacleCellCenter(navigation.Cell, body.Position.y);
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

    private void ReconstructVelocities(
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics)
    {
        float speedBeforeSum = 0f;
        float speedAfterSum = 0f;
        int simulatedBodyCount = 0;

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            if (Bodies[bodyIndex].IsInsideSimulationDomain == 0)
                continue;

            CrowdBodyStepState step = StepStates[bodyIndex];
            step.IntegratedVelocity =
                (step.SolvedPosition - step.PreviousSubstepPosition) /
                substepDeltaTime;
            step.IntegratedVelocity.y = 0f;
            float velocityChange = math.distance(
                step.IntegratedVelocity,
                step.VelocityBeforeContact);
            statistics.TotalVelocityChange += velocityChange;
            statistics.MaxVelocityChange = math.max(
                statistics.MaxVelocityChange,
                velocityChange);
            speedBeforeSum += math.length(step.VelocityBeforeContact);
            speedAfterSum += math.length(step.IntegratedVelocity);
            simulatedBodyCount++;
            StepStates[bodyIndex] = step;
        }

        if (simulatedBodyCount > 0)
        {
            statistics.AverageSpeedBeforeContact += speedBeforeSum / simulatedBodyCount;
            statistics.AverageSpeedAfterContact += speedAfterSum / simulatedBodyCount;
        }
    }
}
}
