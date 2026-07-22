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
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            // StartPosition 保存本 substep 的可信相对分离关系，不冻结实体位置。
            state.StartPosition = state.PredictedPosition;
            state.PreviousSubstepPosition = state.StartPosition;
            state.ContactPositionCorrection = float3.zero;
            state.WallPositionCorrection = float3.zero;

            float3 velocity = state.BasePredictedVelocity;
            float responseRate = math.max(0f, SoftAvoidanceResponseRate);
            if (state.IsSettled)
                responseRate *= math.max(0f, SettledSoftAvoidanceMultiplier);
            velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                velocity,
                state.SoftAvoidanceVelocity,
                responseRate,
                substepDeltaTime,
                state.MoveSpeed);
            if (state.IsSettled)
                velocity *= math.pow(0.8f, substepDeltaTime * 60f);

            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            state.PredictedPosition = state.StartPosition + velocity * substepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.UnconstrainedPredictedPosition = state.PredictedPosition;
            state.VelocityBeforeContact = velocity;
            state.IntegratedVelocity = velocity;
            States[i] = state;
        }
    }

    private void PrepareBaseVelocitiesForSubstep(float substepDeltaTime)
    {
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            state.BasePredictedVelocity = CalculateBaseVelocityForSubstep(
                state,
                substepDeltaTime);
            States[bodyIndex] = state;
        }
    }

    private float3 CalculateBaseVelocityForSubstep(
        FlowMovementFrameState state,
        float substepDeltaTime)
    {
        float3 totalForce = state.IndependentForce;
        if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 cellCenter = GridOrigin + new float3(
                state.CellPosition.x * CellRadius * 2 + CellRadius,
                state.CurrentPosition.y,
                state.CellPosition.y * CellRadius * 2 + CellRadius);
            float3 escapeDirection = state.PredictedPosition - cellCenter;
            escapeDirection.y = 0;
            escapeDirection = math.normalizesafe(escapeDirection, new float3(1, 0, 0));
            totalForce += escapeDirection * state.MoveSpeed * 5f;
        }

        if (math.lengthsq(totalForce) > state.MaxForce * state.MaxForce)
            totalForce = math.normalizesafe(totalForce) * state.MaxForce;

        return state.IntegratedVelocity + totalForce * substepDeltaTime;
    }

    private void ReconstructVelocities(
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics)
    {
        float speedBeforeSum = 0f;
        float speedAfterSum = 0f;
        int simulatedBodyCount = 0;

        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            state.IntegratedVelocity =
                (state.PredictedPosition - state.PreviousSubstepPosition) / substepDeltaTime;
            state.IntegratedVelocity.y = 0;
            float velocityChange = math.distance(
                state.IntegratedVelocity,
                state.VelocityBeforeContact);
            statistics.TotalVelocityChange += velocityChange;
            statistics.MaxVelocityChange = math.max(
                statistics.MaxVelocityChange,
                velocityChange);
            speedBeforeSum += math.length(state.VelocityBeforeContact);
            speedAfterSum += math.length(state.IntegratedVelocity);
            simulatedBodyCount++;
            States[i] = state;
        }

        if (simulatedBodyCount > 0)
        {
            statistics.AverageSpeedBeforeContact += speedBeforeSum / simulatedBodyCount;
            statistics.AverageSpeedAfterContact += speedAfterSum / simulatedBodyCount;
        }
    }
}
}
