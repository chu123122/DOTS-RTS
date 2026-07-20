using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

/// <summary>
/// 单位动态接触 XPBD 求解器。
/// 生命周期与 MiniClothLab PointConstraints 一致：每个 substep 预测位置、清空 lambda、
/// 多轮投影约束，最后通过位置差回写速度。
/// </summary>
[BurstCompile]
public struct SolveXpbdUnitContactsJob : IJob
{
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public float Compliance;

    public float3 GridOrigin;
    public float CellRadius;

    public NativeArray<UnitCollisionPair> Pairs;
    [ReadOnly] public NativeArray<float2> CollisionFootprints;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute()
    {
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        float substepDeltaTime = DeltaTime / substepCount;
        if (substepDeltaTime <= 0f)
            return;

        InitializeSolverState();

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            PredictPositions(substepDeltaTime);
            ResetContactLambdas();

            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
                SolveContactIteration(substepDeltaTime);

            ReconstructVelocities(substepDeltaTime);
        }
    }

    private void InitializeSolverState()
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            state.IntegratedVelocity = state.IsInsideGrid ? state.CurrentVelocity : float3.zero;
            state.PredictedPosition = state.CurrentPosition;
            state.PreviousSubstepPosition = state.CurrentPosition;
            state.PositionCorrection = float3.zero;
            States[i] = state;
        }
    }

    private void PredictPositions(float substepDeltaTime)
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            float3 totalForce = state.IndependentForce + state.SoftAvoidanceForce;
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

            float3 velocity = state.IntegratedVelocity + totalForce * substepDeltaTime;
            if (state.IsSettled)
                velocity *= math.pow(0.8f, substepDeltaTime * 60f);

            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            state.PreviousSubstepPosition = state.PredictedPosition;
            state.PredictedPosition += velocity * substepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.IntegratedVelocity = velocity;
            States[i] = state;
        }
    }

    private void ResetContactLambdas()
    {
        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            pair.Lambda = 0f;
            Pairs[i] = pair;
        }
    }

    private void SolveContactIteration(float substepDeltaTime)
    {
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);

        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            float inverseMassA = bodyA.InverseMass;
            float inverseMassB = bodyB.InverseMass;
            float denominator = inverseMassA + inverseMassB + alpha;
            if (denominator <= 0f)
                continue;

            float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            delta.y = 0;
            float distance = math.length(delta);
            float3 normal = distance > 0.00001f
                ? delta / distance
                : DeterministicFallbackNormal(pair.BodyA, pair.BodyB);

            float radiusA = math.cmax(CollisionFootprints[pair.BodyA]) * 0.5f;
            float radiusB = math.cmax(CollisionFootprints[pair.BodyB]) * 0.5f;
            float constraintValue = distance - (radiusA + radiusB);

            float deltaLambda = -(constraintValue + alpha * pair.Lambda) / denominator;
            float nextLambda = math.max(0f, pair.Lambda + deltaLambda);
            float appliedLambda = nextLambda - pair.Lambda;
            pair.Lambda = nextLambda;
            Pairs[i] = pair;
            if (math.abs(appliedLambda) <= 0.0000001f)
                continue;

            bodyA.PredictedPosition += normal * (inverseMassA * appliedLambda);
            bodyB.PredictedPosition -= normal * (inverseMassB * appliedLambda);
            bodyA.PredictedPosition.y = bodyA.CurrentPosition.y;
            bodyB.PredictedPosition.y = bodyB.CurrentPosition.y;

            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;
        }
    }

    private void ReconstructVelocities(float substepDeltaTime)
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            state.IntegratedVelocity =
                (state.PredictedPosition - state.PreviousSubstepPosition) / substepDeltaTime;
            state.IntegratedVelocity.y = 0;
            States[i] = state;
        }
    }

    private static float3 DeterministicFallbackNormal(int bodyA, int bodyB)
    {
        uint hash = math.hash(new int2(bodyA, bodyB));
        return (hash & 1u) == 0u
            ? new float3(1, 0, 0)
            : new float3(0, 0, 1);
    }
}
