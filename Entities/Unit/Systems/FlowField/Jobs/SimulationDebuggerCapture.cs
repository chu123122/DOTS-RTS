using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask;
    public int SimulationDebuggerMaximumPairs;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
    public NativeReference<SimulationDebuggerUnitSample> SimulationDebuggerSelectedUnit;
    public NativeReference<byte> SimulationDebuggerSelectedUnitValid;

    private bool CaptureSelectedSimulationDebuggerData =>
        DiagnosticSelectedEntity != Entity.Null &&
        (SimulationDebuggerCaptureMask & SimulationDebuggerCaptureMask.SelectedUnit) != 0;

    private bool CaptureSelectedSimulationDebuggerPairs =>
        CaptureSelectedSimulationDebuggerData &&
        (SimulationDebuggerCaptureMask & SimulationDebuggerCaptureMask.SelectedPairs) != 0;

    private void ResetSimulationDebuggerCapture()
    {
        SimulationDebuggerSelectedPairs.Clear();
        SimulationDebuggerSelectedUnit.Value = default;
        SimulationDebuggerSelectedUnitValid.Value = 0;
    }

    private void CaptureSimulationDebuggerPair(
        int substepIndex,
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        float3 normal,
        float constraintValue,
        float pairCorrection)
    {
        if (!CaptureSelectedSimulationDebuggerPairs ||
            (bodyA.Entity != DiagnosticSelectedEntity &&
             bodyB.Entity != DiagnosticSelectedEntity))
            return;

        int sampleIndex = FindSimulationDebuggerPair(pair.BodyA, pair.BodyB);
        bool isNew = sampleIndex < 0;
        if (isNew)
        {
            int maximumPairs = math.max(1, SimulationDebuggerMaximumPairs);
            if (SimulationDebuggerSelectedPairs.Length >= maximumPairs)
                return;
            sampleIndex = SimulationDebuggerSelectedPairs.Length;
            SimulationDebuggerSelectedPairs.Add(new SimulationDebuggerPairSample
            {
                BodyA = pair.BodyA,
                BodyB = pair.BodyB,
                EntityA = bodyA.Entity,
                EntityB = bodyB.Entity,
                GeneratedSubstep = substepIndex,
                FirstActivatedSubstep = -1,
                LastActivatedSubstep = -1,
                StartSeparation = CalculateStartSeparation(pair, bodyA, bodyB),
                Kind = pair.ContactMode == UnitContactMode.Predictive
                    ? SimulationDebuggerPairKind.PredictiveContact
                    : SimulationDebuggerPairKind.ActualContact
            });
        }

        SimulationDebuggerPairSample sample = SimulationDebuggerSelectedPairs[sampleIndex];
        sample.PositionA = bodyA.PredictedPosition;
        sample.PositionB = bodyB.PredictedPosition;
        sample.ReferenceNormal = normal;
        sample.CurrentSeparation = constraintValue;
        sample.Lambda = pair.Lambda;
        sample.TotalCorrection += pairCorrection;
        sample.State = pair.WasActivated != 0
            ? SimulationDebuggerPairState.Active
            : SimulationDebuggerPairState.CachedInactive;
        if (pair.WasActivated != 0)
        {
            if (sample.FirstActivatedSubstep < 0)
                sample.FirstActivatedSubstep = substepIndex;
            sample.LastActivatedSubstep = substepIndex;
        }
        SimulationDebuggerSelectedPairs[sampleIndex] = sample;
    }

    private void CaptureSimulationDebuggerSelectedUnit()
    {
        if (!CaptureSelectedSimulationDebuggerData)
            return;

        int bodyIndex = -1;
        for (int i = 0; i < States.Length; i++)
        {
            if (States[i].Entity == DiagnosticSelectedEntity)
            {
                bodyIndex = i;
                break;
            }
        }
        if (bodyIndex < 0)
            return;

        FlowMovementFrameState state = States[bodyIndex];
        int cachedContacts = 0;
        int activeContacts = 0;
        for (int i = 0; i < SimulationDebuggerSelectedPairs.Length; i++)
        {
            SimulationDebuggerPairSample pair = SimulationDebuggerSelectedPairs[i];
            if (pair.BodyA != bodyIndex && pair.BodyB != bodyIndex)
                continue;
            cachedContacts++;
            if (pair.State == SimulationDebuggerPairState.Active)
                activeContacts++;
        }

        var sample = new SimulationDebuggerUnitSample
        {
            Entity = state.Entity,
            BodyIndex = bodyIndex,
            CurrentPosition = state.CurrentPosition,
            TimestepStartPosition = state.StartPosition,
            UnconstrainedPosition = state.UnconstrainedPredictedPosition,
            FinalPosition = state.PredictedPosition,
            CurrentVelocity = state.CurrentVelocity,
            SoftAvoidanceVelocity = state.SoftAvoidanceVelocity,
            ContactCorrection = math.length(state.ContactPositionCorrection.xz),
            WallCorrection = math.length(state.WallPositionCorrection.xz),
            SoftNeighborCount = state.SoftAvoidanceNeighborCount,
            CandidatePairCount = cachedContacts,
            CachedContactCount = cachedContacts,
            ActiveContactCount = activeContacts
        };

        for (int i = 0; i < AdaptiveDebugProxies.Length; i++)
        {
            AdaptiveFatAabbDebugProxy proxy = AdaptiveDebugProxies[i];
            if (proxy.Entity != state.Entity)
                continue;
            sample.SweptMin = proxy.CoreMin;
            sample.SweptMax = proxy.CoreMax;
            sample.FatMin = proxy.FatMin;
            sample.FatMax = proxy.FatMax;
            sample.HasFatBounds = 1;
            break;
        }

        SimulationDebuggerSelectedUnit.Value = sample;
        SimulationDebuggerSelectedUnitValid.Value = 1;
    }

    private int FindSimulationDebuggerPair(int bodyA, int bodyB)
    {
        for (int i = 0; i < SimulationDebuggerSelectedPairs.Length; i++)
        {
            SimulationDebuggerPairSample sample = SimulationDebuggerSelectedPairs[i];
            if (sample.BodyA == bodyA && sample.BodyB == bodyB)
                return i;
        }
        return -1;
    }

    private static float CalculateStartSeparation(
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB)
    {
        float3 delta = bodyA.StartPosition - bodyB.StartPosition;
        delta.y = 0f;
        float radiusSum = bodyA.Radius + bodyB.Radius;
        if (pair.ContactMode == UnitContactMode.Predictive)
        {
            float3 normal = math.normalizesafe(
                delta,
                DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
            return math.dot(delta, normal) - radiusSum;
        }
        return math.length(delta) - radiusSum;
    }
}
}
