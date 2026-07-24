using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct ParallelSimulationDebuggerPairCapture
{
    public SimulationDebuggerPairSample Sample;
    public byte IsValid;
}

public partial struct SolveXpbdUnitContactsJob
{
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask;
    public int SimulationDebuggerMaximumPairs;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture>
        ParallelSimulationDebuggerPairCandidates;
    public NativeList<SimulationDebuggerPairSample>
        ParallelSimulationDebuggerPairScratch;
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

        MergeSimulationDebuggerPairSample(BuildSimulationDebuggerPairSample(
            substepIndex,
            pair,
            bodyA,
            bodyB,
            normal,
            constraintValue,
            pairCorrection));
    }

    private static SimulationDebuggerPairSample BuildSimulationDebuggerPairSample(
        int substepIndex,
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        float3 normal,
        float constraintValue,
        float pairCorrection)
    {
        bool active = pair.WasActivated != 0;
        return new SimulationDebuggerPairSample
        {
            BodyA = pair.BodyA,
            BodyB = pair.BodyB,
            EntityA = bodyA.Entity,
            EntityB = bodyB.Entity,
            GeneratedSubstep = substepIndex,
            FirstActivatedSubstep = active ? substepIndex : -1,
            LastActivatedSubstep = active ? substepIndex : -1,
            StartSeparation = CalculateStartSeparation(pair, bodyA, bodyB),
            Kind = pair.ContactMode == UnitContactMode.Predictive
                ? SimulationDebuggerPairKind.PredictiveContact
                : SimulationDebuggerPairKind.ActualContact,
            PositionA = bodyA.PredictedPosition,
            PositionB = bodyB.PredictedPosition,
            ReferenceNormal = normal,
            CurrentSeparation = constraintValue,
            Lambda = pair.Lambda,
            TotalCorrection = pairCorrection,
            State = active
                ? SimulationDebuggerPairState.Active
                : SimulationDebuggerPairState.CachedInactive
        };
    }

    private void MergeSimulationDebuggerPairSample(
        SimulationDebuggerPairSample candidate)
    {
        int sampleIndex = FindSimulationDebuggerPair(
            candidate.BodyA,
            candidate.BodyB);
        if (sampleIndex < 0)
        {
            int maximumPairs = math.max(1, SimulationDebuggerMaximumPairs);
            if (SimulationDebuggerSelectedPairs.Length >= maximumPairs)
                return;
            SimulationDebuggerSelectedPairs.Add(candidate);
            return;
        }

        SimulationDebuggerPairSample sample =
            SimulationDebuggerSelectedPairs[sampleIndex];
        sample.PositionA = candidate.PositionA;
        sample.PositionB = candidate.PositionB;
        sample.ReferenceNormal = candidate.ReferenceNormal;
        sample.CurrentSeparation = candidate.CurrentSeparation;
        sample.Lambda = candidate.Lambda;
        sample.TotalCorrection += candidate.TotalCorrection;
        sample.State = candidate.State;
        if (candidate.State == SimulationDebuggerPairState.Active)
        {
            if (sample.FirstActivatedSubstep < 0)
                sample.FirstActivatedSubstep = candidate.FirstActivatedSubstep;
            sample.LastActivatedSubstep = candidate.LastActivatedSubstep;
        }
        SimulationDebuggerSelectedPairs[sampleIndex] = sample;
    }

    private void MergeParallelSimulationDebuggerPairScratch()
    {
        for (int i = 0; i < ParallelSimulationDebuggerPairScratch.Length; i++)
        {
            MergeSimulationDebuggerPairSample(
                ParallelSimulationDebuggerPairScratch[i]);
        }
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
            TimestepStartPosition = state.CurrentPosition,
            UnconstrainedPosition = state.UnconstrainedPredictedPosition,
            FinalPosition = state.PredictedPosition,
            CurrentVelocity = state.CurrentVelocity,
            SoftAvoidanceVelocity = state.SoftAvoidanceVelocity,
            ContactCorrection = math.length(state.ContactPositionCorrection.xz),
            WallCorrection = math.length(state.WallPositionCorrection.xz),
            SoftNeighborCount = state.SoftAvoidanceNeighborCount,
            CapturedPairCount = cachedContacts,
            CachedContactCount = cachedContacts,
            ActiveContactCount = activeContacts
        };

        if (EnablePersistentContactCache &&
            TryFindPersistentProxy(
                state.Entity,
                out PersistentSweptProxy persistentProxy) &&
            persistentProxy.IsValid != 0)
        {
            sample.SweptMin = persistentProxy.TightMin;
            sample.SweptMax = persistentProxy.TightMax;
            sample.FatMin = persistentProxy.GuardMin;
            sample.FatMax = persistentProxy.GuardMax;
            sample.HasFatBounds = 1;
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
