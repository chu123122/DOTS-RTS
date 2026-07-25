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

public partial struct ConstraintSolverJob
{


    private bool CaptureSelectedSimulationDebuggerData =>
        EnableDiagnostics && DiagnosticSelectedEntity != Entity.Null &&
        (SimulationDebuggerCaptureMask & SimulationDebuggerCaptureMask.SelectedUnit) != 0;

    private bool CaptureSelectedSimulationDebuggerPairs =>
        CaptureSelectedSimulationDebuggerData &&
        (SimulationDebuggerCaptureMask & SimulationDebuggerCaptureMask.SelectedPairs) != 0;

    private void ResetSimulationDebuggerCapture()
    {
        if (!EnableDiagnostics) return;
        SimulationDebuggerSelectedPairs.Clear();
        SimulationDebuggerSelectedUnit.Value = default;
        SimulationDebuggerSelectedUnitValid.Value = 0;
    }

    private void CaptureSimulationDebuggerPair(
        int substepIndex,
        ContactConstraint pair,
        CrowdBodySnapshot bodyA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdBodyStepState stepB,
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
            stepA,
            bodyB,
            stepB,
            normal,
            constraintValue,
            pairCorrection));
    }

    internal static SimulationDebuggerPairSample BuildSimulationDebuggerPairSample(
        int substepIndex,
        ContactConstraint pair,
        CrowdBodySnapshot bodyA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdBodyStepState stepB,
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
            StartSeparation = CalculateStartSeparation(pair, bodyA, stepA, bodyB, stepB),
            Kind = pair.ContactMode == ContactConstraintMode.Predictive
                ? SimulationDebuggerPairKind.PredictiveContact
                : SimulationDebuggerPairKind.ActualContact,
            PositionA = stepA.SolvedPosition,
            PositionB = stepB.SolvedPosition,
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
        if (!EnableDiagnostics) return;
        for (int i = 0; i < ParallelSimulationDebuggerPairScratch.Length; i++)
        {
            MergeSimulationDebuggerPairSample(
                ParallelSimulationDebuggerPairScratch[i]);
        }
    }

    private void CaptureSimulationDebuggerSelectedUnit()
    {
        if (!EnableDiagnostics || !CaptureSelectedSimulationDebuggerData)
            return;

        int bodyIndex = -1;
        for (int i = 0; i < Bodies.Length; i++)
        {
            if (Bodies[i].Entity == DiagnosticSelectedEntity)
            {
                bodyIndex = i;
                break;
            }
        }
        if (bodyIndex < 0)
            return;

        CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
        CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
        CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
        CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
        CrowdBodyStepState stateStep = StepStates[bodyIndex];
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
            Entity = stateSnapshot.Entity,
            BodyIndex = bodyIndex,
            CurrentPosition = stateSnapshot.Position,
            TimestepStartPosition = stateEvidence.TrajectoryStart,
            UnconstrainedPosition = stateStep.UnconstrainedPosition,
            FinalPosition = stateStep.SolvedPosition,
            CurrentVelocity = stateSnapshot.Velocity,
            SoftAvoidanceVelocity = stateStep.SoftAvoidanceVelocity,
            ContactCorrection = math.length(stateStep.ContactCorrection.xz),
            WallCorrection = math.length(stateStep.WallCorrection.xz),
            SoftNeighborCount = stateStep.SoftAvoidanceNeighborCount,
            CapturedPairCount = cachedContacts,
            CachedContactCount = cachedContacts,
            ActiveContactCount = activeContacts
        };

        sample.SweptMin = stateEvidence.ContactEnvelopeMin;
        sample.SweptMax = stateEvidence.ContactEnvelopeMax;
        sample.FatMin = stateEvidence.InteractionEnvelopeMin;
        sample.FatMax = stateEvidence.InteractionEnvelopeMax;
        sample.HasFatBounds = 1;

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
        ContactConstraint pair,
        CrowdBodySnapshot bodyA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdBodyStepState stepB)
    {
        float3 delta = stepA.SubstepStartPosition - stepB.SubstepStartPosition;
        delta.y = 0f;
        float radiusSum = bodyA.Radius + bodyB.Radius;
        if (pair.ContactMode == ContactConstraintMode.Predictive)
        {
            float3 normal = math.normalizesafe(
                delta,
                ContactPipelineMath.DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
            return math.dot(delta, normal) - radiusSum;
        }
        return math.length(delta) - radiusSum;
    }
}
}
