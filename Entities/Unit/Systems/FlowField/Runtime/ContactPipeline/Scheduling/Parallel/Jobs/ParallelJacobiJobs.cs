using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

using static RTS.Unit.FlowField.Jobs.CrowdContactPipelineScheduler;

namespace RTS.Unit.FlowField.Jobs
{

internal static class ParallelJacobiJobs
{
internal struct JacobiPairSolveResult
    {
        public ContactConstraint Pair;
        public JacobiPairCorrection Correction;
        public ContactConstraintEvaluation Evaluation;
    }

    internal static JacobiPairSolveResult EvaluateJacobiPair(
        int substepIndex,
        float alpha,
        ContactConstraint pair,
        CrowdBodySnapshot bodyA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdBodyStepState stepB)
    {
        ContactConstraintEvaluation evaluation = XpbdContactConstraintMath.Evaluate(
            ref pair,
            bodyA,
            stepA,
            bodyB,
            stepB,
            alpha,
            substepIndex);

        JacobiPairCorrection correction = default;
        correction.NewlyActivated = evaluation.NewlyActivated;
        correction.PairCorrection = evaluation.PairCorrection;
        if (math.abs(evaluation.AppliedLambda) > 0.0000001f)
        {
            if (pair.WasCorrectedThisTimestep == 0)
            {
                pair.WasCorrectedThisTimestep = 1;
                correction.NewlyCorrected = 1;
            }
            if (bodyA.InverseMass > 0f)
            {
                correction.DeltaA = evaluation.Normal *
                                    (bodyA.InverseMass * evaluation.AppliedLambda);
                correction.ActiveA = 1;
            }
            if (bodyB.InverseMass > 0f)
            {
                correction.DeltaB = -evaluation.Normal *
                                    (bodyB.InverseMass * evaluation.AppliedLambda);
                correction.ActiveB = 1;
            }
        }

        return new JacobiPairSolveResult
        {
            Pair = pair,
            Correction = correction,
            Evaluation = evaluation
        };
    }









    [BurstCompile]
    internal struct EvaluateParallelJacobiPairsJob : IJobParallelForDefer
    {
        public float Alpha;
        public int SubstepIndex;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        public NativeArray<ContactConstraint> Pairs;
        // 并行 job 的按索引写目标必须是 deferred NativeArray（或
        // [NativeDisableParallelForRestriction]）。NativeList 不是原子写容器，
        // 以读写形式喂给 IJobParallelFor 会在 Schedule 阶段被 Jobs 安全检查拒绝
        //（"not declared [ReadOnly] ... does not support parallel writing"）。
        // 长度由 FinalizeP1P6WallIteration 的 resize 与接触对数对齐，运行时由
        // AsDeferredJobArray 从 list length patch，和上方 Pairs 同模式。
        public NativeArray<JacobiPairCorrection> Corrections;

        public void Execute(int pairIndex)
        {
            ContactConstraint pair = Pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            JacobiPairSolveResult result = EvaluateJacobiPair(
                SubstepIndex,
                Alpha,
                pair,
                bodyASnapshot,
                bodyAStep,
                bodyBSnapshot,
                bodyBStep);

            Pairs[pairIndex] = result.Pair;
            Corrections[pairIndex] = result.Correction;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct EvaluateParallelJacobiPairsWithDiagnosticsJob :
        IJobParallelForDefer
    {
        public float Alpha;
        public int SubstepIndex;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        public NativeArray<ContactConstraint> Pairs;
        public NativeArray<JacobiPairCorrection> Corrections;
        public NativeArray<ParallelSimulationDebuggerPairCapture>
            DiagnosticPairCandidates;
        public Entity DiagnosticSelectedEntity;

        public void Execute(int pairIndex)
        {
            ContactConstraint pair = Pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            JacobiPairSolveResult result = EvaluateJacobiPair(
                SubstepIndex,
                Alpha,
                pair,
                bodyASnapshot,
                bodyAStep,
                bodyBSnapshot,
                bodyBStep);

            Pairs[pairIndex] = result.Pair;
            Corrections[pairIndex] = result.Correction;

            ParallelSimulationDebuggerPairCapture capture = default;
            if (bodyASnapshot.Entity == DiagnosticSelectedEntity ||
                bodyBSnapshot.Entity == DiagnosticSelectedEntity)
            {
                capture.IsValid = 1;
                capture.Sample =
                    ConstraintSolverJob.BuildSimulationDebuggerPairSample(
                        SubstepIndex,
                        result.Pair,
                        bodyASnapshot,
                        bodyAStep,
                        bodyBSnapshot,
                        bodyBStep,
                        result.Evaluation.Normal,
                        result.Evaluation.ConstraintValue,
                        result.Evaluation.PairCorrection);
            }
            DiagnosticPairCandidates[pairIndex] = capture;
        }
    }
#endif

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct ReduceParallelJacobiBlocksJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeList<JacobiPairCorrection> Corrections;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Corrections.Length);
            JacobiBlockTelemetry block = default;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                JacobiPairCorrection correction = Corrections[pairIndex];
                if (correction.ActiveA != 0 || correction.ActiveB != 0)
                {
                    block.TotalPositionCorrection += correction.PairCorrection;
                    block.MaxPositionCorrection = math.max(
                        block.MaxPositionCorrection,
                        correction.PairCorrection);
                }
                block.NewlyActivatedPairCount += correction.NewlyActivated;
                block.NewlyCorrectedPairCount += correction.NewlyCorrected;
            }
            Blocks[blockIndex] = block;
        }
    }

#endif

    [BurstCompile]
    internal struct GatherAndApplyParallelJacobiBodiesJob : IJobParallelFor
    {
        public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdNavigationState> NavigationStates;
        public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeList<ContactConstraint> Pairs;
        [ReadOnly] public NativeList<JacobiPairCorrection> Corrections;
        [ReadOnly] public NativeArray<int> IncidentOffsets;
        [ReadOnly] public NativeList<int> IncidentPairIndices;
        public NativeArray<byte> CorrectedBodyFlags;

        public void Execute(int bodyIndex)
        {
            float3 correctionSum = float3.zero;
            int correctionCount = 0;
            int begin = IncidentOffsets[bodyIndex];
            int end = IncidentOffsets[bodyIndex + 1];
            // The incident list is rebuilt by ActiveConstraintIncidentIndexBuilder.Ensure
            // with exactly 2*Pairs.Length entries. If end exceeds the live list length
            // the offsets and the list are out of sync (deferred-write timing). Clamp +
            // assert-free guard: this Execute must never read past the list, but a
            // positive end>length here is a real bug to surface upstream.
            int listLength = IncidentPairIndices.Length;
            if (end > listLength)
            {
                end = listLength;
                CorrectedBodyFlags[bodyIndex] = 2; // sentinel: incident index desync
            }
            for (int incidentIndex = begin; incidentIndex < end; incidentIndex++)
            {
                int pairIndex = IncidentPairIndices[incidentIndex];
                ContactConstraint pair = Pairs[pairIndex];
                JacobiPairCorrection contribution = Corrections[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    correctionSum += contribution.DeltaA;
                    correctionCount++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    correctionSum += contribution.DeltaB;
                    correctionCount++;
                }
            }

            if (correctionCount <= 0)
                return;

            CrowdBodySnapshot bodySnapshot = Bodies[bodyIndex];
            CrowdNavigationState bodyNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent bodyIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence bodyEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState bodyStep = StepStates[bodyIndex];
            float3 correction = correctionSum / correctionCount;
            bodyStep.SolvedPosition += correction;
            bodyStep.ContactCorrection += correction;
            bodyEvidence.ContactCorrection += correction;
            bodyStep.SolvedPosition.y = bodySnapshot.Position.y;
            Bodies[bodyIndex] = bodySnapshot;
            NavigationStates[bodyIndex] = bodyNavigation;
            MotionIntents[bodyIndex] = bodyIntent;
            MotionEvidence[bodyIndex] = bodyEvidence;
            StepStates[bodyIndex] = bodyStep;
            CorrectedBodyFlags[bodyIndex] = 1;
        }
    }
}
}
