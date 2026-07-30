using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

using static RTS.Unit.FlowField.Jobs.CrowdContactPipelineScheduler;

namespace RTS.Unit.FlowField.Jobs
{

internal static class ParallelContactStageJobs
{
[BurstCompile]
internal struct PrepareTimestepPredictionBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdSolverBodyState> StepStates;
        [NativeDisableParallelForRestriction]
        public NativeArray<PersistentSweptProxy> PersistentProxies;
        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
        [ReadOnly] public NativeReference<IncrementalContactCacheState> PersistentCacheState;
        public NativeArray<byte> DirtyFlagsByBody;
        public float Duration;
        public float Skin;
        public float Margin;
        public float GuardMargin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public byte DetectPersistentDirty;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                if (DetectPersistentDirty != 0)
                    DirtyFlagsByBody[bodyIndex] = (byte)PersistentProxyBuilder.ClassifyAndUpdateForBody(
                        bodyIndex, stateSnapshot, stateEvidence, stateStep,
                        PersistentProxies, PersistentProxyIndexByBody,
                        PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                        SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
                return;
            }

            float3 start = FromSolvedPosition != 0
                ? stateStep.SolvedPosition
                : stateSnapshot.Position;
            float3 velocity = FromSolvedPosition != 0
                ? stateStep.BaseVelocity
                : ContactPipelineMath.CalculateBaseVelocity(stateSnapshot, stateNavigation, stateIntent, stateStep, Duration, GridOrigin, CellRadius);
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = stateSnapshot.Position.y;
            float extent = math.max(0f, stateSnapshot.Radius) + math.max(0f, Skin) + math.max(0f, Margin);
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateStep.BaseVelocity = velocity;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            ContactPipelineMath.CalculateInteractionBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                Skin,
                Margin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            if (FromSolvedPosition == 0)
                stateEvidence.EnvelopeEscaped = 0;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
            if (DetectPersistentDirty != 0)
                DirtyFlagsByBody[bodyIndex] = (byte)PersistentProxyBuilder.ClassifyAndUpdateForBody(
                    bodyIndex, stateSnapshot, stateEvidence, stateStep,
                        PersistentProxies, PersistentProxyIndexByBody,
                    PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                    SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
        }
    }

    [BurstCompile]
    internal struct PrepareSubstepContactPredictionBodiesJob :
        IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<byte> Workset;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        public NativeArray<CrowdSolverBodyState> StepStates;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float RvoTimeHorizon;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot body = Bodies[bodyIndex];
            if (body.IsInsideSimulationDomain == 0)
                return;

            CrowdMotionEvidence evidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState step = StepStates[bodyIndex];
            float extent =
                math.max(0f, body.Radius) +
                math.max(0f, PredictiveSkin) +
                math.max(0f, TimestepContactMargin);
            float3 start = step.SubstepStartPosition;
            float3 end = step.SolvedPosition;
            evidence.TrajectoryStart = start;
            evidence.BaselineEnd = end;
            evidence.ContactEnvelopeMin =
                math.min(start.xz, end.xz) - extent;
            evidence.ContactEnvelopeMax =
                math.max(start.xz, end.xz) + extent;
            PersistentContactMath.CalculateIncrementalTightSweptBounds(
                body,
                evidence,
                step,
                PredictiveSkin,
                TimestepContactMargin,
                SoftAvoidanceShell,
                RvoTimeHorizon,
                SoftSolverMode,
                out evidence.InteractionEnvelopeMin,
                out evidence.InteractionEnvelopeMax);
            evidence.EnvelopeEscaped = 0;
            MotionEvidence[bodyIndex] = evidence;
            StepStates[bodyIndex] = step;
        }
    }

    [BurstCompile]
    internal struct CountInitialDirtyBodyBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                count += DirtyFlagsByBody[bodyIndex] != 0 ? 1 : 0;
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    internal struct PrefixInitialDirtyBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public int BlockCount;
        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    internal struct ScatterInitialDirtyBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                IncrementalBodyDirtyFlags flags = (IncrementalBodyDirtyFlags)DirtyFlagsByBody[bodyIndex];
                if (flags == IncrementalBodyDirtyFlags.None)
                    continue;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody { BodyIndex = bodyIndex, Flags = flags };
            }
        }
    }

    [BurstCompile]
    internal struct PrepareBaseVelocityBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        public NativeArray<CrowdSolverBodyState> StepStates;
        public float SubstepDeltaTime;
        public float3 GridOrigin;
        public float CellRadius;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;
            stateStep.BaseVelocity = ContactPipelineMath.CalculateBaseVelocity(
                stateSnapshot,
                stateNavigation,
                stateIntent,
                stateStep,
                SubstepDeltaTime,
                GridOrigin,
                CellRadius);
            StepStates[bodyIndex] = stateStep;
        }
    }

    [BurstCompile]
    internal struct ValidateBaseMotionBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
        public NativeArray<byte> EscapeFlags;
        public byte Enabled;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;

        public void Execute(int bodyIndex)
        {
            if (Enabled == 0)
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            ContactPipelineMath.CalculateValidationBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                PredictiveSkin,
                TimestepContactMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out float2 min,
                out float2 max);
            EscapeFlags[bodyIndex] = (byte)(ContactPipelineMath.Contains(
                stateEvidence.InteractionEnvelopeMin,
                stateEvidence.InteractionEnvelopeMax,
                min,
                max) ? 0 : 1);
        }
    }

    [BurstCompile]
    internal struct CountEnvelopeEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public byte Enabled;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (Enabled != 0 && EscapeFlags[bodyIndex] != 0)
                    count++;
            }
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    internal struct PrefixEnvelopeEscapesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public NativeArray<byte> DirtyFlagsByBody;
        public int BlockCount;

        public void Execute()
        {
            for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)
            {
                int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
                if ((uint)bodyIndex < (uint)DirtyFlagsByBody.Length)
                    DirtyFlagsByBody[bodyIndex] = 0;
            }

            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    internal struct ScatterEnvelopeEscapesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<IncrementalDirtyBody> DirtyBodies;
        [NativeDisableParallelForRestriction]
        public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [NativeDisableParallelForRestriction]
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [NativeDisableParallelForRestriction]
        public NativeArray<CrowdSolverBodyState> StepStates;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (EscapeFlags[bodyIndex] == 0)
                    continue;

                const IncrementalBodyDirtyFlags flags =
                    IncrementalBodyDirtyFlags.Motion |
                    IncrementalBodyDirtyFlags.CorrectedEscape;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody
                {
                    BodyIndex = bodyIndex,
                    Flags = flags
                };
                DirtyFlagsByBody[bodyIndex] = (byte)flags;

                CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
                CrowdSolverBodyState stateStep = StepStates[bodyIndex];
                int newlyEscaped = stateEvidence.EnvelopeEscaped == 0 ? 1 : 0;
                stateEvidence.EnvelopeEscaped = 1;
                MotionEvidence[bodyIndex] = stateEvidence;
                StepStates[bodyIndex] = stateStep;

                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                body.EscapeCount = newlyEscaped;
                BodyStatistics[bodyIndex] = body;
            }
        }
    }



    [BurstCompile]
    internal struct PrepareRepairPredictionBodiesJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [NativeDisableParallelForRestriction]
        public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [NativeDisableParallelForRestriction]
        public NativeArray<CrowdSolverBodyState> StepStates;
        [ReadOnly] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public float Duration;
        public float Skin;
        public float Margin;
        public float3 GridOrigin;
        public float CellRadius;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;
        public byte Enabled;

        public void Execute(int dirtyIndex)
        {
            if (Enabled == 0)
                return;

            int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex >= (uint)Bodies.Length)
                return;
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;

            float3 start = stateStep.SolvedPosition;
            float3 velocity = stateStep.BaseVelocity;
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, Duration * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

            float3 end = start + velocity * Duration;
            end.y = stateSnapshot.Position.y;
            float extent = math.max(0f, stateSnapshot.Radius) +
                           math.max(0f, Skin) + math.max(0f, Margin);
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateStep.BaseVelocity = velocity;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            ContactPipelineMath.CalculateInteractionBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                Skin,
                Margin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftSolverMode,
                RvoTimeHorizon,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }







    [BurstCompile]
    internal struct InitializeSoftAvoidanceBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
        public NativeArray<CrowdAvoidanceState> AvoidanceStates;
        [ReadOnly] public NativeArray<CrowdObstacleCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public float SoftShell;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            CrowdAvoidanceState avoidance = default;
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                AvoidanceStates[bodyIndex] = avoidance;
                return;
            }

            int2 currentCell = FlowGridGeometry.WorldToCell(
                stateStep.SolvedPosition,
                GridOrigin,
                CellRadius);
            FlowGridGeometry obstacleGeometry = new FlowGridGeometry(
                GridOrigin, GridDimensions, CellRadius);
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (!GridObstacleView.IsBlocked(
                            Grid, obstacleGeometry, checkCell))
                        continue;
                    float3 wallPosition = GridObstacleView.CellCenter(
                        obstacleGeometry, checkCell, stateStep.SolvedPosition.y);
                    float wallRadius = CellRadius + math.max(0f, stateSnapshot.Radius) + math.max(0f, SoftShell);
                    avoidance.WallVelocity += SoftAvoidanceMath.CalculateWallVelocity(
                        stateStep.SolvedPosition,
                        wallPosition,
                        stateSnapshot.MoveSpeed,
                        wallRadius);
                }
            }
            AvoidanceStates[bodyIndex] = avoidance;
        }
    }

    [BurstCompile]
    internal struct EvaluateSoftAvoidancePairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
        [ReadOnly] public NativeArray<BodyPair> Pairs;
        public NativeArray<SoftAvoidancePairContribution> Contributions;
        public SoftAvoidanceVelocitySolverMode SolverMode;
        public float SoftShell;
        public float RvoTimeHorizon;
        public float SubstepDeltaTime;

        public void Execute(int pairIndex)
        {
            BodyPair pair = Pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdSolverBodyState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdSolverBodyState bodyBStep = StepStates[pair.BodyB];
            SoftAvoidancePairContribution result = default;

            float3 delta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
            delta.y = 0f;
            float maxDistance = bodyASnapshot.Radius + bodyBSnapshot.Radius + math.max(0f, SoftShell);
            if (math.lengthsq(delta) <= maxDistance * maxDistance &&
                SoftAvoidanceMath.TryCalculatePairVelocities(
                    SolverMode,
                    bodyAStep.SolvedPosition,
                    bodyBStep.SolvedPosition,
                    bodyAStep.BaseVelocity,
                    bodyBStep.BaseVelocity,
                    bodyASnapshot.Radius,
                    bodyBSnapshot.Radius,
                    bodyASnapshot.InverseMass,
                    bodyBSnapshot.InverseMass,
                    bodyASnapshot.MoveSpeed,
                    bodyBSnapshot.MoveSpeed,
                    SoftShell,
                    RvoTimeHorizon,
                    SubstepDeltaTime,
                    ContactPipelineMath.DeterministicPairNormal(pair.BodyA, pair.BodyB),
                    out result.VelocityA,
                    out result.VelocityB))
            {
                result.ActiveA = 1;
                result.ActiveB = 1;
            }
            Contributions[pairIndex] = result;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct ReduceSoftAvoidanceBlocksJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        public NativeArray<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * SoftPairBatchSize;
            int end = math.min(begin + SoftPairBatchSize, Contributions.Length);
            int active = 0;
            for (int i = begin; i < end; i++)
                active += Contributions[i].ActiveA != 0 ? 1 : 0;
            Blocks[blockIndex] = new JacobiBlockTelemetry
            {
                NewlyActivatedPairCount = active
            };
        }
    }

#endif

    [BurstCompile]
    internal struct GatherSoftAvoidanceBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
        public NativeArray<CrowdAvoidanceState> AvoidanceStates;
        [ReadOnly] public NativeArray<BodyPair> Pairs;
        [ReadOnly] public NativeArray<SoftAvoidancePairContribution> Contributions;
        [ReadOnly] public NativeArray<int> IncidentOffsets;
        [ReadOnly] public NativeArray<int> IncidentPairIndices;
        public NativeArray<byte> EscapeFlags;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftShell;
        public byte ClampToEnvelope;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            CrowdAvoidanceState avoidance = AvoidanceStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }

            float3 sum = float3.zero;
            int count = 0;
            int begin = IncidentOffsets[bodyIndex];
            int end = IncidentOffsets[bodyIndex + 1];
            for (int incident = begin; incident < end; incident++)
            {
                int pairIndex = IncidentPairIndices[incident];
                BodyPair pair = Pairs[pairIndex];
                SoftAvoidancePairContribution contribution = Contributions[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    sum += contribution.VelocityA;
                    count++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    sum += contribution.VelocityB;
                    count++;
                }
            }

            if (count > 0 && SoftSolverMode == SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer)
                sum /= count;
            avoidance.SoftVelocity = sum + avoidance.WallVelocity;
            avoidance.NeighborCount = count;
            float maxSpeed = math.max(0f, stateSnapshot.MoveSpeed);
            if (math.lengthsq(avoidance.SoftVelocity) >
                maxSpeed * maxSpeed)
            {
                avoidance.SoftVelocity =
                    math.normalizesafe(avoidance.SoftVelocity) * maxSpeed;
            }

            byte escaped = 0;
            if (ClampToEnvelope != 0)
            {
                float3 requested = avoidance.SoftVelocity;
                if (!ContactPipelineMath.SoftOutputInsideEnvelope(
                        stateSnapshot,
                        stateNavigation,
                        stateEvidence,
                        stateStep,
                        requested,
                        SoftAvoidanceResponseRate,
                        SettledMultiplier,
                        SubstepDeltaTime,
                        PredictiveSkin,
                        TimestepContactMargin,
                        SoftShell))
                {
                    float lower = 0f;
                    float upper = 1f;
                    if (ContactPipelineMath.SoftOutputInsideEnvelope(
                            stateSnapshot,
                            stateNavigation,
                            stateEvidence,
                            stateStep,
                            float3.zero,
                            SoftAvoidanceResponseRate,
                            SettledMultiplier,
                            SubstepDeltaTime,
                            PredictiveSkin,
                            TimestepContactMargin,
                            SoftShell))
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            float middle = (lower + upper) * 0.5f;
                            if (ContactPipelineMath.SoftOutputInsideEnvelope(
                                    stateSnapshot,
                                    stateNavigation,
                                    stateEvidence,
                                    stateStep,
                                    requested * middle,
                                    SoftAvoidanceResponseRate,
                                    SettledMultiplier,
                                    SubstepDeltaTime,
                                    PredictiveSkin,
                                    TimestepContactMargin,
                                    SoftShell))
                                lower = middle;
                            else
                                upper = middle;
                        }
                    }
                    avoidance.SoftVelocity = requested * lower;
                    escaped = 1;
                }
            }
            EscapeFlags[bodyIndex] = escaped;
            AvoidanceStates[bodyIndex] = avoidance;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct ReduceSoftEscapeBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> EscapeFlags;
        public NativeArray<int> EscapeCountsByBlock;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int escaped = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                escaped += EscapeFlags[bodyIndex] != 0 ? 1 : 0;
            EscapeCountsByBlock[blockIndex] = escaped;
        }
    }



#endif

    [BurstCompile]
    internal struct PredictUnconstrainedBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdAvoidanceState> AvoidanceStates;
        public NativeArray<CrowdSolverBodyState> StepStates;
        public float SoftAvoidanceResponseRate;
        public float SettledMultiplier;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            CrowdAvoidanceState avoidance = AvoidanceStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                return;
            stateStep.SubstepStartPosition = stateStep.SolvedPosition;
            stateStep.PreviousSubstepPosition = stateStep.SubstepStartPosition;
            stateStep.ContactCorrection = float3.zero;
            stateStep.WallCorrection = float3.zero;
            float response = math.max(0f, SoftAvoidanceResponseRate);
            if ((stateNavigation.IsSettled != 0))
                response *= math.max(0f, SettledMultiplier);
            float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                stateStep.BaseVelocity,
                avoidance.SoftVelocity,
                response,
                SubstepDeltaTime,
                stateSnapshot.MoveSpeed);
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, SubstepDeltaTime * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;
            stateStep.SolvedPosition = stateStep.SubstepStartPosition + velocity * SubstepDeltaTime;
            stateStep.SolvedPosition.y = stateSnapshot.Position.y;
            stateStep.UnconstrainedPosition = stateStep.SolvedPosition;
            stateStep.VelocityBeforeContact = velocity;
            stateStep.IntegratedVelocity = velocity;
            StepStates[bodyIndex] = stateStep;
        }
    }

    [BurstCompile]
    internal struct ValidatePredictedContactEnvelopeBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
        public NativeArray<byte> EscapeFlags;
        public float PredictiveSkin;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                EscapeFlags[bodyIndex] = 0;
                return;
            }
            float extent = math.max(0f, stateSnapshot.Radius) + math.max(0f, PredictiveSkin);
            EscapeFlags[bodyIndex] = (byte)(ContactPipelineMath.Contains(
                stateEvidence.ContactEnvelopeMin,
                stateEvidence.ContactEnvelopeMax,
                stateStep.SolvedPosition.xz - extent,
                stateStep.SolvedPosition.xz + extent) ? 0 : 1);
        }
    }



    [BurstCompile]
    internal struct ResetContactPairStateJob : IJobParallelForDefer
    {
        public NativeArray<ContactConstraint> Pairs;
        public void Execute(int pairIndex)
        {
            ContactConstraint pair = Pairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            Pairs[pairIndex] = pair;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct CountParallelSimulationDebuggerPairBlocksJob :
        IJobParallelForDefer
    {
        [ReadOnly] public NativeList<ParallelSimulationDebuggerPairCapture>
            Candidates;
        public NativeList<JacobiBlockTelemetry> Blocks;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int selectedPairCount = 0;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                selectedPairCount += Candidates[pairIndex].IsValid != 0 ? 1 : 0;
            }

            JacobiBlockTelemetry block = Blocks[blockIndex];
            block.SelectedPairCount = selectedPairCount;
            Blocks[blockIndex] = block;
        }
    }

    [BurstCompile]
    internal struct PrefixParallelSimulationDebuggerPairsJob : IJob
    {
        public NativeList<JacobiBlockTelemetry> Blocks;
        public NativeList<SimulationDebuggerPairSample> Scratch;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
            {
                JacobiBlockTelemetry block = Blocks[blockIndex];
                block.SelectedPairOffset = offset;
                offset += block.SelectedPairCount;
                Blocks[blockIndex] = block;
            }
            Scratch.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    internal struct ScatterParallelSimulationDebuggerPairsJob : IJobParallelForDefer
    {
        [ReadOnly] public NativeList<ParallelSimulationDebuggerPairCapture>
            Candidates;
        [ReadOnly] public NativeList<JacobiBlockTelemetry> Blocks;
        [NativeDisableParallelForRestriction]
        public NativeList<SimulationDebuggerPairSample> Scratch;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * JacobiPairBatchSize;
            int end = math.min(begin + JacobiPairBatchSize, Candidates.Length);
            int writeIndex = Blocks[blockIndex].SelectedPairOffset;
            for (int pairIndex = begin; pairIndex < end; pairIndex++)
            {
                ParallelSimulationDebuggerPairCapture candidate =
                    Candidates[pairIndex];
                if (candidate.IsValid == 0)
                    continue;
                Scratch[writeIndex++] = candidate.Sample;
            }
        }
    }



#endif



    [BurstCompile]
    internal struct SolveWallConstraintBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdSolverBodyState> StepStates;
        [ReadOnly] public NativeArray<CrowdObstacleCell> Grid;
        public float3 GridOrigin;
        public int2 GridDimensions;
        public float CellRadius;
        public NativeArray<byte> CorrectedBodyFlags;
        public NativeArray<ParallelBodyStageResult> BodyStatistics;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            float total = 0f;
            float maximum = 0f;
            int corrected = 0;
            if (Grid.IsCreated && (stateSnapshot.IsInsideSimulationDomain != 0) && stateSnapshot.InverseMass > 0f)
            {
                int2 currentCell = FlowGridGeometry.WorldToCell(
                    stateStep.SolvedPosition,
                    GridOrigin,
                    CellRadius);
                FlowGridGeometry obstacleGeometry = new FlowGridGeometry(
                    GridOrigin, GridDimensions, CellRadius);
                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        int2 checkCell = currentCell + new int2(x, y);
                        if (!GridObstacleView.IsBlocked(
                                Grid, obstacleGeometry, checkCell))
                            continue;
                        float3 wallPosition = GridObstacleView.CellCenter(
                            obstacleGeometry, checkCell, stateStep.SolvedPosition.y);
                        float3 delta = stateStep.SolvedPosition - wallPosition;
                        delta.y = 0f;
                        float distance = math.length(delta);
                        float hardDistance = CellRadius + math.max(0f, stateSnapshot.Radius);
                        if (distance >= hardDistance)
                            continue;
                        float3 normal = distance > 0.00001f
                            ? delta / distance
                            : ContactPipelineMath.DeterministicPairNormal(
                                bodyIndex,
                                obstacleGeometry.FlatIndex(checkCell));
                        float3 correction = normal * ((hardDistance - distance) * 0.5f);
                        stateStep.SolvedPosition += correction;
                        stateStep.SolvedPosition.y = stateSnapshot.Position.y;
                        stateStep.WallCorrection += correction;
                        stateStep.TimestepWallCorrection += correction;
                        float length = math.length(correction);
                        total += length;
                        maximum = math.max(maximum, length);
                        corrected = 1;
                    }
                }
            }
            StepStates[bodyIndex] = stateStep;
            CorrectedBodyFlags[bodyIndex] = (byte)corrected;
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
            {
                Total = total,
                Maximum = maximum,
                Count = corrected
            };
        }
    }

    [BurstCompile]
    internal struct CountAndReduceWallBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int correctedCount = 0;
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                correctedCount += CorrectedBodyFlags[bodyIndex] != 0 ? 1 : 0;
            }

            BlockOffsetsAndCounts[blockIndex] = correctedCount;
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }

    [BurstCompile]
    internal struct PrefixCorrectedBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<int> CorrectedBodyIndices;
        public int BlockCount;

        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            CorrectedBodyIndices.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    internal struct ScatterCorrectedBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> CorrectedBodyFlags;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction]
        public NativeArray<int> CorrectedBodyIndices;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                if (CorrectedBodyFlags[bodyIndex] != 0)
                    CorrectedBodyIndices[writeIndex++] = bodyIndex;
            }
        }
    }



#if RTS_CONTACT_DIAGNOSTICS


#endif

    [BurstCompile]
    internal struct ReconstructVelocityBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        public NativeArray<CrowdSolverBodyState> StepStates;
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public float SubstepDeltaTime;

        public void Execute(int bodyIndex)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            {
                BodyStatistics[bodyIndex] = default;
                return;
            }
            stateStep.IntegratedVelocity =
                (stateStep.SolvedPosition - stateStep.PreviousSubstepPosition) / SubstepDeltaTime;
            stateStep.IntegratedVelocity.y = 0f;
            float change = math.distance(stateStep.IntegratedVelocity, stateStep.VelocityBeforeContact);
            BodyStatistics[bodyIndex] = new ParallelBodyStageResult
            {
                Total = change,
                Maximum = change,
                SecondaryTotal = math.length(stateStep.VelocityBeforeContact),
                TertiaryTotal = math.length(stateStep.IntegratedVelocity),
                Count = 1
            };
            StepStates[bodyIndex] = stateStep;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    [BurstCompile]
    internal struct ReduceVelocityBodyBlocksJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction]
        public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public int BodyCount;

        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            ParallelBodyStageResult aggregate = default;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                ParallelBodyStageResult body = BodyStatistics[bodyIndex];
                aggregate.Total += body.Total;
                aggregate.Maximum = math.max(aggregate.Maximum, body.Maximum);
                aggregate.SecondaryTotal += body.SecondaryTotal;
                aggregate.TertiaryTotal += body.TertiaryTotal;
                aggregate.Count += body.Count;
            }
            if (begin < BodyCount)
                BodyStatistics[begin] = aggregate;
        }
    }



#endif





















#if RTS_CONTACT_DIAGNOSTICS


#endif







#if RTS_CONTACT_DIAGNOSTICS




#endif
}
}
