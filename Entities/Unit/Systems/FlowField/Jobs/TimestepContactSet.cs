using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private void PrepareTimestepContactPrediction(float duration, bool fromSolvedPosition)
    {
        duration = math.max(0f, duration);
        float margin = math.max(0f, TimestepContactMargin);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            float3 start = fromSolvedPosition
                ? state.PredictedPosition
                : state.CurrentPosition;
            float3 velocity = fromSolvedPosition
                ? state.BasePredictedVelocity
                : CalculateBaseVelocityForSubstep(state, duration);
            if (state.IsSettled)
                velocity *= math.pow(0.8f, duration * 60f);
            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            float3 end = start + velocity * duration;
            end.y = state.CurrentPosition.y;
            float extent = math.max(0f, state.Radius) + skin + margin;
            state.TimestepStartPosition = start;
            state.TimestepPredictedPosition = end;
            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;
            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;
            if (!fromSolvedPosition)
                state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }
    }

    private void PrepareSubstepContactPrediction()
    {
        float margin = math.max(0f, TimestepContactMargin);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            float3 start = state.StartPosition;
            float3 end = state.PredictedPosition;
            float extent = math.max(0f, state.Radius) + skin + margin;
            state.TimestepStartPosition = start;
            state.TimestepPredictedPosition = end;
            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;
            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;
            state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }
    }

    private bool BuildTimestepContactSet(
        ref PredictiveDiscContactStatistics statistics,
        ref ShadowNeighborCacheStatistics shadowStatistics,
        ref bool fatCachePairsMappedThisFrame,
        bool forceFullBroadPhase,
        bool fallback)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        MappedFatCachePairs.Clear();
        if (fallback)
            MappedFatCachePairs.AddRange(TimestepContactPairs.AsArray());

        bool sourcedFromFatCache = false;
        if (!forceFullBroadPhase && HasActiveAdaptiveFatRegions)
        {
            sourcedFromFatCache = BuildAdaptiveHybridContactPairs(
                ref statistics,
                ref shadowStatistics,
                ref fatCachePairsMappedThisFrame);
        }
        else
        {
            BuildSweptContactPairs(ref statistics);
        }

        TimestepContactPairs.Clear();
        TimestepContactPairs.AddRange(Pairs.AsArray());
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount = TimestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount = 0;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            if (pair.IsDormant != 0)
                statistics.TimestepContactSetDormantPairCount++;

            if (!fallback)
                continue;

            int previousIndex = FindPairIndex(MappedFatCachePairs, pair.BodyA, pair.BodyB);
            if (previousIndex >= 0)
            {
                UnitCollisionPair previous = MappedFatCachePairs[previousIndex];
                pair.WasActivatedThisTimestep = previous.WasActivatedThisTimestep;
                pair.FirstActivatedSubstep = previous.FirstActivatedSubstep;
                pair.ActivatedSubstepCount = previous.ActivatedSubstepCount;
            }
            else
            {
                pair.WasAddedByFallback = 1;
                statistics.TimestepContactSetFallbackAddedPairCount++;
            }
            TimestepContactPairs[pairIndex] = pair;
        }

        long elapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
        if (fallback)
        {
            statistics.TimestepContactSetFullRebuildCount++;
            statistics.TimestepContactSetFallbackNanoseconds += elapsed;
        }
        return sourcedFromFatCache;
    }

    private void ResetTimestepContactSetForSubstep()
    {
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            TimestepContactPairs[pairIndex] = pair;
        }
    }

    private bool AreCorrectedDiscsInsideTimestepEnvelope(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics)
    {
        bool allInside = true;
        float skin = math.max(0f, PredictiveSkin);
        for (int correctedIndex = 0; correctedIndex < CorrectedBodyIndices.Length; correctedIndex++)
        {
            int bodyIndex = CorrectedBodyIndices[correctedIndex];
            FlowMovementFrameState state = States[bodyIndex];
            float extent = math.max(0f, state.Radius) + skin;
            float2 currentMin = state.PredictedPosition.xz - extent;
            float2 currentMax = state.PredictedPosition.xz + extent;
            if (AabbContains(
                    state.TimestepEnvelopeMin,
                    state.TimestepEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;

            allInside = false;
            if (state.TimestepEscaped == 0)
            {
                state.TimestepEscaped = 1;
                statistics.TimestepContactSetEscapeBodyCount++;
                if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                    statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
                States[bodyIndex] = state;
            }
        }
        return allInside;
    }

    private void RebuildTimestepContactSetForRemainingTime(
        int substepIndex,
        int substepCount,
        float substepDeltaTime,
        bool persistAcrossSubsteps,
        ref PredictiveDiscContactStatistics statistics,
        ref ShadowNeighborCacheStatistics shadowStatistics,
        ref bool fatCachePairsMappedThisFrame)
    {
        float remainingDuration = persistAcrossSubsteps
            ? math.max(
                substepDeltaTime,
                (substepCount - substepIndex) * substepDeltaTime)
            : 0f;
        PrepareTimestepContactPrediction(remainingDuration, true);
        if (AdaptiveFatAabbRequested)
            InvalidateFatAabbCache(ref shadowStatistics, true);
        BuildTimestepContactSet(
            ref statistics,
            ref shadowStatistics,
            ref fatCachePairsMappedThisFrame,
            true,
            true);
        shadowStatistics.FullBroadPhaseFallbackCount++;
    }

    private static int FindPairIndex(
        Unity.Collections.NativeList<UnitCollisionPair> pairs,
        int bodyA,
        int bodyB)
    {
        int low = 0;
        int high = pairs.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            UnitCollisionPair candidate = pairs[middle];
            if (candidate.BodyA == bodyA && candidate.BodyB == bodyB)
                return middle;
            if (candidate.BodyA < bodyA ||
                (candidate.BodyA == bodyA && candidate.BodyB < bodyB))
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    private void BuildContactHeatSamples()
    {
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            HeatSamples[bodyIndex] = new Stage3ContactHeatSample
            {
                Entity = state.Entity,
                Position = state.PredictedPosition,
                ContactCorrection = math.length(state.TimestepContactCorrection),
                Escaped = state.TimestepEscaped
            };
        }

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            AccumulateHeatPair(pair.BodyA, pair);
            AccumulateHeatPair(pair.BodyB, pair);
        }
    }

    private void AccumulateHeatPair(int bodyIndex, UnitCollisionPair pair)
    {
        Stage3ContactHeatSample sample = HeatSamples[bodyIndex];
        sample.ContactPairDegree++;
        if (pair.WasActivatedThisTimestep != 0)
            sample.ActivePairDegree++;
        if (pair.ContactMode == UnitContactMode.Predictive)
            sample.PredictivePairDegree++;
        if (pair.WasAddedByFallback != 0)
            sample.HasFallbackPair = 1;
        HeatSamples[bodyIndex] = sample;
    }
}
}
