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
            state.BasePredictedVelocity = velocity;
            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;
            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;
            CalculateIncrementalTightSweptBounds(
                state,
                out state.TimestepInteractionEnvelopeMin,
                out state.TimestepInteractionEnvelopeMax);
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
            CalculateIncrementalTightSweptBounds(
                state,
                out state.TimestepInteractionEnvelopeMin,
                out state.TimestepInteractionEnvelopeMax);
            state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }
    }

    private void BuildSubstepInteractionAndSoftViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        BuildSweptInteractionPairs(ref statistics);
        BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
        long elapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        incrementalStatistics.FullSweepSourceNanoseconds += elapsed;
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
    }

    private void BuildSubstepContactView(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        PreviousTimestepContactPairs.Clear();
        ClassifyTimestepContacts(
            ref statistics,
            ref incrementalStatistics,
            0);
        CommitTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            false);
        statistics.TimestepContactSetBuildNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
    }

    private bool BuildOrRefreshTimestepContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullBroadPhase,
        bool fallback,
        int scheduleStartSubstep = 0)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        PreviousTimestepContactPairs.Clear();
        if (fallback)
            PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());

        bool sourcedFromIncrementalCache = false;
        bool persistentViewReady = false;
        if (EnablePersistentContactCache)
        {
            sourcedFromIncrementalCache = BuildContactPairsFromPersistentNeighborSet(
                ref statistics,
                ref incrementalStatistics,
                forceFullBroadPhase,
                scheduleStartSubstep,
                out persistentViewReady);
        }
        else
        {
            long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptInteractionPairs(ref statistics);
            incrementalStatistics.FullSweepSourceNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - fullSweepStart);
        }

        if (persistentViewReady)
        {
            incrementalStatistics.CurrentInteractionPairCount =
                PersistentNeighborPairs.Length;
            ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
                ref incrementalStatistics);
            CommitTimestepContactViews(
                ref statistics,
                ref incrementalStatistics,
                fallback);
        }
        else
        {
            incrementalStatistics.CurrentInteractionPairCount =
                TimestepInteractionPairs.Length;
            BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
            ClassifyTimestepContacts(
                ref statistics,
                ref incrementalStatistics,
                scheduleStartSubstep);
            CommitTimestepContactViews(
                ref statistics,
                ref incrementalStatistics,
                fallback);
        }

        long elapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
        if (fallback)
        {
            statistics.TimestepContactSetFullRebuildCount++;
            statistics.TimestepContactSetFallbackNanoseconds += elapsed;
        }
        return sourcedFromIncrementalCache;
    }

    private void ClassifyTimestepContacts(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        Pairs.Clear();
        Pairs.AddRange(TimestepInteractionPairs.AsArray());
        int classificationCandidateCount = Pairs.Length;
        statistics.CandidatePairCount += classificationCandidateCount;
        incrementalStatistics.ReclassifiedPairEvaluationCount +=
            classificationCandidateCount;
        incrementalStatistics.SweptClassificationEvaluationCount +=
            classificationCandidateCount;
        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);

        long scheduleStart = ProfilerUnsafeUtility.Timestamp;
        BuildTimestepPredictiveSchedule(
            ref incrementalStatistics,
            scheduleStartSubstep);
        incrementalStatistics.ContactActivationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - scheduleStart);

    }

    private void CommitTimestepContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool fallback)
    {
        TimestepContactPairs.Clear();
        TimestepContactPairs.AddRange(Pairs.AsArray());
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount = TimestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount =
            incrementalStatistics.CurrentDormantPairCount;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            if (!fallback)
                continue;

            int previousIndex = FindPairIndex(PreviousTimestepContactPairs, pair.BodyA, pair.BodyB);
            if (previousIndex >= 0)
            {
                UnitCollisionPair previous = PreviousTimestepContactPairs[previousIndex];
                pair.WasActivatedThisTimestep = previous.WasActivatedThisTimestep;
                pair.WasCorrectedThisTimestep = previous.WasCorrectedThisTimestep;
                pair.FirstActivatedSubstep = previous.FirstActivatedSubstep;
                pair.ActivatedSubstepCount = previous.ActivatedSubstepCount;
                pair.WasAddedByFallback = previous.WasAddedByFallback;
            }
            else
            {
                pair.WasAddedByFallback = 1;
                statistics.TimestepContactSetFallbackAddedPairCount++;
            }
            TimestepContactPairs[pairIndex] = pair;
        }

        ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics);
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
        // Configuration.EnableDiagnostics is compile-time false in gameplay-only
        // builds. Keeping the guard inside the capture method lets Burst remove
        // both body and pair scans without splitting the solver call graph.
        if (!EnableDiagnostics)
            return;

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
