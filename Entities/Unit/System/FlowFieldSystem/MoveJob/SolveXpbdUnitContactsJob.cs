using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;

/// <summary>
/// Frostbite-inspired Predictive Disc Contact 求解器。
/// 每个 substep 保存可信起始构型、预测无约束终点、生成 swept disc Pair，
/// 随后全部 XPBD iteration 复用同一份 Pair，不在 iteration 内重复 Broad/Narrow Phase。
/// </summary>
[BurstCompile]
public struct SolveXpbdUnitContactsJob : IJob
{
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public float Compliance;
    public float PredictiveSkin;
    public bool EnablePredictiveContacts;
    public bool EnableDiagnostics;
    public bool EnableShadowNeighborCacheTest;
    public float ShadowCacheMargin;
    public Entity DiagnosticSelectedEntity;

    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<UnitCollisionPair> Pairs;
    public NativeList<SweptDiscCellEntry> ShadowCellEntries;
    public NativeList<UnitCollisionPair> ShadowBodyPairs;
    public NativeList<ShadowFatBodyProxy> ShadowCurrentProxies;
    public NativeList<ShadowEntityPair> ShadowCurrentPairs;
    public NativeList<ShadowFatBodyProxy> ShadowPreviousProxies;
    public NativeList<ShadowEntityPair> ShadowPreviousPairs;
    public NativeArray<FlowMovementFrameState> States;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<ShadowNeighborCacheStatistics> ShadowStatistics;
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;

    public void Execute()
    {
        long solverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        float substepDeltaTime = DeltaTime / substepCount;
        var statistics = new PredictiveDiscContactStatistics();
        var shadowStatistics = new ShadowNeighborCacheStatistics();
        float penetrationSum = 0f;
        IterationDiagnostics.Clear();
        PairDiagnostics.Clear();
        SelectedBodyDiagnostic.Value = default;

        if (substepDeltaTime <= 0f)
        {
            Statistics.Value = statistics;
            ShadowStatistics.Value = shadowStatistics;
            return;
        }

        if (!EnableShadowNeighborCacheTest)
        {
            ShadowPreviousProxies.Clear();
            ShadowPreviousPairs.Clear();
            ShadowCurrentProxies.Clear();
            ShadowCurrentPairs.Clear();
        }
        else
        {
            shadowStatistics.PreviousFrameCacheAvailable =
                (byte)(ShadowPreviousProxies.Length > 0 ? 1 : 0);
            shadowStatistics.PreviousFrameCacheBodyCount = ShadowPreviousProxies.Length;
            shadowStatistics.PreviousFrameCachePairCount = ShadowPreviousPairs.Length;
        }

        InitializeSolverState();

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            PredictUnconstrainedPositions(substepDeltaTime);

            long pairGenerationStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptContactPairs(ref statistics);
            statistics.PairGenerationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - pairGenerationStart);

            if (EnableShadowNeighborCacheTest && substepIndex == 0)
            {
                long shadowBuildStart = ProfilerUnsafeUtility.Timestamp;
                BuildCurrentShadowCache(ref shadowStatistics);
                shadowStatistics.CacheBuildNanoseconds +=
                    TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - shadowBuildStart);
            }

            long iterationStart = ProfilerUnsafeUtility.Timestamp;
            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                SolveWallConstraintIteration(
                    out float totalWallPositionCorrection,
                    out float maxWallPositionCorrection);
                SolveContactIteration(
                    substepDeltaTime,
                    out float totalPositionCorrection,
                    out float maxPositionCorrection);

                statistics.TotalContactPositionCorrection += totalPositionCorrection;
                statistics.MaxContactPositionCorrection = math.max(
                    statistics.MaxContactPositionCorrection,
                    maxPositionCorrection);
                statistics.TotalWallPositionCorrection += totalWallPositionCorrection;
                statistics.MaxWallPositionCorrection = math.max(
                    statistics.MaxWallPositionCorrection,
                    maxWallPositionCorrection);

                if (EnableDiagnostics)
                {
                    RecordIterationDiagnostic(
                        substepIndex,
                        iterationIndex,
                        totalPositionCorrection,
                        maxPositionCorrection,
                        totalWallPositionCorrection,
                        maxWallPositionCorrection);
                }
            }
            statistics.IterationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - iterationStart);

            AccumulateConstraintStatistics(ref statistics, ref penetrationSum);
            ReconstructVelocities(substepDeltaTime, ref statistics);

            if (EnableShadowNeighborCacheTest)
            {
                long shadowValidationStart = ProfilerUnsafeUtility.Timestamp;
                if (substepIndex == 0 && shadowStatistics.PreviousFrameCacheAvailable != 0)
                {
                    ValidateShadowReference(
                        ShadowPreviousProxies,
                        ShadowPreviousPairs,
                        true,
                        true,
                        ref shadowStatistics);
                }

                ValidateShadowReference(
                    ShadowCurrentProxies,
                    ShadowCurrentPairs,
                    false,
                    substepIndex > 0,
                    ref shadowStatistics);
                shadowStatistics.ValidationNanoseconds +=
                    TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - shadowValidationStart);
            }

            if (EnableDiagnostics)
                CaptureSelectedBodyAndPairs(substepIndex);
        }

        if (EnableShadowNeighborCacheTest)
            PromoteCurrentShadowCache();

        statistics.AveragePenetration = statistics.PenetratingPairCount > 0
            ? penetrationSum / statistics.PenetratingPairCount
            : 0f;
        statistics.UnactivatedPairCount =
            statistics.ContactPairCount - statistics.ActiveConstraintCount;
        statistics.PredictiveUnactivatedCount =
            statistics.PredictivePairCount - statistics.PredictiveActivatedCount;
        statistics.UnactivatedRatio = statistics.ContactPairCount > 0
            ? (float)statistics.UnactivatedPairCount / statistics.ContactPairCount
            : 0f;
        statistics.PredictiveUnactivatedRatio = statistics.PredictivePairCount > 0
            ? (float)statistics.PredictiveUnactivatedCount / statistics.PredictivePairCount
            : 0f;
        statistics.AverageIterationNanoseconds =
            statistics.IterationNanoseconds / math.max(1, substepCount * iterationCount);
        statistics.AverageSpeedBeforeContact /= substepCount;
        statistics.AverageSpeedAfterContact /= substepCount;
        statistics.SolverNanoseconds =
            TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - solverStartTimestamp);
        Statistics.Value = statistics;
        ShadowStatistics.Value = shadowStatistics;
    }

    private void InitializeSolverState()
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            state.IntegratedVelocity = state.IsInsideGrid ? state.CurrentVelocity : float3.zero;
            state.StartPosition = state.CurrentPosition;
            state.PredictedPosition = state.CurrentPosition;
            state.PreviousSubstepPosition = state.CurrentPosition;
            state.ContactPositionCorrection = float3.zero;
            state.WallPositionCorrection = float3.zero;
            States[i] = state;
        }
    }

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

            float3 totalForce = state.IndependentForce + state.SoftAvoidanceForce;
            if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
            {
                float3 cellCenter = GridOrigin + new float3(
                    state.CellPosition.x * CellRadius * 2 + CellRadius,
                    state.CurrentPosition.y,
                    state.CellPosition.y * CellRadius * 2 + CellRadius);
                float3 escapeDirection = state.StartPosition - cellCenter;
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

            state.PredictedPosition = state.StartPosition + velocity * substepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.UnconstrainedPredictedPosition = state.PredictedPosition;
            state.VelocityBeforeContact = velocity;
            state.IntegratedVelocity = velocity;
            States[i] = state;
        }
    }

    private void BuildSweptContactPairs(ref PredictiveDiscContactStatistics statistics)
    {
        SweptCellEntries.Clear();
        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            float sweptExtent = math.max(0f, state.Radius) + skin;
            float2 sweptMin = math.min(state.StartPosition.xz, state.PredictedPosition.xz) - sweptExtent;
            float2 sweptMax = math.max(state.StartPosition.xz, state.PredictedPosition.xz) + sweptExtent;
            int2 minCell = (int2)math.floor((sweptMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((sweptMax - GridOrigin.xz) / cellSize);

            if (maxCell.x < 0 || maxCell.y < 0 ||
                minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
                continue;

            minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
            maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);

            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    SweptCellEntries.Add(new SweptDiscCellEntry
                    {
                        CellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), GridDimensions),
                        BodyIndex = bodyIndex
                    });
                }
            }
        }

        SweptCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
        EmitCellPairs();
        SortAndDeduplicatePairs();
        statistics.CandidatePairCount += Pairs.Length;
        FilterAndClassifyPairs(ref statistics, skin);
    }

    private void BuildCurrentShadowCache(ref ShadowNeighborCacheStatistics statistics)
    {
        ShadowCellEntries.Clear();
        ShadowBodyPairs.Clear();
        ShadowCurrentProxies.Clear();
        ShadowCurrentPairs.Clear();

        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float extentMargin = math.max(0f, PredictiveSkin) + math.max(0f, ShadowCacheMargin);
        int validBodyCount = 0;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            var proxy = new ShadowFatBodyProxy { Entity = state.Entity };
            if (!state.IsInsideGrid)
            {
                ShadowCurrentProxies.Add(proxy);
                continue;
            }

            float extent = math.max(0f, state.Radius) + extentMargin;
            proxy.FatMin = math.min(state.StartPosition.xz, state.UnconstrainedPredictedPosition.xz) - extent;
            proxy.FatMax = math.max(state.StartPosition.xz, state.UnconstrainedPredictedPosition.xz) + extent;
            proxy.IsValid = 1;
            ShadowCurrentProxies.Add(proxy);
            validBodyCount++;

            int2 minCell = (int2)math.floor((proxy.FatMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((proxy.FatMax - GridOrigin.xz) / cellSize);
            if (maxCell.x < 0 || maxCell.y < 0 ||
                minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
                continue;

            minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
            maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);
            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    ShadowCellEntries.Add(new SweptDiscCellEntry
                    {
                        CellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), GridDimensions),
                        BodyIndex = bodyIndex
                    });
                }
            }
        }

        ShadowCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
        int cellStart = 0;
        while (cellStart < ShadowCellEntries.Length)
        {
            int cellIndex = ShadowCellEntries[cellStart].CellIndex;
            int cellEnd = cellStart + 1;
            while (cellEnd < ShadowCellEntries.Length &&
                   ShadowCellEntries[cellEnd].CellIndex == cellIndex)
                cellEnd++;

            for (int first = cellStart; first < cellEnd; first++)
            {
                int bodyA = ShadowCellEntries[first].BodyIndex;
                for (int second = first + 1; second < cellEnd; second++)
                {
                    int bodyB = ShadowCellEntries[second].BodyIndex;
                    if (bodyA == bodyB)
                        continue;
                    ShadowBodyPairs.Add(new UnitCollisionPair
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    });
                }
            }

            cellStart = cellEnd;
        }

        SortAndDeduplicateBodyPairs(ShadowBodyPairs);
        for (int i = 0; i < ShadowBodyPairs.Length; i++)
        {
            UnitCollisionPair bodyPair = ShadowBodyPairs[i];
            ShadowFatBodyProxy proxyA = ShadowCurrentProxies[bodyPair.BodyA];
            ShadowFatBodyProxy proxyB = ShadowCurrentProxies[bodyPair.BodyB];
            if (proxyA.IsValid == 0 || proxyB.IsValid == 0 ||
                !AabbOverlaps(proxyA.FatMin, proxyA.FatMax, proxyB.FatMin, proxyB.FatMax))
                continue;

            ShadowCurrentPairs.Add(ShadowEntityOrdering.CreatePair(proxyA.Entity, proxyB.Entity));
        }

        SortAndDeduplicateEntityPairs(ShadowCurrentPairs);
        ShadowCurrentProxies.AsArray().Sort(new ShadowFatBodyProxyComparer());
        statistics.CurrentFrameCacheBodyCount = validBodyCount;
        statistics.CurrentFrameCachePairCount = ShadowCurrentPairs.Length;
    }

    private static void SortAndDeduplicateBodyPairs(NativeList<UnitCollisionPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new UnitCollisionPairComparer());
        int writeIndex = 1;
        UnitCollisionPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            UnitCollisionPair current = pairs[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private static void SortAndDeduplicateEntityPairs(NativeList<ShadowEntityPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new ShadowEntityPairComparer());
        int writeIndex = 1;
        ShadowEntityPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            ShadowEntityPair current = pairs[readIndex];
            if (current.EntityA == previous.EntityA && current.EntityB == previous.EntityB)
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private void ValidateShadowReference(
        NativeList<ShadowFatBodyProxy> referenceProxies,
        NativeList<ShadowEntityPair> referencePairs,
        bool isPreviousFrame,
        bool countPairCoverage,
        ref ShadowNeighborCacheStatistics statistics)
    {
        int authoritativePairs = 0;
        int pairHits = 0;
        int pairMisses = 0;
        int activePairMisses = 0;
        int predictivePairMisses = 0;
        int preSolveEscapes = 0;
        int finalEscapes = 0;
        int contactDrivenEscapes = 0;
        int wallDrivenEscapes = 0;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            bool hasProxy = TryFindProxy(referenceProxies, state.Entity, out ShadowFatBodyProxy proxy);
            float coreExtent = math.max(0f, state.Radius) + math.max(0f, PredictiveSkin);
            float2 coreMin = math.min(state.StartPosition.xz, state.UnconstrainedPredictedPosition.xz) - coreExtent;
            float2 coreMax = math.max(state.StartPosition.xz, state.UnconstrainedPredictedPosition.xz) + coreExtent;
            bool preSolveEscaped = !hasProxy || !AabbContains(proxy.FatMin, proxy.FatMax, coreMin, coreMax);
            if (preSolveEscaped)
                preSolveEscapes++;

            float2 finalMin = state.PredictedPosition.xz - coreExtent;
            float2 finalMax = state.PredictedPosition.xz + coreExtent;
            bool finalEscaped = !hasProxy || !AabbContains(proxy.FatMin, proxy.FatMax, finalMin, finalMax);
            if (!finalEscaped)
                continue;

            finalEscapes++;
            if (math.lengthsq(state.ContactPositionCorrection) > 0.0000001f)
                contactDrivenEscapes++;
            if (math.lengthsq(state.WallPositionCorrection) > 0.0000001f)
                wallDrivenEscapes++;
        }

        if (countPairCoverage)
        {
            for (int i = 0; i < Pairs.Length; i++)
            {
                UnitCollisionPair pair = Pairs[i];
                ShadowEntityPair entityPair = ShadowEntityOrdering.CreatePair(
                    States[pair.BodyA].Entity,
                    States[pair.BodyB].Entity);
                bool hit = ContainsPair(referencePairs, entityPair);
                authoritativePairs++;
                if (hit)
                {
                    pairHits++;
                    continue;
                }

                pairMisses++;
                if (pair.WasActivated != 0)
                    activePairMisses++;
                if (pair.ContactMode == UnitContactMode.Predictive)
                    predictivePairMisses++;
            }
        }

        if (isPreviousFrame)
        {
            if (countPairCoverage)
                statistics.PreviousFrameCheckCount++;
            statistics.PreviousFrameAuthoritativePairCount += authoritativePairs;
            statistics.PreviousFramePairHitCount += pairHits;
            statistics.PreviousFramePairMissCount += pairMisses;
            statistics.PreviousFrameActivePairMissCount += activePairMisses;
            statistics.PreviousFramePredictivePairMissCount += predictivePairMisses;
            statistics.PreviousFramePreSolveEscapeBodyCount += preSolveEscapes;
            statistics.PreviousFrameFinalEscapeBodyCount += finalEscapes;
            statistics.PreviousFrameContactDrivenEscapeBodyCount += contactDrivenEscapes;
            statistics.PreviousFrameWallDrivenEscapeBodyCount += wallDrivenEscapes;
        }
        else
        {
            if (countPairCoverage)
                statistics.CurrentFrameCheckCount++;
            statistics.CurrentFrameAuthoritativePairCount += authoritativePairs;
            statistics.CurrentFramePairHitCount += pairHits;
            statistics.CurrentFramePairMissCount += pairMisses;
            statistics.CurrentFrameActivePairMissCount += activePairMisses;
            statistics.CurrentFramePredictivePairMissCount += predictivePairMisses;
            statistics.CurrentFramePreSolveEscapeBodyCount += preSolveEscapes;
            statistics.CurrentFrameFinalEscapeBodyCount += finalEscapes;
            statistics.CurrentFrameContactDrivenEscapeBodyCount += contactDrivenEscapes;
            statistics.CurrentFrameWallDrivenEscapeBodyCount += wallDrivenEscapes;
        }
    }

    private void PromoteCurrentShadowCache()
    {
        ShadowPreviousProxies.Clear();
        ShadowPreviousPairs.Clear();
        ShadowPreviousProxies.AddRange(ShadowCurrentProxies.AsArray());
        ShadowPreviousPairs.AddRange(ShadowCurrentPairs.AsArray());
    }

    private static bool TryFindProxy(
        NativeList<ShadowFatBodyProxy> proxies,
        Entity entity,
        out ShadowFatBodyProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            ShadowFatBodyProxy candidate = proxies[middle];
            int comparison = ShadowEntityOrdering.Compare(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return candidate.IsValid != 0;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }

        proxy = default;
        return false;
    }

    private static bool ContainsPair(
        NativeList<ShadowEntityPair> pairs,
        ShadowEntityPair target)
    {
        int low = 0;
        int high = pairs.Length - 1;
        var comparer = new ShadowEntityPairComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = comparer.Compare(pairs[middle], target);
            if (comparison == 0)
                return true;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return false;
    }

    private static bool AabbContains(
        float2 outerMin,
        float2 outerMax,
        float2 innerMin,
        float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    private static bool AabbOverlaps(
        float2 minA,
        float2 maxA,
        float2 minB,
        float2 maxB)
    {
        return math.all(maxA >= minB) && math.all(maxB >= minA);
    }

    private void EmitCellPairs()
    {
        int cellStart = 0;
        while (cellStart < SweptCellEntries.Length)
        {
            int cellIndex = SweptCellEntries[cellStart].CellIndex;
            int cellEnd = cellStart + 1;
            while (cellEnd < SweptCellEntries.Length &&
                   SweptCellEntries[cellEnd].CellIndex == cellIndex)
                cellEnd++;

            for (int first = cellStart; first < cellEnd; first++)
            {
                int firstBody = SweptCellEntries[first].BodyIndex;
                for (int second = first + 1; second < cellEnd; second++)
                {
                    int secondBody = SweptCellEntries[second].BodyIndex;
                    if (firstBody == secondBody)
                        continue;

                    Pairs.Add(new UnitCollisionPair
                    {
                        BodyA = math.min(firstBody, secondBody),
                        BodyB = math.max(firstBody, secondBody)
                    });
                }
            }

            cellStart = cellEnd;
        }
    }

    private void SortAndDeduplicatePairs()
    {
        if (Pairs.Length <= 1)
            return;

        Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        int writeIndex = 1;
        UnitCollisionPair previous = Pairs[0];

        for (int readIndex = 1; readIndex < Pairs.Length; readIndex++)
        {
            UnitCollisionPair current = Pairs[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;

            Pairs[writeIndex++] = current;
            previous = current;
        }

        Pairs.ResizeUninitialized(writeIndex);
    }

    private void FilterAndClassifyPairs(
        ref PredictiveDiscContactStatistics statistics,
        float skin)
    {
        int writeIndex = 0;

        for (int readIndex = 0; readIndex < Pairs.Length; readIndex++)
        {
            UnitCollisionPair pair = Pairs[readIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;

            float3 r0 = bodyB.StartPosition - bodyA.StartPosition;
            float3 relativeDisplacement =
                (bodyB.PredictedPosition - bodyB.StartPosition) -
                (bodyA.PredictedPosition - bodyA.StartPosition);
            r0.y = 0;
            relativeDisplacement.y = 0;

            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistanceSq = math.lengthsq(r0 + closestTime * relativeDisplacement);
            float candidateDistance = radiusSum + skin;
            if (minDistanceSq > candidateDistance * candidateDistance)
            {
                if (EnableDiagnostics)
                {
                    AddSelectedPairDiagnostic(
                        pair,
                        Stage3ContactDiagnosticPairKind.BroadPhaseRejected,
                        closestTime,
                        math.sqrt(minDistanceSq),
                        radiusSum,
                        0);
                }
                continue;
            }

            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta = bodyB.PredictedPosition - bodyA.PredictedPosition;
            endDelta.y = 0;
            float endDistanceSq = math.lengthsq(endDelta);
            float radiusSumSq = radiusSum * radiusSum;

            // 只有“起终点均分离、但线性 swept path 实际穿过接触半径”的 Pair
            // 使用初始分离平面。普通重叠和仅进入 skin 的 Pair 保持径向约束。
            bool shouldPreventSideExchange =
                startDistanceSq >= radiusSumSq &&
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;

            if (shouldPreventSideExchange)
                statistics.PotentialPredictivePairCount++;

            pair.Lambda = 0f;
            pair.WasActivated = 0;
            pair.ContactMode = shouldPreventSideExchange && EnablePredictiveContacts
                ? UnitContactMode.Predictive
                : UnitContactMode.Regular;
            Pairs[writeIndex++] = pair;

            if (pair.ContactMode == UnitContactMode.Predictive)
                statistics.PredictivePairCount++;
        }

        Pairs.ResizeUninitialized(writeIndex);
        statistics.ContactPairCount += writeIndex;
    }

    private void SolveContactIteration(
        float substepDeltaTime,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);

        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            float denominator = bodyA.InverseMass + bodyB.InverseMass + alpha;
            if (denominator <= 0f)
                continue;

            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0;
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 normal;
            float constraintValue;

            if (pair.ContactMode == UnitContactMode.Predictive)
            {
                float3 initialDelta = bodyA.StartPosition - bodyB.StartPosition;
                initialDelta.y = 0;
                normal = math.normalizesafe(
                    initialDelta,
                    DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
                constraintValue = math.dot(currentDelta, normal) - radiusSum;
            }
            else
            {
                float distance = math.length(currentDelta);
                normal = distance > 0.00001f
                    ? currentDelta / distance
                    : DeterministicFallbackNormal(pair.BodyA, pair.BodyB);
                constraintValue = distance - radiusSum;
            }

            float deltaLambda = -(constraintValue + alpha * pair.Lambda) / denominator;
            float nextLambda = math.max(0f, pair.Lambda + deltaLambda);
            float appliedLambda = nextLambda - pair.Lambda;
            pair.Lambda = nextLambda;

            if (nextLambda > 0.0000001f)
                pair.WasActivated = 1;
            Pairs[i] = pair;

            if (math.abs(appliedLambda) <= 0.0000001f)
                continue;

            float pairCorrection =
                (bodyA.InverseMass + bodyB.InverseMass) * math.abs(appliedLambda);
            totalPositionCorrection += pairCorrection;
            maxPositionCorrection = math.max(maxPositionCorrection, pairCorrection);

            bodyA.PredictedPosition += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.PredictedPosition -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.ContactPositionCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.ContactPositionCorrection -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.PredictedPosition.y = bodyA.CurrentPosition.y;
            bodyB.PredictedPosition.y = bodyB.CurrentPosition.y;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;
        }
    }

    private void AccumulateConstraintStatistics(
        ref PredictiveDiscContactStatistics statistics,
        ref float penetrationSum)
    {
        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            if (pair.WasActivated != 0)
            {
                statistics.ActiveConstraintCount++;
                if (pair.ContactMode == UnitContactMode.Predictive)
                    statistics.PredictiveActivatedCount++;
            }

            float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            delta.y = 0;
            float penetration = math.max(0f, bodyA.Radius + bodyB.Radius - math.length(delta));
            if (penetration <= 0f)
                continue;

            statistics.PenetratingPairCount++;
            statistics.MaxPenetration = math.max(statistics.MaxPenetration, penetration);
            penetrationSum += penetration;
        }
    }

    private void SolveWallConstraintIteration(
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        if (!Grid.IsCreated)
            return;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid || state.InverseMass <= 0f)
                continue;

            int2 currentCell = FlowFieldUtils.WorldToCell(
                state.PredictedPosition,
                GridOrigin,
                CellRadius);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                        checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                        continue;

                    int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                    if (Grid[checkIndex].Cost != 0)
                        continue;

                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2f + CellRadius,
                        state.PredictedPosition.y,
                        checkCell.y * CellRadius * 2f + CellRadius);
                    float3 delta = state.PredictedPosition - wallPosition;
                    delta.y = 0f;
                    float distance = math.length(delta);
                    float hardDistance = CellRadius + math.max(0f, state.Radius);
                    if (distance >= hardDistance)
                        continue;

                    float3 normal = distance > 0.00001f
                        ? delta / distance
                        : DeterministicFallbackNormal(bodyIndex, checkIndex);
                    float3 correction = normal * ((hardDistance - distance) * 0.5f);
                    state.PredictedPosition += correction;
                    state.PredictedPosition.y = state.CurrentPosition.y;
                    state.WallPositionCorrection += correction;

                    float correctionLength = math.length(correction);
                    totalPositionCorrection += correctionLength;
                    maxPositionCorrection = math.max(maxPositionCorrection, correctionLength);
                }
            }

            States[bodyIndex] = state;
        }
    }

    private void RecordIterationDiagnostic(
        int substepIndex,
        int iterationIndex,
        float totalPositionCorrection,
        float maxPositionCorrection,
        float totalWallPositionCorrection,
        float maxWallPositionCorrection)
    {
        float violationSum = 0f;
        float radialPenetrationSum = 0f;
        float maxViolation = 0f;
        float maxRadialPenetration = 0f;
        int violatingCount = 0;
        int penetratingCount = 0;
        int activeCount = 0;
        int predictiveActivatedCount = 0;

        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0;

            float constraintValue;
            if (pair.ContactMode == UnitContactMode.Predictive)
            {
                float3 initialDelta = bodyA.StartPosition - bodyB.StartPosition;
                initialDelta.y = 0;
                float3 normal = math.normalizesafe(
                    initialDelta,
                    DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
                constraintValue = math.dot(currentDelta, normal) - radiusSum;
            }
            else
            {
                constraintValue = math.length(currentDelta) - radiusSum;
            }

            float violation = math.max(0f, -constraintValue);
            if (violation > 0f)
            {
                violationSum += violation;
                maxViolation = math.max(maxViolation, violation);
                violatingCount++;
            }

            float radialPenetration = math.max(0f, radiusSum - math.length(currentDelta));
            if (radialPenetration > 0f)
            {
                radialPenetrationSum += radialPenetration;
                maxRadialPenetration = math.max(maxRadialPenetration, radialPenetration);
                penetratingCount++;
            }

            if (pair.Lambda <= 0.0000001f)
                continue;

            activeCount++;
            if (pair.ContactMode == UnitContactMode.Predictive)
                predictiveActivatedCount++;
        }

        IterationDiagnostics.Add(new Stage3ContactIterationDiagnostic
        {
            SubstepIndex = substepIndex,
            IterationIndex = iterationIndex,
            ActiveConstraintCount = activeCount,
            PredictiveActivatedCount = predictiveActivatedCount,
            MaxConstraintViolation = maxViolation,
            AverageConstraintViolation = violatingCount > 0
                ? violationSum / violatingCount
                : 0f,
            MaxRadialPenetration = maxRadialPenetration,
            AverageRadialPenetration = penetratingCount > 0
                ? radialPenetrationSum / penetratingCount
                : 0f,
            TotalPositionCorrection = totalPositionCorrection,
            MaxPositionCorrection = maxPositionCorrection,
            TotalWallPositionCorrection = totalWallPositionCorrection,
            MaxWallPositionCorrection = maxWallPositionCorrection
        });
    }

    private void CaptureSelectedBodyAndPairs(int substepIndex)
    {
        int selectedBodyIndex = FindSelectedBodyIndex();
        if (selectedBodyIndex < 0)
        {
            SelectedBodyDiagnostic.Value = default;
            return;
        }

        FlowMovementFrameState selected = States[selectedBodyIndex];
        var selectedDiagnostic = new Stage3SelectedBodyDiagnostic
        {
            IsValid = 1,
            SubstepIndex = substepIndex,
            Radius = selected.Radius,
            Skin = math.max(0f, PredictiveSkin),
            StartPosition = selected.StartPosition,
            UnconstrainedPredictedPosition = selected.UnconstrainedPredictedPosition,
            SolvedPosition = selected.PredictedPosition,
            ContactCorrection = selected.ContactPositionCorrection,
            WallCorrection = selected.WallPositionCorrection,
            VelocityBeforeContact = selected.VelocityBeforeContact,
            VelocityAfterContact = selected.IntegratedVelocity
        };

        if (EnableShadowNeighborCacheTest)
        {
            NativeList<ShadowFatBodyProxy> referenceProxies =
                substepIndex == 0 && ShadowPreviousProxies.Length > 0
                    ? ShadowPreviousProxies
                    : ShadowCurrentProxies;
            if (TryFindProxy(referenceProxies, selected.Entity, out ShadowFatBodyProxy proxy))
            {
                float coreExtent = math.max(0f, selected.Radius) + math.max(0f, PredictiveSkin);
                float2 finalMin = selected.PredictedPosition.xz - coreExtent;
                float2 finalMax = selected.PredictedPosition.xz + coreExtent;
                selectedDiagnostic.ShadowReferenceAvailable = 1;
                selectedDiagnostic.ShadowEscaped =
                    (byte)(AabbContains(proxy.FatMin, proxy.FatMax, finalMin, finalMax) ? 0 : 1);
                selectedDiagnostic.ShadowFatMin = proxy.FatMin;
                selectedDiagnostic.ShadowFatMax = proxy.FatMax;
            }
        }

        SelectedBodyDiagnostic.Value = selectedDiagnostic;

        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            if (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex)
                continue;

            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float3 r0 = bodyB.StartPosition - bodyA.StartPosition;
            float3 relativeDisplacement =
                (bodyB.UnconstrainedPredictedPosition - bodyB.StartPosition) -
                (bodyA.UnconstrainedPredictedPosition - bodyA.StartPosition);
            r0.y = 0;
            relativeDisplacement.y = 0;
            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistance = math.length(r0 + closestTime * relativeDisplacement);
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta =
                bodyB.UnconstrainedPredictedPosition - bodyA.UnconstrainedPredictedPosition;
            endDelta.y = 0;
            bool potentialPredictive =
                startDistanceSq >= radiusSum * radiusSum &&
                math.lengthsq(endDelta) >= radiusSum * radiusSum &&
                minDistance <= radiusSum;

            Stage3ContactDiagnosticPairKind kind;
            if (potentialPredictive)
            {
                kind = EnablePredictiveContacts
                    ? Stage3ContactDiagnosticPairKind.Predictive
                    : Stage3ContactDiagnosticPairKind.PredictiveDisabled;
            }
            else
            {
                kind = Stage3ContactDiagnosticPairKind.Regular;
            }

            AddSelectedPairDiagnostic(
                pair,
                kind,
                closestTime,
                minDistance,
                radiusSum,
                pair.WasActivated);
        }
    }

    private void AddSelectedPairDiagnostic(
        UnitCollisionPair pair,
        Stage3ContactDiagnosticPairKind kind,
        float closestTime,
        float minimumDistance,
        float radiusSum,
        byte wasActivated)
    {
        int selectedBodyIndex = FindSelectedBodyIndex();
        if (selectedBodyIndex < 0 ||
            (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex))
            return;

        int otherBodyIndex = pair.BodyA == selectedBodyIndex ? pair.BodyB : pair.BodyA;
        FlowMovementFrameState selected = States[selectedBodyIndex];
        FlowMovementFrameState other = States[otherBodyIndex];
        float3 selectedClosest = math.lerp(
            selected.StartPosition,
            selected.UnconstrainedPredictedPosition,
            closestTime);
        float3 otherClosest = math.lerp(
            other.StartPosition,
            other.UnconstrainedPredictedPosition,
            closestTime);

        PairDiagnostics.Add(new Stage3ContactPairDiagnostic
        {
            OtherEntity = other.Entity,
            Kind = kind,
            WasActivated = wasActivated,
            ClosestTime = closestTime,
            MinimumDistance = minimumDistance,
            RadiusSum = radiusSum,
            OtherRadius = other.Radius,
            OtherStartPosition = other.StartPosition,
            OtherPredictedPosition = other.UnconstrainedPredictedPosition,
            SelectedClosestPosition = selectedClosest,
            OtherClosestPosition = otherClosest
        });
    }

    private int FindSelectedBodyIndex()
    {
        if (DiagnosticSelectedEntity == Entity.Null)
            return -1;

        for (int i = 0; i < States.Length; i++)
        {
            if (States[i].Entity == DiagnosticSelectedEntity)
                return i;
        }

        return -1;
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

    public static float3 DeterministicFallbackNormal(int bodyA, int bodyB)
    {
        uint hash = math.hash(new int2(bodyA, bodyB));
        return (hash & 1u) == 0u
            ? new float3(1, 0, 0)
            : new float3(0, 0, 1);
    }

    private static long TimestampToNanoseconds(long timestampDelta)
    {
        var ratio = ProfilerUnsafeUtility.TimestampToNanosecondsConversionRatio;
        return timestampDelta * ratio.Numerator / ratio.Denominator;
    }
}

[BurstCompile]
public partial struct PublishPredictiveDiscContactStatisticsJob : IJobEntity
{
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> Source;
    [ReadOnly] public NativeReference<ShadowNeighborCacheStatistics> ShadowSource;
    [ReadOnly] public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodySource;
    [ReadOnly] public NativeList<Stage3ContactIterationDiagnostic> IterationSource;
    [ReadOnly] public NativeList<Stage3ContactPairDiagnostic> PairSource;

    public void Execute(
        ref PredictiveDiscContactStatistics destination,
        ref ShadowNeighborCacheStatistics shadowDestination,
        ref Stage3SelectedBodyDiagnostic selectedBodyDestination,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDestination,
        DynamicBuffer<Stage3ContactPairDiagnostic> pairDestination)
    {
        destination = Source.Value;
        shadowDestination = ShadowSource.Value;
        selectedBodyDestination = SelectedBodySource.Value;
        iterationDestination.Clear();
        pairDestination.Clear();
        iterationDestination.AddRange(IterationSource.AsArray());
        pairDestination.AddRange(PairSource.AsArray());
    }
}
