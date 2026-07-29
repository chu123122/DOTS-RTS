using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
using Unity.Entities;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
    private void BuildInitialContactSet(
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0 || !Configuration.EnableTimestepContactSetCache)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        long start = ProfilerUnsafeUtility.Timestamp;
        BuildOrRefreshTimestepContactViews(ref statistics, ref incremental, false, false);
        statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - start);
        RebuildPersistentIncidentPairLookupIfNeededParallel();
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void FinalizeEnvelopeEscapes(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0 || !Configuration.EnableTimestepContactSetCache)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int newlyEscaped = CountNewlyEscaped();
        if (newlyEscaped > 0)
        {
            statistics.TimestepContactSetEscapeBodyCount += newlyEscaped;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
        }
        incremental.InteractionEnvelopeEscapeCount += IncrementalDirtyBodies.Length;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private int CountNewlyEscaped()
    {
        int newlyEscaped = 0;
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            newlyEscaped += ParallelBodyStatistics[bodyIndex].EscapeCount;
        }
        return newlyEscaped;
    }

    private void PrepareSubstepRepairClassification(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        PersistentClassificationPhaseState phase = default;
        PersistentClassificationResults.Clear();
        PersistentClassificationState.Value = phase;

        if (runtimeState.Value.IsValid == 0)
            return;
        if (!Configuration.EnableTimestepContactSetCache ||
            !Configuration.EnablePersistentContactCache ||
            IncrementalDirtyBodies.Length == 0)
        {
            RepairSubstepContactView(substepIndex, runtimeState);
            return;
        }

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        if (!ConsumeDirtyBodyRefreshSummary(
                ref incremental,
                out int topologyDirtyCount))
        {
            incremental.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndex();
            RebuildPersistentIncidentPairLookupIfNeededParallel();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }
        incremental.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = Bodies.Length > 0
            ? (float)IncrementalDirtyBodies.Length / Bodies.Length
            : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
        {
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndex();
            RebuildPersistentIncidentPairLookupIfNeededParallel();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }

        long pairDiffStart = ProfilerUnsafeUtility.Timestamp;
        long localBroadPhaseBefore = incremental.LocalBroadPhaseNanoseconds;
        if (topologyDirtyCount > 0)
            IncrementallyRepairPersistentNeighborTopology(ref incremental, false);
        long pairDiffElapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localBroadPhaseElapsed =
            incremental.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
        long pairDiffExclusive = pairDiffElapsed - localBroadPhaseElapsed;
        incremental.PairDiffNanoseconds += pairDiffExclusive > 0L
            ? pairDiffExclusive
            : 0L;

        PreviousTimestepContactPairs.Clear();
        PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies(
                out int _,
                out int eligibilitySkipped))
        {
            incremental.ClassificationSkippedCount += eligibilitySkipped;
            incremental.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            BuildOrRefreshTimestepContactViews(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndex();
            RebuildPersistentIncidentPairLookupIfNeededParallel();
            StoreContactStatistics(statistics);
            StoreIncrementalStatistics(incremental);
            return;
        }
        incremental.ClassificationSkippedCount += eligibilitySkipped;
        incremental.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);

#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry =
            new PersistentClassificationTelemetryState
            {
                BuildStartTimestamp = ProfilerUnsafeUtility.Timestamp
            };
        telemetry.ClassificationStartTimestamp = telemetry.BuildStartTimestamp;
        PersistentClassificationTelemetry.Value = telemetry;
#endif
        phase.Timestep = IncrementalCacheState.Value.Timestep;
        phase.ClassificationEpoch = CalculateClassificationEpoch();
        phase.NeedsCommit = 2;
        ClassificationBodyPairs.Clear();
        ContactPipelineShared.CopyConstraintsToBodyPairs(Pairs.AsArray(), ClassificationBodyPairs);
        PersistentClassificationResults.ResizeUninitialized(
            ClassificationBodyPairs.Length);
        PersistentClassificationState.Value = phase;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void CommitSubstepRepairClassification(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        PersistentClassificationPhaseState phase =
            PersistentClassificationState.Value;
        if (runtimeState.Value.IsValid == 0 || phase.NeedsCommit != 2)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int retainedCount = 0;
        int activeWriteIndex = 0;
        statistics.CandidatePairCount += PersistentClassificationResults.Length;

        for (int pairIndex = 0;
             pairIndex < PersistentClassificationResults.Length;
             pairIndex++)
        {
            PersistentPairClassificationResult result =
                PersistentClassificationResults[pairIndex];
            BodyPair rawPair = result.RawPair;
            PersistentPredictiveContact contact = result.Contact;
            PredictiveContactScratch.Add(contact);
            // 新分类结果写入 O(1) 索引
            if (PersistentContactIndex.IsCreated)
                PersistentContactIndex[contact.Key] = contact;
            if (result.WasReclassified != 0)
            {
                incremental.ReclassifiedPairEvaluationCount++;
                incremental.SweptClassificationEvaluationCount++;
            }
            else
            {
                incremental.ClassificationReuseCount++;
                incremental.ClassificationSkippedCount++;
            }
            AccumulatePersistentClassificationStatistics(contact, ref statistics);

            if (contact.Lifecycle == PersistentContactLifecycle.Expired)
                continue;
            retainedCount++;
            if (contact.Lifecycle == PersistentContactLifecycle.Dormant)
            {
                PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = contact.Key,
                    Substep = contact.NextCheckSubstep
                });
                continue;
            }
            Pairs[activeWriteIndex++] = BuildContactConstraintFromPersistentContact(
                rawPair.BodyA,
                rawPair.BodyB,
                contact);
        }

        Pairs.ResizeUninitialized(activeWriteIndex);
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new ContactConstraintComparer());
        // 无需排序 PredictiveContactScratch：hashmap 提供 O(1) 查找
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(
                new PredictiveContactScheduleEntryComparer());
        PredictiveContactScheduleCursor.Value = 0;

        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
        // hashmap 已在 Prepare（删dirty）+ 本函数各条目写入（Add new）中增量维护，无需全量重建
        RebuildPersistentContactViews();
        RebuildSoftAvoidancePairSetFromPersistentContacts();
        statistics.ContactPairCount += retainedCount;
        incremental.CurrentInteractionPairCount = PersistentNeighborPairs.Length;
        incremental.CurrentSoftAvoidancePairCount = SoftAvoidancePairs.Length;
        incremental.PersistentViewRebuildCount++;

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.ClassificationEpoch = phase.ClassificationEpoch;
        cacheState.LastUpdateWasFullRebuild = 0;
        cacheState.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = cacheState;

        RebuildEscapedTimestepContactView(ref statistics, ref incremental);
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            stateEvidence.EnvelopeEscaped = 0;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }

        incremental.IncrementalRepairCount++;
        incremental.UsedIncrementalTopology = 1;
        incremental.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry =
            PersistentClassificationTelemetry.Value;
        incremental.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - telemetry.ClassificationStartTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - telemetry.BuildStartTimestamp);
#endif

        InvalidateSoftIncidentIndex();
        RebuildPersistentIncidentPairLookupIfNeededParallel();
        IssueCertificateForCommittedViews(incremental, substepIndex);
        phase.NeedsCommit = 0;
        PersistentClassificationState.Value = phase;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private bool TryFindCurrentIncrementalProxy(
        Entity entity,
        out PersistentSweptProxy proxy,
        out int proxyIndex)
    {
        int low = 0;
        int high = CurrentIncrementalProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = CurrentIncrementalProxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                proxyIndex = middle;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        proxy = default;
        proxyIndex = -1;
        return false;
    }

    private void RepairSubstepContactView(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        if (runtimeState.Value.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, Configuration.SubstepCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;

        if (!Configuration.EnableTimestepContactSetCache)
        {
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepInteractionAndSoftViews(ref statistics, ref incremental);
            InvalidateSoftIncidentIndex();
            statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }
        else if (IncrementalDirtyBodies.Length > 0)
        {
            RepairOrRebuildPreparedContactViewForRemainingTime(
                substepIndex,
                ref statistics,
                ref incremental);
            InvalidateSoftIncidentIndex();
            RebuildPersistentIncidentPairLookupIfNeededParallel();
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void RepairOrRebuildPreparedContactViewForRemainingTime(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        int scheduleStartSubstep = substepIndex;
        if (Configuration.EnablePersistentContactCache &&
            TryIncrementallyRepairEscapedContactSet(
                substepIndex,
                scheduleStartSubstep,
                ref statistics,
                ref incrementalStatistics))
            return;

        BuildOrRefreshTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            true,
            true,
            scheduleStartSubstep);
    }

    private void FinalizePreparedSubstep(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int substepCount = math.max(1, Configuration.SubstepCount);
        float substepDeltaTime = Configuration.DeltaTime / substepCount;

        int newlyEscaped = CountNewlyEscaped();
        if (newlyEscaped > 0)
        {
            statistics.TimestepContactSetEscapeBodyCount += newlyEscaped;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
        }
        incremental.CorrectedEscapeBodyCount += IncrementalDirtyBodies.Length;
        bool rebuilt = false;
        if (IncrementalDirtyBodies.Length > 0)
        {
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                Configuration.EnableTimestepContactSetCache,
                ref statistics,
                ref incremental,
                false);
            InvalidateSoftIncidentIndex();
            RebuildPersistentIncidentPairLookupIfNeededParallel();
            rebuilt = true;
        }
        if (!Configuration.EnableTimestepContactSetCache && !rebuilt)
        {
            // Preserve the reference ordering: first validate the pre-soft swept
            // envelope, then publish the actual solved substep trajectory used by
            // Narrow Phase. Preparing this before validation would make every B0
            // validation trivially pass.
            PrepareSubstepContactPrediction();
            long start = ProfilerUnsafeUtility.Timestamp;
            BuildSubstepContactView(ref statistics, ref incremental);
            InvalidateSoftIncidentIndex();
            statistics.PairGenerationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - start);
        }

        ActivateScheduledPredictiveContactsForSubstep(
            Configuration.EnableTimestepContactSetCache ? substepIndex : 0,
            Configuration.EnableTimestepContactSetCache ? substepCount : 1,
            ref incremental);
        EnsureActiveConstraintIncidentIndex();
        statistics.TimestepContactSetSubstepUseCount++;
#if RTS_CONTACT_DIAGNOSTICS
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        runtime.IterationAccountedStartNanoseconds =
            AccountedCandidateNanoseconds(incremental);
#endif
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
        runtimeState.Value = runtime;
    }

    private void RebuildPersistentIncidentPairLookupIfNeededParallel()
    {
        if (!Configuration.EnablePersistentContactCache ||
            !PersistentIncidentPairLookup.IsCreated ||
            !PersistentIncidentLookupEpoch.IsCreated)
            return;
        uint epoch = IncrementalCacheState.Value.TopologyEpoch;
        int requiredEntryCount = PersistentNeighborPairs.Length * 2;
        if (requiredEntryCount > PersistentIncidentPairLookup.Capacity)
        {
            // Never publish a partial incident index. The repair caller detects
            // the invalid epoch and takes the authoritative full-rebuild path.
            PersistentIncidentPairLookup.Clear();
            PersistentIncidentLookupEpoch.Value = uint.MaxValue;
            return;
        }
        if (PersistentIncidentLookupEpoch.Value == epoch &&
            PersistentIncidentPairLookup.Count() == requiredEntryCount)
            return;
        PersistentIncidentPairLookup.Clear();
        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            StableEntityPairKey key = PersistentNeighborPairs[pairIndex].Key;
            PersistentIncidentPairLookup.Add(key.EntityA, pairIndex);
            PersistentIncidentPairLookup.Add(key.EntityB, pairIndex);
        }
        PersistentIncidentLookupEpoch.Value = epoch;
    }

    private void InvalidateSoftIncidentIndex()
    {
        ActiveIncidentIndexState state = ActiveIncidentIndexState.Value;
        state.SoftIsValid = 0;
        ActiveIncidentIndexState.Value = state;
    }


    private void FinalizeWallIteration(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        , int bodyBlockCount)
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        ContactSolverIterationTelemetry iteration = iterationState.Value;
        for (int blockIndex = 0; blockIndex < bodyBlockCount; blockIndex++)
        {
            int bodyIndex = blockIndex * CrowdContactPipelineScheduler.ParallelBodyBatchSize;
            ParallelBodyStageResult body = ParallelBodyStatistics[bodyIndex];
            iteration.TotalWallPositionCorrection += body.Total;
            iteration.MaxWallPositionCorrection = math.max(
                iteration.MaxWallPositionCorrection,
                body.Maximum);
        }

#endif

        if (runtime.IsValid != 0)
        {
            if (!ValidateSolverCorrectionContactEnvelope(
                    substepIndex,
                    ref statistics,
                    ref incremental))
            {
                int substepCount = math.max(1, Configuration.SubstepCount);
                float substepDeltaTime = Configuration.DeltaTime / substepCount;
                RepairOrRebuildContactViewForRemainingTime(
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    Configuration.EnableTimestepContactSetCache,
                    ref statistics,
                    ref incremental);
                InvalidateSoftIncidentIndex();
                ResetTimestepContactSetForSubstep();
                RebuildPersistentIncidentPairLookupIfNeededParallel();
            }

            ResetCorrectedBodyTracking();
        }

        // Unconditionally rebuild the active-constraint incident index against
        // the current TimestepContactPairs. GatherAndApplyParallelJacobiBodiesJob
        // (scheduled right after this) reads pairIndex values stored in the index
        // and indexes Pairs/Corrections with them; the index MUST match the live
        // pair count. The repair above (or an upstream escaped-view rebuild) can
        // shrink TimestepContactPairs, and the fingerprint fast-path inside Ensure
        // can otherwise leave a stale, larger index in place -> IndexOutOfRange in
        // Gather. Force the rebuild every iteration so the index is always current.
        ActiveIncidentIndexState.Value = default;
        EnsureActiveConstraintIncidentIndex();

        // This serial dependency boundary owns the deferred contact workset
        // lengths. The parallel Jacobi eval/reduce jobs scheduled after it run
        // unconditionally — even when the runtime is invalid (e.g. a consumer
        // certificate mismatch set IsValid=0) — so these deferred write targets
        // must match the committed contact-pair count regardless of validity.
        // Skipping the resize on the invalid path left them at length 0 and the
        // eval job threw IndexOutOfRange. Repair above may change the count, so
        // this resize must follow it.
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);
#if RTS_CONTACT_DIAGNOSTICS
        if (ParallelSimulationDebuggerPairCandidates.IsCreated)
        {
            ParallelSimulationDebuggerPairCandidates.ResizeUninitialized(
                TimestepContactPairs.Length);
        }
        blockStatistics.ResizeUninitialized(
            (TimestepContactPairs.Length + CrowdContactPipelineScheduler.JacobiPairBatchSize - 1) / CrowdContactPipelineScheduler.JacobiPairBatchSize);
#endif

        if (runtime.IsValid == 0)
            return;

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
#if RTS_CONTACT_DIAGNOSTICS
        iterationState.Value = iteration;
#endif
    }
}
}
