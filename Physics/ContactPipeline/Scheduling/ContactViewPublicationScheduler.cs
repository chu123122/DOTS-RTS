using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CrowdContactPipelineScheduler
{
    private void ScheduleContactViewCandidateSort(
        ref JobHandle handle,
        long maximumCandidateCount)
    {
        const int blockSize = 256;
        handle = new SortContactViewCandidateBlocksJob
        {
            Workset =
                Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray(),
            BlockSize = blockSize
        }.Schedule(
            Repair.ContactViewBlockWorkset,
            1,
            handle);

        long maximumBlockCount =
            (maximumCandidateCount + blockSize - 1L) / blockSize;
        int mergePassCount = 0;
        for (long width = 1; width < maximumBlockCount; width <<= 1)
            mergePassCount++;

        for (int mergePass = 0;
             mergePass < mergePassCount;
             mergePass++)
        {
            bool sourceIsCandidates = (mergePass & 1) == 0;
            handle = new MergeContactViewCandidateBlocksJob
            {
                Workset =
                    Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
                Source = sourceIsCandidates
                    ? Repair.ContactViewCandidates.AsDeferredJobArray()
                    : Repair.ContactViewSortScratch.AsDeferredJobArray(),
                Destination = sourceIsCandidates
                    ? Repair.ContactViewSortScratch.AsDeferredJobArray()
                    : Repair.ContactViewCandidates.AsDeferredJobArray(),
                BlockSize = blockSize,
                MergePass = mergePass
            }.Schedule(
                Repair.ContactViewBlockWorkset,
                1,
                handle);
        }

        if ((mergePassCount & 1) != 0)
        {
            handle = new CopyContactViewCandidateSortResultJob
            {
                Workset =
                    Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
                Source =
                    Repair.ContactViewSortScratch.AsDeferredJobArray(),
                Destination =
                    Repair.ContactViewCandidates.AsDeferredJobArray(),
                BlockSize = blockSize
            }.Schedule(
                Repair.ContactViewBlockWorkset,
                1,
                handle);
        }
    }

    private void ScheduleRepairContactViewPublication(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        const int blockSize = 256;
        handle = new PrepareRepairContactViewPublicationJob
        {
            RuntimeState = runtimeState,
            PhaseState = Classification.State,
            PreviousContacts = PreviousTimestepContactPairs,
            NewContacts = BroadPhase.CollisionPairs,
            Candidates = Repair.ContactViewCandidates,
            SortScratch = Repair.ContactViewSortScratch,
            CandidateWorkset = Repair.ContactViewCandidateWorkset,
            PublicationBlocks = Repair.ContactViewPublicationBlocks,
            BlockWorkset = Repair.ContactViewBlockWorkset,
            OutputContacts = NarrowPhaseConstraints.HardContacts,
            BlockSize = blockSize
        }.Schedule(handle);
        handle = new MaterializeRepairContactCandidatesJob
        {
            Workset =
                Repair.ContactViewCandidateWorkset.AsDeferredJobArray(),
            PreviousContacts =
                PreviousTimestepContactPairs.AsDeferredJobArray(),
            NewContacts =
                BroadPhase.CollisionPairs.AsDeferredJobArray(),
            DirtyFlagsByBody = Repair.DirtyFlagsByBody,
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray()
        }.Schedule(
            Repair.ContactViewCandidateWorkset,
            SoftPairBatchSize,
            handle);

        long maximumCandidateCount =
            (long)Body.Bodies.Length *
            math.max(0, Body.Bodies.Length - 1);
        ScheduleContactViewCandidateSort(
            ref handle,
            maximumCandidateCount);

        handle = new CountRepairContactPublicationBlocksJob
        {
            Workset =
                Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray(),
            Blocks =
                Repair.ContactViewPublicationBlocks.AsDeferredJobArray(),
            BlockSize = blockSize
        }.Schedule(
            Repair.ContactViewBlockWorkset,
            1,
            handle);
        handle = new PrefixRepairContactPublicationJob
        {
            Blocks = Repair.ContactViewPublicationBlocks,
            OutputContacts = NarrowPhaseConstraints.HardContacts
        }.Schedule(handle);
        handle = new ScatterRepairContactPublicationBlocksJob
        {
            Workset =
                Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray(),
            Blocks =
                Repair.ContactViewPublicationBlocks.AsDeferredJobArray(),
            OutputContacts =
                NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
            BlockSize = blockSize
        }.Schedule(
            Repair.ContactViewBlockWorkset,
            1,
            handle);
        handle = new FinalizeRepairContactViewPublicationJob
        {
            RuntimeState = runtimeState,
            PhaseState = Classification.State,
            Blocks = Repair.ContactViewPublicationBlocks,
            OutputContacts = NarrowPhaseConstraints.HardContacts,
#if RTS_CONTACT_DIAGNOSTICS
            Configuration = Configuration,
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            PersistentContacts =
                Persistent.PersistentPredictiveContacts,
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
            OracleContactPairs =
                Diagnostics.IncrementalOracleContactPairs,
#endif
        }.Schedule(handle);
    }

    private void SchedulePredictiveContactActivation(
        ref JobHandle handle,
        NativeReference<ContactPipelineExecutionState> runtimeState,
        int substepIndex,
        int substepCount)
    {
        const int activationBlockSize = 256;
        handle = new PreparePredictiveContactActivationJob
        {
            RuntimeState = runtimeState,
            Schedule = Certificate.Schedule,
            Records = Certificate.ActivationRecords,
            RecordWorkset = Certificate.ActivationRecordWorkset,
            Blocks = Certificate.ActivationBlocks,
            BlockWorkset = Certificate.ActivationBlockWorkset,
            ActivatedContacts = Certificate.ActivatedContacts,
            ScheduleScratch = Certificate.ScheduleScratch,
            Summary = Certificate.ActivationSummary,
            StartTimestamp = Certificate.ActivationStartTimestamp,
            BlockSize = activationBlockSize
        }.Schedule(handle);
        handle = new EvaluateScheduledContactsJob
        {
            Workset =
                Certificate.ActivationRecordWorkset.AsDeferredJobArray(),
            Schedule = Certificate.Schedule.AsDeferredJobArray(),
            Configuration = Configuration,
            Bodies = Body.Bodies,
            MotionEvidence = Body.MotionEvidence,
            StepStates = Body.StepStates,
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            ContactIndex = Persistent.PersistentContactIndex,
            PersistentContacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            Records =
                Certificate.ActivationRecords.AsDeferredJobArray(),
            SubstepIndex = Configuration.EnableTimestepContactSetCache
                ? substepIndex
                : 0,
            SubstepCount = Configuration.EnableTimestepContactSetCache
                ? math.max(1, substepCount)
                : 1
        }.Schedule(
            Certificate.ActivationRecordWorkset,
            SoftPairBatchSize,
            handle);
        handle = new CountPredictiveContactActivationBlocksJob
        {
            Workset =
                Certificate.ActivationBlockWorkset.AsDeferredJobArray(),
            Records =
                Certificate.ActivationRecords.AsDeferredJobArray(),
            Blocks =
                Certificate.ActivationBlocks.AsDeferredJobArray(),
            BlockSize = activationBlockSize
        }.Schedule(
            Certificate.ActivationBlockWorkset,
            1,
            handle);
        handle = new PrefixPredictiveContactActivationJob
        {
            Blocks = Certificate.ActivationBlocks,
            ActivatedContacts = Certificate.ActivatedContacts,
            ScheduleScratch = Certificate.ScheduleScratch,
            Summary = Certificate.ActivationSummary,
            ScheduleCursor = Certificate.ScheduleCursor,
            CacheState = Persistent.IncrementalCacheState,
            EnablePersistentContactCache = (byte)(
                Configuration.EnablePersistentContactCache ? 1 : 0)
        }.Schedule(handle);
        handle = new ScatterPredictiveContactActivationBlocksJob
        {
            Workset =
                Certificate.ActivationBlockWorkset.AsDeferredJobArray(),
            Records =
                Certificate.ActivationRecords.AsDeferredJobArray(),
            Blocks =
                Certificate.ActivationBlocks.AsDeferredJobArray(),
            ActivatedContacts =
                Certificate.ActivatedContacts.AsDeferredJobArray(),
            ScheduleScratch =
                Certificate.ScheduleScratch.AsDeferredJobArray(),
            PersistentContacts =
                Persistent.PersistentPredictiveContacts
                    .AsDeferredJobArray(),
            BlockSize = activationBlockSize
        }.Schedule(
            Certificate.ActivationBlockWorkset,
            1,
            handle);
        handle = new PreparePredictiveContactScheduleCommitJob
        {
            ScheduleScratch = Certificate.ScheduleScratch,
            Schedule = Certificate.Schedule
        }.Schedule(handle);
        handle = new CopyPredictiveContactScheduleJob
        {
            Source =
                Certificate.ScheduleScratch.AsDeferredJobArray(),
            Destination =
                Certificate.Schedule.AsDeferredJobArray()
        }.Schedule(
            Certificate.ScheduleScratch,
            SoftPairBatchSize,
            handle);

        const int contactViewBlockSize = 256;
        handle = new PrepareActivationContactViewPublicationJob
        {
            RuntimeState = runtimeState,
            ExistingContacts = NarrowPhaseConstraints.HardContacts,
            ActivatedContacts = Certificate.ActivatedContacts,
            Candidates = Repair.ContactViewCandidates,
            SortScratch = Repair.ContactViewSortScratch,
            CandidateWorkset = Repair.ContactViewCandidateWorkset,
            PublicationBlocks = Repair.ContactViewPublicationBlocks,
            BlockWorkset = Repair.ContactViewBlockWorkset,
            BlockSize = contactViewBlockSize
        }.Schedule(handle);
        handle = new MaterializeActivationContactCandidatesJob
        {
            Workset =
                Repair.ContactViewCandidateWorkset.AsDeferredJobArray(),
            ExistingContacts =
                NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
            ActivatedContacts =
                Certificate.ActivatedContacts.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray()
        }.Schedule(
            Repair.ContactViewCandidateWorkset,
            SoftPairBatchSize,
            handle);

        long maximumCandidateCount =
            (long)Body.Bodies.Length *
            math.max(0, Body.Bodies.Length - 1);
        ScheduleContactViewCandidateSort(
            ref handle,
            maximumCandidateCount);
        handle = new CountActivationContactPublicationBlocksJob
        {
            Workset =
                Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray(),
            Blocks =
                Repair.ContactViewPublicationBlocks.AsDeferredJobArray(),
            BlockSize = contactViewBlockSize
        }.Schedule(
            Repair.ContactViewBlockWorkset,
            1,
            handle);
        handle = new PrefixActivationContactPublicationJob
        {
            Blocks = Repair.ContactViewPublicationBlocks,
            OutputContacts = NarrowPhaseConstraints.HardContacts
        }.Schedule(handle);
        handle = new ScatterActivationContactPublicationBlocksJob
        {
            Workset =
                Repair.ContactViewBlockWorkset.AsDeferredJobArray(),
            Candidates =
                Repair.ContactViewCandidates.AsDeferredJobArray(),
            Blocks =
                Repair.ContactViewPublicationBlocks.AsDeferredJobArray(),
            OutputContacts =
                NarrowPhaseConstraints.HardContacts.AsDeferredJobArray(),
            BlockSize = contactViewBlockSize
        }.Schedule(
            Repair.ContactViewBlockWorkset,
            1,
            handle);

        handle = new FinalizePredictiveContactActivationJob
        {
            Configuration = Configuration,
            Bodies = Body.Bodies,
            TimestepInteractionPairs =
                BroadPhaseCandidates.Pairs,
            SoftAvoidancePairs =
                NarrowPhaseConstraints.SoftInteractions,
            TimestepContactPairs =
                NarrowPhaseConstraints.HardContacts,
            CurrentBodyIndexByEntity =
                Body.CurrentBodyIndexByEntity,
            PersistentNeighborPairs =
                Persistent.PersistentNeighborPairs,
            Schedule = Certificate.Schedule,
            CacheState = Persistent.IncrementalCacheState,
            DirtyBodies = Repair.DirtyBodies,
            Summary = Certificate.ActivationSummary,
            StartTimestamp = Certificate.ActivationStartTimestamp,
            InteractionCertificate = Certificate.Certificate,
            CertificateViolations = Certificate.Violations,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Diagnostics.ContactStatistics,
            IncrementalStatistics =
                Diagnostics.IncrementalStatistics,
#endif
            RuntimeState = runtimeState,
            SubstepIndex = substepIndex
        }.Schedule(handle);
    }
}
}
