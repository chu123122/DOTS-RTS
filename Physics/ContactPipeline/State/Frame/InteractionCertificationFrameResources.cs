using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// 单步认证器 scratch 加上已认证的紧凑产物。这些容器并非通用帧袋：仅认证方有写权限；
/// 下游阶段只能消费各自对应的已认证列表。
/// </summary>
internal struct InteractionCertificationFrameResources
{
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeArray<int> BodyCellCounts;
    public NativeArray<int> BodyCellOffsets;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> CollisionPairs;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<BodyPair> ClassificationBodyPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;

    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> InteractionViolations;

    public NativeList<PersistentPairClassificationResult> PersistentClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> PersistentClassificationState;
    public NativeArray<uint> PersistentSpatialVisitStampByProxy;
    public NativeReference<uint> PersistentSpatialVisitStamp;
    public NativeArray<DirtyBodyRefreshResult> DirtyBodyRefreshResults;
    public NativeReference<DirtyBodyRefreshSummary> DirtyBodyRefreshSummary;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockCounts;
    public NativeList<DirtyContactScheduleBlock> DirtyContactScheduleBlockOffsets;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
#endif

    public static InteractionCertificationFrameResources Create(int unitCount)
    {
        int one = math.max(unitCount, 1);
        return new InteractionCertificationFrameResources
        {
            SweptCellEntries = new NativeList<SweptDiscCellEntry>(math.max(unitCount * 4, 1), Allocator.TempJob),
            BodyCellCounts = new NativeArray<int>(
                unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            BodyCellOffsets = new NativeArray<int>(
                unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            CellPairCounts = new NativeList<int>(math.max(unitCount * 4, 1), Allocator.TempJob),
            CellPairOffsets = new NativeList<int>(math.max(unitCount * 4, 1), Allocator.TempJob),
            FullSweepPrepared = new NativeReference<byte>(Allocator.TempJob),
            CollisionPairs = new NativeList<ContactConstraint>(math.max(unitCount * 4, 1), Allocator.TempJob),
            TimestepContactPairs = new NativeList<ContactConstraint>(math.max(unitCount * 4, 1), Allocator.TempJob),
            PreviousTimestepContactPairs = new NativeList<ContactConstraint>(math.max(unitCount * 8, 1), Allocator.TempJob),
            TimestepInteractionPairs = new NativeList<BodyPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            SoftAvoidancePairs = new NativeList<BodyPair>(math.max(unitCount * 4, 1), Allocator.TempJob),
            ClassificationBodyPairs = new NativeList<BodyPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            CurrentBodyIndexByEntity = new NativeParallelHashMap<Entity, int>(one, Allocator.TempJob),
            CurrentIncrementalProxies = new NativeList<PersistentSweptProxy>(one, Allocator.TempJob),
            IncrementalDirtyBodies = new NativeList<IncrementalDirtyBody>(one, Allocator.TempJob),
            IncrementalDirtyFlagsByBody = new NativeArray<byte>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            PredictiveContactScratch = new NativeList<PersistentPredictiveContact>(math.max(unitCount * 4, 1), Allocator.TempJob),
            IncrementalNeighborPairScratch = new NativeList<PersistentNeighborPair>(math.max(unitCount * 8, 1), Allocator.TempJob),
            PredictiveContactSchedule = new NativeList<PredictiveContactScheduleEntry>(math.max(unitCount * 2, 1), Allocator.TempJob),
            PredictiveContactScheduleScratch = new NativeList<PredictiveContactScheduleEntry>(one, Allocator.TempJob),
            PredictiveContactScheduleCursor = new NativeReference<int>(Allocator.TempJob),
            InteractionCertificate = new NativeReference<InteractionCertificate>(Allocator.TempJob),
            InteractionViolations = new NativeList<InteractionCertificateViolation>(one, Allocator.TempJob),
            PersistentClassificationResults = new NativeList<PersistentPairClassificationResult>(math.max(unitCount * 8, 1), Allocator.TempJob),
            PersistentClassificationState = new NativeReference<PersistentClassificationPhaseState>(Allocator.TempJob),
            PersistentSpatialVisitStampByProxy = new NativeArray<uint>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            PersistentSpatialVisitStamp = new NativeReference<uint>(Allocator.TempJob),
            DirtyBodyRefreshResults = new NativeArray<DirtyBodyRefreshResult>(
                unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            DirtyBodyRefreshSummary =
                new NativeReference<DirtyBodyRefreshSummary>(Allocator.TempJob),
            DirtyContactScheduleBlockCounts =
                new NativeList<DirtyContactScheduleBlock>(
                    one, Allocator.TempJob),
            DirtyContactScheduleBlockOffsets =
                new NativeList<DirtyContactScheduleBlock>(
                    one, Allocator.TempJob),
#if RTS_CONTACT_DIAGNOSTICS
            PersistentClassificationTelemetry = new NativeReference<PersistentClassificationTelemetryState>(Allocator.TempJob),
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = Combine(combined, SweptCellEntries.Dispose(finalReader));
        combined = Combine(combined, BodyCellCounts.Dispose(finalReader));
        combined = Combine(combined, BodyCellOffsets.Dispose(finalReader));
        combined = Combine(combined, CellPairCounts.Dispose(finalReader));
        combined = Combine(combined, CellPairOffsets.Dispose(finalReader));
        combined = Combine(combined, FullSweepPrepared.Dispose(finalReader));
        combined = Combine(combined, CollisionPairs.Dispose(finalReader));
        combined = Combine(combined, TimestepContactPairs.Dispose(finalReader));
        combined = Combine(combined, PreviousTimestepContactPairs.Dispose(finalReader));
        combined = Combine(combined, TimestepInteractionPairs.Dispose(finalReader));
        combined = Combine(combined, SoftAvoidancePairs.Dispose(finalReader));
        combined = Combine(combined, ClassificationBodyPairs.Dispose(finalReader));
        combined = Combine(combined, CurrentBodyIndexByEntity.Dispose(finalReader));
        combined = Combine(combined, CurrentIncrementalProxies.Dispose(finalReader));
        combined = Combine(combined, IncrementalDirtyBodies.Dispose(finalReader));
        combined = Combine(combined, IncrementalDirtyFlagsByBody.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScratch.Dispose(finalReader));
        combined = Combine(combined, IncrementalNeighborPairScratch.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactSchedule.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScheduleScratch.Dispose(finalReader));
        combined = Combine(combined, PredictiveContactScheduleCursor.Dispose(finalReader));
        combined = Combine(combined, InteractionCertificate.Dispose(finalReader));
        combined = Combine(combined, InteractionViolations.Dispose(finalReader));
        if (PersistentClassificationResults.IsCreated)
            combined = Combine(combined, PersistentClassificationResults.Dispose(finalReader));
        if (PersistentClassificationState.IsCreated)
            combined = Combine(combined, PersistentClassificationState.Dispose(finalReader));
        combined = Combine(combined, PersistentSpatialVisitStampByProxy.Dispose(finalReader));
        combined = Combine(combined, PersistentSpatialVisitStamp.Dispose(finalReader));
        combined = Combine(combined, DirtyBodyRefreshResults.Dispose(finalReader));
        combined = Combine(combined, DirtyBodyRefreshSummary.Dispose(finalReader));
        combined = Combine(
            combined, DirtyContactScheduleBlockCounts.Dispose(finalReader));
        combined = Combine(
            combined, DirtyContactScheduleBlockOffsets.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        if (PersistentClassificationTelemetry.IsCreated)
            combined = Combine(combined, PersistentClassificationTelemetry.Dispose(finalReader));
#endif
        return combined;
    }

    private static JobHandle Combine(JobHandle a, JobHandle b) =>
        JobHandle.CombineDependencies(a, b);
}
}
