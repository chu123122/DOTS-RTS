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
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
#endif

    public static InteractionCertificationFrameResources Create(int unitCount)
    {
        int one = math.max(unitCount, 1);
        return new InteractionCertificationFrameResources
        {
            SweptCellEntries = new NativeList<SweptDiscCellEntry>(math.max(unitCount * 4, 1), Allocator.TempJob),
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
#if RTS_CONTACT_DIAGNOSTICS
            PersistentClassificationTelemetry = new NativeReference<PersistentClassificationTelemetryState>(Allocator.TempJob),
#endif
        };
    }

    public InteractionCertificationAlgorithms CreateAlgorithms(
        ContactPipelineConfiguration configuration,
        FlowFieldGrid grid,
        CrowdStepBodyResources body,
        InteractionCandidateStore candidates,
        ConstraintSolverFrameResources solver,
        ContactPipelineExecutionResources execution,
        ContactDiagnosticsFrameResources diagnostics,
        Entity diagnosticSelectedEntity)
    {
        return new InteractionCertificationAlgorithms
        {
            Environment = new CertificationEnvironmentResources
            {
                Configuration = configuration, GridOrigin = grid.GridOrigin,
                GridDimensions = grid.GridDimensions, CellRadius = grid.CellRadius, Grid = grid.Grid
            },
            Body = new CertificationBodyResources
            {
                Bodies = body.Bodies, NavigationStates = body.NavigationStates,
                MotionIntents = body.MotionIntents, MotionEvidence = body.MotionEvidence,
                StepStates = body.StepStates
            },
            Views = new CertificationViewResources
            {
                SweptCellEntries = SweptCellEntries, Pairs = CollisionPairs,
                TimestepContactPairs = TimestepContactPairs,
                PreviousTimestepContactPairs = PreviousTimestepContactPairs,
                TimestepInteractionPairs = TimestepInteractionPairs,
                SoftAvoidancePairs = SoftAvoidancePairs,
                ClassificationBodyPairs = ClassificationBodyPairs,
                CurrentBodyIndexByEntity = CurrentBodyIndexByEntity
            },
            Persistent = new PersistentCertificationResources
            {
                CurrentIncrementalProxies = CurrentIncrementalProxies,
                PersistentSweptProxies = candidates.SweptProxies,
                PersistentProxyIndexByBody = candidates.ProxyIndexByBody,
                PersistentNeighborPairs = candidates.NeighborPairs,
                PersistentPredictiveContacts = candidates.PredictiveContacts,
                PersistentContactIndex = candidates.PredictiveContactIndex,
                PersistentActiveContactKeys = candidates.ActiveContactKeys,
                PersistentSoftAvoidancePairKeys = candidates.SoftAvoidancePairKeys,
                PersistentDormantContactSchedule = candidates.DormantContactSchedule,
                PredictiveContactScratch = PredictiveContactScratch,
                IncrementalDirtyBodies = IncrementalDirtyBodies,
                IncrementalDirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                IncrementalNeighborPairScratch = IncrementalNeighborPairScratch,
                PredictiveContactSchedule = PredictiveContactSchedule,
                PredictiveContactScheduleScratch = PredictiveContactScheduleScratch,
                PredictiveContactScheduleCursor = PredictiveContactScheduleCursor,
                IncrementalCacheState = candidates.CacheState,
                InteractionCertificate = InteractionCertificate,
                InteractionCertificateViolations = InteractionViolations,
                PersistentClassificationResults = PersistentClassificationResults,
                PersistentClassificationState = PersistentClassificationState,
                PersistentSpatialMembership = candidates.SpatialMembership,
                PersistentSpatialMembershipEpoch = candidates.SpatialMembershipEpoch,
                PersistentSpatialVisitStampByProxy = PersistentSpatialVisitStampByProxy,
                PersistentSpatialVisitStamp = PersistentSpatialVisitStamp,
                PersistentIncidentPairLookup = candidates.IncidentPairLookup,
                PersistentIncidentLookupEpoch = candidates.IncidentLookupEpoch
            },
            Solver = new CertificationSolverResources
            {
                CorrectedBodyFlags = solver.CorrectedBodyFlags,
                CorrectedBodyIndices = solver.CorrectedBodyIndices,
                ParallelBodyStatistics = solver.ParallelBodyResults,
                EnvelopeEscapeFlags = solver.EnvelopeEscapeFlags,
                DirtyBodyBlockOffsets = solver.DirtyBodyBlockOffsets,
                ActiveIncidentIndexState = solver.ActiveIncidentIndexState,
                ActiveIncidentOffsets = solver.ActiveIncidentOffsets,
                ActiveIncidentWriteCursors = solver.ActiveIncidentWriteCursors,
                ActiveIncidentPairIndices = solver.ActiveIncidentPairIndices,
                JacobiPairCorrections = solver.JacobiPairCorrections
            },
#if RTS_CONTACT_DIAGNOSTICS
            Diagnostics = new CertificationDiagnosticsResources
            {
                IterationState = execution.SolverIterationState,
                BlockStatistics = execution.JacobiBlockStatistics,
                DiagnosticSelectedEntity = diagnosticSelectedEntity,
                PersistentClassificationTelemetry = PersistentClassificationTelemetry,
                IncrementalOracleContactPairs = diagnostics.IncrementalOracleContactPairs,
                IncrementalStatistics = diagnostics.IncrementalStatistics,
                Statistics = diagnostics.ContactStatistics,
                IterationDiagnostics = diagnostics.Iterations,
                PairDiagnostics = diagnostics.Pairs,
                HeatSamples = diagnostics.HeatSamples,
                ParallelSimulationDebuggerPairCandidates = diagnostics.ParallelPairCandidates
            }
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = Combine(combined, SweptCellEntries.Dispose(finalReader));
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
