namespace RTS.Unit.FlowField.Jobs
{
internal static class CertificationKernelResources
{
    internal static CertificationStageKernel Compose(
        CertificationEnvironmentResources environment,
        CertificationBodyResources body,
        CertificationViewResources views,
        PersistentCertificationResources persistent,
        CertificationSolverResources solver,
        CertificationDiagnosticsResources diagnostics)
    {
        return new CertificationStageKernel
        {
            Configuration = environment.Configuration,
            GridOrigin = environment.GridOrigin,
            GridDimensions = environment.GridDimensions,
            CellRadius = environment.CellRadius,
            Grid = environment.Grid,
            Bodies = body.Bodies,
            NavigationStates = body.NavigationStates,
            MotionIntents = body.MotionIntents,
            MotionEvidence = body.MotionEvidence,
            StepStates = body.StepStates,
            SweptCellEntries = views.SweptCellEntries,
            BodyCellCounts = views.BodyCellCounts,
            BodyCellOffsets = views.BodyCellOffsets,
            CellPairCounts = views.CellPairCounts,
            CellPairOffsets = views.CellPairOffsets,
            FullSweepPrepared = views.FullSweepPrepared,
            Pairs = views.Pairs,
            TimestepContactPairs = views.TimestepContactPairs,
            PreviousTimestepContactPairs = views.PreviousTimestepContactPairs,
            TimestepInteractionPairs = views.TimestepInteractionPairs,
            SoftAvoidancePairs = views.SoftAvoidancePairs,
            ClassificationBodyPairs = views.ClassificationBodyPairs,
            CurrentBodyIndexByEntity = views.CurrentBodyIndexByEntity,
            CurrentIncrementalProxies = persistent.CurrentIncrementalProxies,
            PersistentSweptProxies = persistent.PersistentSweptProxies,
            PersistentProxyIndexByBody = persistent.PersistentProxyIndexByBody,
            PersistentNeighborPairs = persistent.PersistentNeighborPairs,
            PersistentPredictiveContacts = persistent.PersistentPredictiveContacts,
            PersistentContactIndex = persistent.PersistentContactIndex,
            PersistentActiveContactKeys = persistent.PersistentActiveContactKeys,
            PersistentSoftAvoidancePairKeys = persistent.PersistentSoftAvoidancePairKeys,
            PersistentDormantContactSchedule = persistent.PersistentDormantContactSchedule,
            PredictiveContactScratch = persistent.PredictiveContactScratch,
            IncrementalDirtyBodies = persistent.IncrementalDirtyBodies,
            IncrementalDirtyFlagsByBody = persistent.IncrementalDirtyFlagsByBody,
            IncrementalNeighborPairScratch = persistent.IncrementalNeighborPairScratch,
            PredictiveContactSchedule = persistent.PredictiveContactSchedule,
            PredictiveContactScheduleScratch = persistent.PredictiveContactScheduleScratch,
            PredictiveContactScheduleCursor = persistent.PredictiveContactScheduleCursor,
            IncrementalCacheState = persistent.IncrementalCacheState,
            InteractionCertificate = persistent.InteractionCertificate,
            InteractionCertificateViolations = persistent.InteractionCertificateViolations,
            PersistentClassificationResults = persistent.PersistentClassificationResults,
            PersistentClassificationState = persistent.PersistentClassificationState,
            PersistentSpatialMembership = persistent.PersistentSpatialMembership,
            PersistentSpatialMembershipEpoch = persistent.PersistentSpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = persistent.PersistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = persistent.PersistentSpatialVisitStamp,
            PersistentIncidentPairLookup = persistent.PersistentIncidentPairLookup,
            PersistentIncidentLookupEpoch = persistent.PersistentIncidentLookupEpoch,
            DirtyBodyRefreshResults = persistent.DirtyBodyRefreshResults,
            DirtyBodyRefreshSummary = persistent.DirtyBodyRefreshSummary,
            DirtyContactScheduleBlockCounts =
                persistent.DirtyContactScheduleBlockCounts,
            DirtyContactScheduleBlockOffsets =
                persistent.DirtyContactScheduleBlockOffsets,
            CorrectedBodyFlags = solver.CorrectedBodyFlags,
            CorrectedBodyIndices = solver.CorrectedBodyIndices,
            ParallelBodyStatistics = solver.ParallelBodyStatistics,
            EnvelopeEscapeFlags = solver.EnvelopeEscapeFlags,
            DirtyBodyBlockOffsets = solver.DirtyBodyBlockOffsets,
            ActiveIncidentIndexState = solver.ActiveIncidentIndexState,
            ActiveIncidentOffsets = solver.ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = solver.ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = solver.ActiveIncidentPairIndices,
            JacobiPairCorrections = solver.JacobiPairCorrections,
#if RTS_CONTACT_DIAGNOSTICS
            IterationState = diagnostics.IterationState,
            BlockStatistics = diagnostics.BlockStatistics,
            ParallelSimulationDebuggerPairCandidates =
                diagnostics.ParallelSimulationDebuggerPairCandidates,
            PersistentClassificationTelemetry =
                diagnostics.PersistentClassificationTelemetry,
            DiagnosticSelectedEntity = diagnostics.DiagnosticSelectedEntity,
            IncrementalOracleContactPairs = diagnostics.IncrementalOracleContactPairs,
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.Statistics,
            IterationDiagnostics = diagnostics.IterationDiagnostics,
            PairDiagnostics = diagnostics.PairDiagnostics,
            HeatSamples = diagnostics.HeatSamples,
#endif
        };
    }
}
}
