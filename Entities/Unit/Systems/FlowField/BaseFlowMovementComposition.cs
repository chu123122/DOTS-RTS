using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// Temporary assembly boundary until the resource-owner cutover. It constructs
/// capability-limited stage jobs; no scheduled job receives the whole pipeline.
/// </summary>
public abstract partial class BaseFlowMovementSystem
{
    private CrowdContactPipelineScheduler ComposeContactPipelineScheduler(
        ContactPipelineConfiguration configuration,
        FlowFieldGrid gridComponent,
        ContactFrameResources frame,
        ContactDiagnosticsFrameResources diagnostics,
        Entity diagnosticSelectedEntity,
        SimulationDebuggerCaptureMask captureMask,
        int maximumVisualizedPairs)
    {
        var lifecycle = new ContactPipelineLifecycleJob
        {
            Configuration = configuration,
            SerialControl = frame.SerialControlState,
            PersistentSweptProxies = _persistentState.SweptProxies,
            PersistentProxyIndexByBody = _persistentState.ProxyIndexByBody,
            PersistentNeighborPairs = _persistentState.NeighborPairs,
            PersistentPredictiveContacts = _persistentState.PredictiveContacts,
            PersistentSpatialMembership = _persistentState.SpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentState.SpatialMembershipEpoch,
            PersistentIncidentPairLookup = _persistentState.IncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentState.IncidentLookupEpoch,
            IncrementalCacheState = _persistentState.CacheState,
            ActiveIncidentIndexState = frame.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            SelectedBodyDiagnostic = diagnostics.SelectedBody,
            SimulationDebuggerSelectedPairs = _simulationDebuggerSelectedPairs,
#endif
        };

        var certification = new InteractionCertificationJob
        {
            Configuration = configuration,
            SerialControl = frame.SerialControlState,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Grid = gridComponent.Grid,
            Bodies = frame.Bodies,
            NavigationStates = frame.NavigationStates,
            MotionIntents = frame.MotionIntents,
            MotionEvidence = frame.MotionEvidence,
            StepStates = frame.StepStates,
            SweptCellEntries = frame.SweptCellEntries,
            Pairs = frame.CollisionPairs,
            TimestepContactPairs = frame.TimestepContactPairs,
            PreviousTimestepContactPairs = frame.PreviousTimestepContactPairs,
            TimestepInteractionPairs = frame.TimestepInteractionPairs,
            SoftAvoidancePairs = frame.SoftAvoidancePairs,
            ClassificationBodyPairs = frame.ClassificationBodyPairs,
            CurrentBodyIndexByEntity = frame.CurrentBodyIndexByEntity,
            CurrentIncrementalProxies = frame.CurrentIncrementalProxies,
            PersistentSweptProxies = _persistentState.SweptProxies,
            PersistentProxyIndexByBody = _persistentState.ProxyIndexByBody,
            PersistentNeighborPairs = _persistentState.NeighborPairs,
            PersistentPredictiveContacts = _persistentState.PredictiveContacts,
            PersistentActiveContactKeys = _persistentState.ActiveContactKeys,
            PersistentSoftAvoidancePairKeys = _persistentState.SoftAvoidancePairKeys,
            PersistentDormantContactSchedule = _persistentState.DormantContactSchedule,
            PredictiveContactScratch = frame.PredictiveContactScratch,
            IncrementalDirtyBodies = frame.IncrementalDirtyBodies,
            IncrementalDirtyFlagsByBody = frame.IncrementalDirtyFlagsByBody,
            IncrementalNeighborPairScratch = frame.IncrementalNeighborPairScratch,
            PredictiveContactSchedule = frame.PredictiveContactSchedule,
            PredictiveContactScheduleScratch = frame.PredictiveContactScheduleScratch,
            PredictiveContactScheduleCursor = frame.PredictiveContactScheduleCursor,
            IncrementalCacheState = _persistentState.CacheState,
            InteractionCertificate = frame.InteractionCertificate,
            InteractionCertificateViolations = frame.InteractionViolations,
            CorrectedBodyFlags = frame.CorrectedBodyFlags,
            CorrectedBodyIndices = frame.CorrectedBodyIndices,
            ParallelBodyStatistics = frame.ParallelBodyResults,
            EnvelopeEscapeFlags = frame.EnvelopeEscapeFlags,
            DirtyBodyBlockOffsets = frame.DirtyBodyBlockOffsets,
            ActiveIncidentIndexState = frame.ActiveIncidentIndexState,
            ActiveIncidentOffsets = frame.ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = frame.ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = frame.ActiveIncidentPairIndices,
            JacobiPairCorrections = frame.JacobiPairCorrections,
            PersistentClassificationResults = frame.PersistentClassificationResults,
            PersistentClassificationState = frame.PersistentClassificationState,
            PersistentSpatialMembership = _persistentState.SpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentState.SpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = frame.PersistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = frame.PersistentSpatialVisitStamp,
            PersistentIncidentPairLookup = _persistentState.IncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentState.IncidentLookupEpoch,
#if RTS_CONTACT_DIAGNOSTICS
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            PersistentClassificationTelemetry = frame.PersistentClassificationTelemetry,
            IncrementalOracleContactPairs = diagnostics.IncrementalOracleContactPairs,
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            HeatSamples = diagnostics.HeatSamples,
            ParallelSimulationDebuggerPairCandidates = diagnostics.ParallelPairCandidates,
#endif
        };

        var motion = new MotionIntegrationJob
        {
            Configuration = configuration,
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Bodies = frame.Bodies,
            NavigationStates = frame.NavigationStates,
            MotionIntents = frame.MotionIntents,
            StepStates = frame.StepStates,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = diagnostics.ContactStatistics,
#endif
        };

        var softAvoidance = new SoftAvoidanceJob
        {
            Configuration = configuration,
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Bodies = frame.Bodies,
            StepStates = frame.StepStates,
            SoftAvoidancePairs = frame.SoftAvoidancePairs,
            SoftIncidentOffsets = frame.SoftIncidentOffsets,
            SoftIncidentWriteCursors = frame.SoftIncidentWriteCursors,
            SoftIncidentPairIndices = frame.SoftIncidentPairIndices,
            SoftPairContributions = frame.SoftPairContributions,
            ActiveIncidentIndexState = frame.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            BlockStatistics = frame.ParallelJacobiBlockTelemetry,
            EscapeCountsByBlock = frame.DirtyBodyBlockOffsets,
#endif
        };

        var constraintSolver = new ConstraintSolverJob
        {
            Configuration = configuration,
            SerialControl = frame.SerialControlState,
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Bodies = frame.Bodies,
            NavigationStates = frame.NavigationStates,
            MotionIntents = frame.MotionIntents,
            MotionEvidence = frame.MotionEvidence,
            StepStates = frame.StepStates,
            TimestepContactPairs = frame.TimestepContactPairs,
            CorrectedBodyFlags = frame.CorrectedBodyFlags,
            CorrectedBodyIndices = frame.CorrectedBodyIndices,
            ActiveIncidentOffsets = frame.ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = frame.ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = frame.ActiveIncidentPairIndices,
            JacobiPairCorrections = frame.JacobiPairCorrections,
            ActiveIncidentIndexState = frame.ActiveIncidentIndexState,
            ParallelBodyStatistics = frame.ParallelBodyResults,
#if RTS_CONTACT_DIAGNOSTICS
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            SelectedBodyDiagnostic = diagnostics.SelectedBody,
            HeatSamples = diagnostics.HeatSamples,
            SimulationDebuggerCaptureMask = captureMask,
            SimulationDebuggerMaximumPairs = maximumVisualizedPairs,
            SimulationDebuggerSelectedPairs = _simulationDebuggerSelectedPairs,
            ParallelSimulationDebuggerPairCandidates = diagnostics.ParallelPairCandidates,
            ParallelSimulationDebuggerPairScratch = diagnostics.ParallelPairScratch,
            SimulationDebuggerSelectedUnit = _simulationDebuggerSelectedUnit,
            SimulationDebuggerSelectedUnitValid = _simulationDebuggerSelectedUnitValid,
#endif
        };

        return new CrowdContactPipelineScheduler
        {
            Configuration = configuration,
            Lifecycle = lifecycle,
            Certification = certification,
            Motion = motion,
            SoftAvoidance = softAvoidance,
            ConstraintSolver = constraintSolver
        };
    }
}
}
