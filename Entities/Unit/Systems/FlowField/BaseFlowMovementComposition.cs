using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// Solver ABI composition boundary. BaseFlowMovementSystem.OnUpdate owns stage
/// order and JobHandle dependencies; this file alone expands coarse resource
/// owners into the historical solver fields while the internal stages are
/// progressively narrowed.
/// </summary>
public abstract partial class BaseFlowMovementSystem
{
    private SolveXpbdUnitContactsJob ComposeContactSolverJob(
        ContactPipelineConfiguration configuration,
        FlowFieldGrid gridComponent,
        ContactFrameResources frame,
        ContactDiagnosticsFrameResources diagnostics,
        Entity diagnosticSelectedEntity,
        SimulationDebuggerCaptureMask captureMask,
        int maximumVisualizedPairs)
    {
        return new SolveXpbdUnitContactsJob
        {
            Configuration = configuration,
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Grid = gridComponent.Grid,

            SweptCellEntries = frame.SweptCellEntries,
            Pairs = frame.CollisionPairs,
            TimestepContactPairs = frame.TimestepContactPairs,
            CurrentBodyIndexByEntity = frame.CurrentBodyIndexByEntity,
            PreviousTimestepContactPairs = frame.PreviousTimestepContactPairs,
            TimestepInteractionPairs = frame.TimestepInteractionPairs,
            SoftAvoidancePairs = frame.SoftAvoidancePairs,
            PredictiveContactSchedule = frame.PredictiveContactSchedule,
            PredictiveContactScheduleScratch = frame.PredictiveContactScheduleScratch,
            PredictiveContactScheduleCursor = frame.PredictiveContactScheduleCursor,
            InteractionCertificate = frame.InteractionCertificate,
            InteractionCertificateViolations = frame.InteractionViolations,

            CorrectedBodyFlags = frame.CorrectedBodyFlags,
            CorrectedBodyIndices = frame.CorrectedBodyIndices,
            ActiveIncidentOffsets = frame.ActiveIncidentOffsets,
            ActiveIncidentWriteCursors = frame.ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices = frame.ActiveIncidentPairIndices,
            JacobiPairCorrections = frame.JacobiPairCorrections,
            EnvelopeEscapeFlags = frame.EnvelopeEscapeFlags,
            ParallelBodyStatistics = frame.ParallelBodyResults,
            DirtyBodyBlockOffsets = frame.DirtyBodyBlockOffsets,
            SoftIncidentOffsets = frame.SoftIncidentOffsets,
            SoftIncidentWriteCursors = frame.SoftIncidentWriteCursors,
            SoftIncidentPairIndices = frame.SoftIncidentPairIndices,
            SoftPairContributions = frame.SoftPairContributions,
            ActiveIncidentIndexState = frame.ActiveIncidentIndexState,

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
            IncrementalCacheState = _persistentState.CacheState,
            PersistentIncidentPairLookup = _persistentState.IncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentState.IncidentLookupEpoch,
            PersistentSpatialMembership = _persistentState.SpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentState.SpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = frame.PersistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = frame.PersistentSpatialVisitStamp,
            PersistentClassificationResults = frame.PersistentClassificationResults,
            PersistentClassificationState = frame.PersistentClassificationState,

#if RTS_CONTACT_DIAGNOSTICS
            PersistentClassificationTelemetry = frame.PersistentClassificationTelemetry,
#endif
            IncrementalOracleContactPairs = diagnostics.IncrementalOracleContactPairs,
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

            States = frame.States
        };
    }
}
}
