using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 仅做托管调度组合：本身不作为 Job 调度，也不实现算法；
/// 每个调度的阶段把 NativeContainer 作为直接字段携带，
/// Collections Safety 看得到真实容器边界。
/// </summary>
public partial struct CrowdContactPipelineScheduler
{
    internal const int ParallelBodyBatchSize = 64;
    internal const int SoftPairBatchSize = 64;
    internal const int JacobiPairBatchSize = 64;

    public ContactPipelineConfiguration Configuration;
    public ContactPipelineLifecycleJob Lifecycle;
    public InteractionCertificationJob Certification;
    public SoftAvoidanceJob SoftAvoidance;
    public ConstraintSolverJob ConstraintSolver;

    private float DeltaTime => Configuration.DeltaTime;
    private int SubstepCount => Configuration.SubstepCount;
    private int IterationCount => Configuration.IterationCount;
    private float Compliance => Configuration.Compliance;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private bool EnablePredictivePairGeneration => Configuration.EnablePredictivePairGeneration;
    private bool EnablePredictiveContacts => Configuration.EnablePredictiveContacts;
    private bool EnableTimestepContactSetCache => Configuration.EnableTimestepContactSetCache;
    private float PredictiveSkin => Configuration.PredictiveSkin;
    private float TimestepContactMargin => Configuration.TimestepContactMargin;
    private float SoftAvoidanceShell => Configuration.SoftAvoidanceShell;
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver => Configuration.SoftAvoidanceVelocitySolver;
    private float RvoTimeHorizon => Configuration.RvoTimeHorizon;

    private NativeArray<CrowdBodySnapshot> Bodies => Certification.Bodies;
    private NativeArray<CrowdNavigationState> NavigationStates => Certification.NavigationStates;
    private NativeArray<CrowdMotionIntent> MotionIntents => Certification.MotionIntents;
    private NativeArray<CrowdMotionEvidence> MotionEvidence => Certification.MotionEvidence;
    private NativeArray<CrowdBodyStepState> StepStates => Certification.StepStates;
    private NativeArray<FlowFieldCell> Grid => ConstraintSolver.Grid;
    private float3 GridOrigin => ConstraintSolver.GridOrigin;
    private int2 GridDimensions => ConstraintSolver.GridDimensions;
    private float CellRadius => ConstraintSolver.CellRadius;

    private NativeList<ContactConstraint> Pairs => Certification.Pairs;
    private NativeList<ContactConstraint> TimestepContactPairs => Certification.TimestepContactPairs;
    private NativeList<ContactConstraint> PreviousTimestepContactPairs => Certification.PreviousTimestepContactPairs;
    private NativeList<BodyPair> TimestepInteractionPairs => Certification.TimestepInteractionPairs;
    private NativeList<BodyPair> SoftAvoidancePairs => Certification.SoftAvoidancePairs;
    private NativeList<BodyPair> ClassificationBodyPairs => Certification.ClassificationBodyPairs;
    private NativeList<PersistentSweptProxy> CurrentIncrementalProxies => Certification.CurrentIncrementalProxies;
    private NativeList<PersistentSweptProxy> PersistentSweptProxies => Certification.PersistentSweptProxies;
    private NativeList<int> PersistentProxyIndexByBody => Certification.PersistentProxyIndexByBody;
    private NativeList<PersistentNeighborPair> PersistentNeighborPairs => Certification.PersistentNeighborPairs;
    private NativeList<PersistentPredictiveContact> PersistentPredictiveContacts => Certification.PersistentPredictiveContacts;
    private NativeList<IncrementalDirtyBody> IncrementalDirtyBodies => Certification.IncrementalDirtyBodies;
    private NativeArray<byte> IncrementalDirtyFlagsByBody => Certification.IncrementalDirtyFlagsByBody;
    private NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule => Certification.PredictiveContactSchedule;
    private NativeReference<IncrementalContactCacheState> IncrementalCacheState => Certification.IncrementalCacheState;
    private NativeList<PersistentPairClassificationResult> PersistentClassificationResults => Certification.PersistentClassificationResults;
    private NativeReference<PersistentClassificationPhaseState> PersistentClassificationState => Certification.PersistentClassificationState;

    private NativeArray<byte> CorrectedBodyFlags => Certification.CorrectedBodyFlags;
    private NativeList<int> CorrectedBodyIndices => Certification.CorrectedBodyIndices;
    private NativeArray<byte> EnvelopeEscapeFlags => Certification.EnvelopeEscapeFlags;
    private NativeArray<ParallelBodyStageResult> ParallelBodyStatistics => Certification.ParallelBodyStatistics;
    private NativeArray<int> DirtyBodyBlockOffsets => Certification.DirtyBodyBlockOffsets;
    private NativeArray<int> SoftIncidentOffsets => SoftAvoidance.SoftIncidentOffsets;
    private NativeArray<int> SoftIncidentWriteCursors => SoftAvoidance.SoftIncidentWriteCursors;
    private NativeList<int> SoftIncidentPairIndices => SoftAvoidance.SoftIncidentPairIndices;
    private NativeList<SoftAvoidancePairContribution> SoftPairContributions => SoftAvoidance.SoftPairContributions;
    private NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState => Certification.ActiveIncidentIndexState;
    private NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup => Certification.PersistentIncidentPairLookup;
    private NativeReference<uint> PersistentIncidentLookupEpoch => Certification.PersistentIncidentLookupEpoch;
    private NativeArray<int> ActiveIncidentOffsets => Certification.ActiveIncidentOffsets;
    private NativeArray<int> ActiveIncidentWriteCursors => Certification.ActiveIncidentWriteCursors;
    private NativeList<int> ActiveIncidentPairIndices => Certification.ActiveIncidentPairIndices;
    private NativeList<JacobiPairCorrection> JacobiPairCorrections => ConstraintSolver.JacobiPairCorrections;

#if RTS_CONTACT_DIAGNOSTICS
    private NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics => Certification.IncrementalStatistics;
    private NativeReference<PredictiveDiscContactStatistics> Statistics => Certification.Statistics;
    private Entity DiagnosticSelectedEntity => ConstraintSolver.DiagnosticSelectedEntity;
    private SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask => ConstraintSolver.SimulationDebuggerCaptureMask;
    private NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs => ConstraintSolver.SimulationDebuggerSelectedPairs;
    private NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates => ConstraintSolver.ParallelSimulationDebuggerPairCandidates;
    private NativeList<SimulationDebuggerPairSample> ParallelSimulationDebuggerPairScratch => ConstraintSolver.ParallelSimulationDebuggerPairScratch;
#endif

}
}
