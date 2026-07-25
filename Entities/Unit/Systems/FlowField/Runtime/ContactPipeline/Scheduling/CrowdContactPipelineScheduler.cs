using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Managed scheduling composition only. It is never scheduled as a job and owns no
/// algorithm implementation; each scheduled stage carries its NativeContainers as
/// direct fields so Collections Safety sees the real capability boundary.
/// </summary>
public partial struct CrowdContactPipelineScheduler
{
    internal const int ParallelBodyBatchSize = 64;
    internal const int SoftPairBatchSize = 64;
    internal const int JacobiPairBatchSize = 64;

    public ContactPipelineConfiguration Configuration;
    public ContactPipelineLifecycleJob Lifecycle;
    public InteractionCertificationJob Certification;
    public MotionIntegrationJob Motion;
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

    public JobHandle ScheduleSerial(JobHandle dependency)
    {
        ContactPipelineLifecycleJob lifecycle = Lifecycle;
        lifecycle.Operation = ContactPipelineLifecycleOperation.InitializeSerial;
        JobHandle handle = lifecycle.Schedule(dependency);

        InteractionCertificationJob certification = Certification;
        certification.Operation = InteractionCertificationOperation.InitializeSerial;
        handle = certification.Schedule(handle);
        certification.Operation = InteractionCertificationOperation.BuildInitialSerial;
        handle = certification.Schedule(handle);

        int substeps = math.max(1, SubstepCount);
        int iterations = math.max(1, IterationCount);
        for (int substep = 0; substep < substeps; substep++)
        {
            MotionIntegrationJob motion = Motion;
            motion.Operation = MotionIntegrationOperation.PrepareBaseVelocity;
            handle = motion.Schedule(handle);

            certification = Certification;
            certification.SubstepIndex = substep;
            certification.Operation = InteractionCertificationOperation.BuildSubstepInteractionSerial;
            handle = certification.Schedule(handle);
            certification.Operation = InteractionCertificationOperation.ValidateBaseMotionSerial;
            handle = certification.Schedule(handle);

            SoftAvoidanceJob soft = SoftAvoidance;
            soft.Operation = SoftAvoidanceOperation.SolveSerial;
            handle = soft.Schedule(handle);

            certification = Certification;
            certification.SubstepIndex = substep;
            certification.Operation = InteractionCertificationOperation.ClampSoftOutputSerial;
            handle = certification.Schedule(handle);

            motion = Motion;
            motion.Operation = MotionIntegrationOperation.PredictUnconstrained;
            handle = motion.Schedule(handle);

            certification = Certification;
            certification.SubstepIndex = substep;
            certification.Operation = InteractionCertificationOperation.ValidatePredictedAndActivateSerial;
            handle = certification.Schedule(handle);

            for (int iteration = 0; iteration < iterations; iteration++)
            {
                ConstraintSolverJob solver = ConstraintSolver;
                solver.SubstepIndex = substep;
                solver.IterationIndex = iteration;
                solver.Operation = ConstraintSolverOperation.SolveWallSerial;
                handle = solver.Schedule(handle);

                certification = Certification;
                certification.SubstepIndex = substep;
                certification.AfterContact = 0;
                certification.Operation = InteractionCertificationOperation.ValidateSolverCorrectionSerial;
                handle = certification.Schedule(handle);

                solver = ConstraintSolver;
                solver.SubstepIndex = substep;
                solver.IterationIndex = iteration;
                solver.Operation = ConstraintSolverOperation.SolveContactSerial;
                handle = solver.Schedule(handle);

                certification = Certification;
                certification.SubstepIndex = substep;
                certification.AfterContact = 1;
                certification.IsLastIteration = (byte)(iteration == iterations - 1 ? 1 : 0);
                certification.Operation = InteractionCertificationOperation.ValidateSolverCorrectionSerial;
                handle = certification.Schedule(handle);

                solver = ConstraintSolver;
                solver.SubstepIndex = substep;
                solver.Operation = ConstraintSolverOperation.SolveRecoverySerial;
                handle = solver.Schedule(handle);
            }

            ConstraintSolverJob finalizeSubstep = ConstraintSolver;
            finalizeSubstep.Operation = ConstraintSolverOperation.FinalizeSerialSubstep;
            handle = finalizeSubstep.Schedule(handle);

            MotionIntegrationJob reconstruct = Motion;
            reconstruct.Operation = MotionIntegrationOperation.ReconstructVelocity;
            handle = reconstruct.Schedule(handle);
        }

        ConstraintSolverJob finalize = ConstraintSolver;
        finalize.Operation = ConstraintSolverOperation.FinalizeSerialPipeline;
        return finalize.Schedule(handle);
    }
}
}
