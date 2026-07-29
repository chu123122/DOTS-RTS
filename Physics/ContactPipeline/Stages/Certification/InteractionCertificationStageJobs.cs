using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationAlgorithms
{
    [BurstCompile]
    public struct PreparePersistentClassificationJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public void Execute() => Create(Environment, Body, Views, Persistent, default, Diagnostics)
            .PreparePersistentClassification(RuntimeState);
    }

    [BurstCompile]
    public struct CommitPersistentClassificationJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public void Execute() => Create(Environment, Body, Views, Persistent, default, Diagnostics)
            .CommitPersistentClassification(RuntimeState);
    }

    [BurstCompile]
    public struct BuildInitialContactSetJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public void Execute() => Create(Environment, Body, Views, Persistent, default, Diagnostics)
            .BuildInitialContactSet(RuntimeState);
    }

    [BurstCompile]
    public struct FinalizeEnvelopeEscapesJob : IJob
    {
        public ContactPipelineConfiguration Configuration;
        [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
        [ReadOnly] public NativeArray<ParallelBodyStageResult> BodyStatistics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
#if RTS_CONTACT_DIAGNOSTICS
        public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
        public NativeReference<PredictiveDiscContactStatistics> Statistics;
#endif
        public void Execute()
        {
            var algorithms = new InteractionCertificationAlgorithms
            {
                Environment = new CertificationEnvironmentResources { Configuration = Configuration },
                Persistent = new PersistentCertificationResources { IncrementalDirtyBodies = DirtyBodies },
                Solver = new CertificationSolverResources { ParallelBodyStatistics = BodyStatistics },
#if RTS_CONTACT_DIAGNOSTICS
                Diagnostics = new CertificationDiagnosticsResources
                {
                    IncrementalStatistics = IncrementalStatistics,
                    Statistics = Statistics
                }
#endif
            };
            algorithms.FinalizeEnvelopeEscapes(SubstepIndex, RuntimeState);
        }
    }

    [BurstCompile]
    public struct PrepareSubstepRepairJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationSolverResources Solver;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Create(Environment, Body, Views, Persistent, Solver, Diagnostics)
            .PrepareSubstepRepairClassification(SubstepIndex, RuntimeState);
    }

    [BurstCompile]
    public struct CommitSubstepRepairJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationSolverResources Solver;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Create(Environment, Body, Views, Persistent, Solver, Diagnostics)
            .CommitSubstepRepairClassification(SubstepIndex, RuntimeState);
    }

    [BurstCompile]
    public struct FinalizePreparedSubstepJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationSolverResources Solver;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
        public void Execute() => Create(Environment, Body, Views, Persistent, Solver, Diagnostics)
            .FinalizePreparedSubstep(SubstepIndex, RuntimeState);
    }

    [BurstCompile]
    public struct ValidateConsumerViewsJob : IJob
    {
        public ContactPipelineConfiguration Configuration;
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
        [ReadOnly] public NativeList<ContactConstraint> TimestepContactPairs;
        [ReadOnly] public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
        public NativeReference<InteractionCertificate> InteractionCertificate;
        public NativeList<InteractionCertificateViolation> InteractionCertificateViolations;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
#if RTS_CONTACT_DIAGNOSTICS
        public NativeReference<PredictiveDiscContactStatistics> Statistics;
#endif
        public void Execute()
        {
            var algorithms = new InteractionCertificationAlgorithms
            {
                Environment = new CertificationEnvironmentResources { Configuration = Configuration },
                Body = new CertificationBodyResources { Bodies = Bodies },
                Views = new CertificationViewResources
                {
                    SoftAvoidancePairs = SoftAvoidancePairs,
                    TimestepContactPairs = TimestepContactPairs
                },
                Persistent = new PersistentCertificationResources
                {
                    PredictiveContactSchedule = PredictiveContactSchedule,
                    InteractionCertificate = InteractionCertificate,
                    InteractionCertificateViolations = InteractionCertificateViolations
                },
#if RTS_CONTACT_DIAGNOSTICS
                Diagnostics = new CertificationDiagnosticsResources { Statistics = Statistics }
#endif
            };
            algorithms.ValidateConsumerViews(SubstepIndex, RuntimeState);
        }
    }

    [BurstCompile]
    public struct FinalizeWallIterationJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationSolverResources Solver;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
        public int BodyBlockCount;
        public void Execute()
        {
            var algorithms = Create(Environment, Body, Views, Persistent, Solver, Diagnostics);
            algorithms.FinalizeWallIteration(SubstepIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                , Diagnostics.IterationState, Diagnostics.BlockStatistics
#endif
                , BodyBlockCount);
        }
    }

    [BurstCompile]
    public struct FinalizeContactIterationJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationSolverResources Solver;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public int SubstepIndex;
        public int IterationIndex;
        public void Execute()
        {
            var algorithms = Create(Environment, Body, Views, Persistent, Solver, Diagnostics);
            algorithms.FinalizeContactIteration(SubstepIndex, IterationIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                , Diagnostics.IterationState, Diagnostics.BlockStatistics
#endif
            );
        }
    }

    private static InteractionCertificationAlgorithms Create(
        CertificationEnvironmentResources environment,
        CertificationBodyResources body,
        CertificationViewResources views,
        PersistentCertificationResources persistent,
        CertificationSolverResources solver,
        CertificationDiagnosticsResources diagnostics) =>
        new InteractionCertificationAlgorithms
        {
            Environment = environment,
            Body = body,
            Views = views,
            Persistent = persistent,
            Solver = solver,
            Diagnostics = diagnostics
        };

    internal PreparePersistentClassificationJob CreatePreparePersistentClassificationJob(
        NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new PreparePersistentClassificationJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Diagnostics = Diagnostics, RuntimeState = runtimeState };

    internal CommitPersistentClassificationJob CreateCommitPersistentClassificationJob(
        NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new CommitPersistentClassificationJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Diagnostics = Diagnostics, RuntimeState = runtimeState };

    internal BuildInitialContactSetJob CreateBuildInitialContactSetJob(
        NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new BuildInitialContactSetJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Diagnostics = Diagnostics, RuntimeState = runtimeState };

    internal FinalizeEnvelopeEscapesJob CreateFinalizeEnvelopeEscapesJob(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new FinalizeEnvelopeEscapesJob
        {
            Configuration = Configuration,
            DirtyBodies = IncrementalDirtyBodies,
            BodyStatistics = ParallelBodyStatistics,
            RuntimeState = runtimeState,
            SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = IncrementalStatistics,
            Statistics = Statistics
#endif
        };

    internal PrepareSubstepRepairJob CreatePrepareSubstepRepairJob(int substepIndex, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new PrepareSubstepRepairJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Solver = Solver, Diagnostics = Diagnostics, RuntimeState = runtimeState, SubstepIndex = substepIndex };

    internal CommitSubstepRepairJob CreateCommitSubstepRepairJob(int substepIndex, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new CommitSubstepRepairJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Solver = Solver, Diagnostics = Diagnostics, RuntimeState = runtimeState, SubstepIndex = substepIndex };

    internal FinalizePreparedSubstepJob CreateFinalizePreparedSubstepJob(int substepIndex, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new FinalizePreparedSubstepJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Solver = Solver, Diagnostics = Diagnostics, RuntimeState = runtimeState, SubstepIndex = substepIndex };

    internal ValidateConsumerViewsJob CreateValidateConsumerViewsJob(int substepIndex, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new ValidateConsumerViewsJob
        {
            Configuration = Configuration,
            Bodies = Bodies,
            SoftAvoidancePairs = SoftAvoidancePairs,
            TimestepContactPairs = TimestepContactPairs,
            PredictiveContactSchedule = PredictiveContactSchedule,
            InteractionCertificate = InteractionCertificate,
            InteractionCertificateViolations = InteractionCertificateViolations,
            RuntimeState = runtimeState,
            SubstepIndex = substepIndex,
#if RTS_CONTACT_DIAGNOSTICS
            Statistics = Statistics
#endif
        };

    internal FinalizeWallIterationJob CreateFinalizeWallIterationJob(int substepIndex, int bodyBlockCount, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new FinalizeWallIterationJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Solver = Solver, Diagnostics = Diagnostics, RuntimeState = runtimeState, SubstepIndex = substepIndex, BodyBlockCount = bodyBlockCount };

    internal FinalizeContactIterationJob CreateFinalizeContactIterationJob(int substepIndex, int iterationIndex, NativeReference<ContactPipelineExecutionState> runtimeState) =>
        new FinalizeContactIterationJob { Environment = Environment, Body = Body, Views = Views, Persistent = Persistent, Solver = Solver, Diagnostics = Diagnostics, RuntimeState = runtimeState, SubstepIndex = substepIndex, IterationIndex = iterationIndex };
}
}
