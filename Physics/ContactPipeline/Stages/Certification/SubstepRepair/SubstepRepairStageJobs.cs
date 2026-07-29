using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
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
            var kernel = new CertificationStageKernel
            {
                Configuration = Configuration,
                IncrementalDirtyBodies = DirtyBodies,
                ParallelBodyStatistics = BodyStatistics,
#if RTS_CONTACT_DIAGNOSTICS
                IncrementalStatistics = IncrementalStatistics,
                Statistics = Statistics
#endif
            };
            kernel.FinalizeEnvelopeEscapes(SubstepIndex, RuntimeState);
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
        public void Execute() => CertificationKernelResources.Compose(Environment, Body, Views, Persistent, Solver, Diagnostics)
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
        public void Execute() => CertificationKernelResources.Compose(Environment, Body, Views, Persistent, Solver, Diagnostics)
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
        public void Execute() => CertificationKernelResources.Compose(Environment, Body, Views, Persistent, Solver, Diagnostics)
            .FinalizePreparedSubstep(SubstepIndex, RuntimeState);
    }
}
}
