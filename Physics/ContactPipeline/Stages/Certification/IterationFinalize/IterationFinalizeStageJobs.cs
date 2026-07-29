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
            var kernel = CertificationKernelResources.Compose(Environment, Body, Views, Persistent, Solver, Diagnostics);
            kernel.FinalizeWallIteration(SubstepIndex, RuntimeState
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
            var kernel = CertificationKernelResources.Compose(Environment, Body, Views, Persistent, Solver, Diagnostics);
            kernel.FinalizeContactIteration(SubstepIndex, IterationIndex, RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                , Diagnostics.IterationState, Diagnostics.BlockStatistics
#endif
            );
        }
    }
}
}
