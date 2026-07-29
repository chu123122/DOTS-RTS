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
    public struct PreparePersistentClassificationJob : IJob
    {
        public CertificationEnvironmentResources Environment;
        public CertificationBodyResources Body;
        public CertificationViewResources Views;
        public PersistentCertificationResources Persistent;
        public CertificationDiagnosticsResources Diagnostics;
        public NativeReference<ContactPipelineExecutionState> RuntimeState;
        public void Execute() => CertificationKernelResources.Compose(Environment, Body, Views, Persistent, default, Diagnostics)
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
        public void Execute() => CertificationKernelResources.Compose(Environment, Body, Views, Persistent, default, Diagnostics)
            .CommitPersistentClassification(RuntimeState);
    }
}
}
