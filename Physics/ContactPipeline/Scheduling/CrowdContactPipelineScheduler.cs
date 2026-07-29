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
    public CertificationEnvironmentResources CertificationEnvironment;
    public CertificationBodyResources CertificationBody;
    public CertificationViewResources CertificationViews;
    public PersistentCertificationResources CertificationPersistent;
    public CertificationSolverResources CertificationSolver;
    public CertificationDiagnosticsResources CertificationDiagnostics;
    public SoftAvoidanceJob SoftAvoidance;
    public ConstraintSolverJob ConstraintSolver;

}
}
