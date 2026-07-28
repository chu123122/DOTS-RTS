using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField
{

public enum SoftAvoidanceVelocitySolverMode : byte
{
    SurfaceVelocityBuffer,
    ReciprocalVelocityObstacle
}

public enum ContactPositionSolverMode : byte
{
    GaussSeidel,
    Jacobi
}

public enum ContactSolverSkipReason : byte
{
    None,
    CertificateUnavailable,
    CertificateNotIssued,
    CertificateStructureNotVerified,
    CertificateInteractionCountInvalid,
    CertificateViewUnavailable,
    CertificateInteractionPairInvalid,
    CertificateSoftPairInvalid,
    CertificateContactPairInvalid,
    CertificateScheduleInvalid,
    CertificateEntityMappingNotVerified,
    CertificateConfigurationNotVerified,
    CertificateTopologyNotVerified,
    CertificateClassificationNotVerified,
    CertificateConsumerViewsNotCommitted,
    CertificateScopeMismatch,
    BodySetMismatch,
    ConfigurationMismatch,
    SoftPairCountMismatch,
    ContactConstraintCountMismatch,
    DormantScheduleCountMismatch
}

public enum ContactHeatmapMode : byte
{
    ContactLoad,
    ContactSetDensity,
    EscapeFallback
}

public struct FlowFieldCell
{
    public byte Cost; 
    public ushort IntegrationValue; 
    public byte BestDirectionIndex;
}

public struct FlowFieldGrid : IComponentData
{
    public float3 GridOrigin;   
    public int2 GridDimensions; 
    public float CellRadius;     
    
    public NativeArray<FlowFieldCell> Grid;
    public NativeArray<FlowFieldCell> PendingGrid;
}
public struct FlowFieldSettings : IComponentData
{
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
}

public struct FlowFieldGlobalTarget : IComponentData
{
    public float3 TargetPosition; 
}

/// <summary>
/// 单次全局移动订单。组件启用时由 RtsCommandSystem 消费，随后立即禁用。
/// </summary>
public struct MoveOrder : IComponentData, IEnableableComponent
{
    public float3 TargetPosition;
}

/// <summary>
/// 下达 MoveOrder 时的选中单位快照。消费命令时不能改查实时 UnitSelected，
/// 否则输入与预测更新的时序差会把订单绑到之后的选择上。
/// </summary>
public struct MoveOrderSelectionElement : IBufferElementData
{
    public Entity Entity;
}

/// <summary>
/// 单位动态接触 XPBD 求解配置。lambda 在每个 substep 开始时清零。
/// </summary>
public struct UnitContactSolverSettings : IComponentData
{
    public int SubstepCount;
    public int IterationCount;
    public ContactPositionSolverMode ContactPositionSolver;
    public float Compliance;
    public float PredictiveSkin;
    public bool EnablePredictivePairGeneration;
    public bool EnablePredictiveContacts;
    public bool EnableDiagnostics;
    public bool VisualizeSelectedContacts;
    public float DiagnosticCaptureDuration;
    public float DiagnosticCaptureInterval;
    public bool EnableTimestepContactSetCache;
    public bool EnablePersistentContactCache;
    public float PersistentGuardEnvelopeMargin;
    public float TimestepContactMargin;
    public bool VisualizeContactHeatmap;
    public ContactHeatmapMode ContactHeatmapMode;
    public float DiagnosticSlowMotionScale;
}

/// <summary>
/// 最近一帧 Predictive Disc Contact 求解统计。
/// 时间字段由 Job 内 Profiler 时间戳换算成纳秒，不需要主线程 Complete。
///
/// 宏关闭时仍保留同名 IComponentData 契约，属性返回空值：运行时配置和场景
/// 序列化保持兼容，同时让 Burst 能删掉计数器和比率计算。
/// </summary>
public struct PredictiveDiscContactStatistics : IComponentData
{
#if RTS_CONTACT_DIAGNOSTICS
    public int TimestepContactSetBuildCount;
    public int TimestepContactSetClassificationPassCount;
    public int TimestepContactSetSubstepUseCount;
    public int TimestepContactSetUniquePairCount;
    public int TimestepContactSetUniqueActivatedPairCount;
    public int TimestepContactSetDormantPairCount;
    public int TimestepContactSetEscapeBodyCount;
    public int TimestepContactSetFirstEscapeSubstep;
    public int TimestepContactSetFullRebuildCount;
    public int TimestepContactSetFallbackAddedPairCount;
    public long TimestepContactSetBuildNanoseconds;
    public long TimestepContactSetFallbackNanoseconds;
    public int CandidatePairCount;
    public int ContactPairCount;
    public int ActualGeneratedPairCount;
    public int PredictiveGeneratedPairCount;
    public int PotentialPredictivePairCount;
    public int PredictivePairCount;
    public int SoftAvoidanceEvaluationCount;
    public int SoftAvoidanceCandidatePairCount;
    public int SoftAvoidanceActivatedPairCount;
    public int SoftAvoidanceFatAabbUseCount;
    public int ActiveConstraintCount;
    public ContactSolverSkipReason SolverSkipReason;
    public int SolverSkippedSubstepCount;
    public int PredictiveActivatedCount;
    public int UnactivatedPairCount;
    public int PredictiveUnactivatedCount;
    public int PenetratingPairCount;
    public float MaxPenetration;
    public float AveragePenetration;
    public float UnactivatedRatio;
    public float PredictiveUnactivatedRatio;
    public float TotalContactPositionCorrection;
    public float MaxContactPositionCorrection;
    public float TotalVelocityChange;
    public float MaxVelocityChange;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
    public float AverageSpeedBeforeContact;
    public float AverageSpeedAfterContact;
    public long PairGenerationNanoseconds;
    public long SoftAvoidanceNanoseconds;
    public long IterationNanoseconds;
    public long MotionNanoseconds;
    public long ValidationRepairNanoseconds;
    public long DiagnosticsNanoseconds;
    public long SolverNanoseconds;
    public long AverageSoftAvoidanceNanoseconds;
    public long AverageIterationNanoseconds;
#else
    private byte _disabledStorage;
    public int TimestepContactSetBuildCount { get => default; set { } }
    public int TimestepContactSetClassificationPassCount { get => default; set { } }
    public int TimestepContactSetSubstepUseCount { get => default; set { } }
    public int TimestepContactSetUniquePairCount { get => default; set { } }
    public int TimestepContactSetUniqueActivatedPairCount { get => default; set { } }
    public int TimestepContactSetDormantPairCount { get => default; set { } }
    public int TimestepContactSetEscapeBodyCount { get => default; set { } }
    public int TimestepContactSetFirstEscapeSubstep { get => default; set { } }
    public int TimestepContactSetFullRebuildCount { get => default; set { } }
    public int TimestepContactSetFallbackAddedPairCount { get => default; set { } }
    public long TimestepContactSetBuildNanoseconds { get => default; set { } }
    public long TimestepContactSetFallbackNanoseconds { get => default; set { } }
    public int CandidatePairCount { get => default; set { } }
    public int ContactPairCount { get => default; set { } }
    public int ActualGeneratedPairCount { get => default; set { } }
    public int PredictiveGeneratedPairCount { get => default; set { } }
    public int PotentialPredictivePairCount { get => default; set { } }
    public int PredictivePairCount { get => default; set { } }
    public int SoftAvoidanceEvaluationCount { get => default; set { } }
    public int SoftAvoidanceCandidatePairCount { get => default; set { } }
    public int SoftAvoidanceActivatedPairCount { get => default; set { } }
    public int SoftAvoidanceFatAabbUseCount { get => default; set { } }
    public int ActiveConstraintCount { get => default; set { } }
    public ContactSolverSkipReason SolverSkipReason { get => default; set { } }
    public int SolverSkippedSubstepCount { get => default; set { } }
    public int PredictiveActivatedCount { get => default; set { } }
    public int UnactivatedPairCount { get => default; set { } }
    public int PredictiveUnactivatedCount { get => default; set { } }
    public int PenetratingPairCount { get => default; set { } }
    public float MaxPenetration { get => default; set { } }
    public float AveragePenetration { get => default; set { } }
    public float UnactivatedRatio { get => default; set { } }
    public float PredictiveUnactivatedRatio { get => default; set { } }
    public float TotalContactPositionCorrection { get => default; set { } }
    public float MaxContactPositionCorrection { get => default; set { } }
    public float TotalVelocityChange { get => default; set { } }
    public float MaxVelocityChange { get => default; set { } }
    public float TotalWallPositionCorrection { get => default; set { } }
    public float MaxWallPositionCorrection { get => default; set { } }
    public float AverageSpeedBeforeContact { get => default; set { } }
    public float AverageSpeedAfterContact { get => default; set { } }
    public long PairGenerationNanoseconds { get => default; set { } }
    public long SoftAvoidanceNanoseconds { get => default; set { } }
    public long IterationNanoseconds { get => default; set { } }
    public long MotionNanoseconds { get => default; set { } }
    public long ValidationRepairNanoseconds { get => default; set { } }
    public long DiagnosticsNanoseconds { get => default; set { } }
    public long SolverNanoseconds { get => default; set { } }
    public long AverageSoftAvoidanceNanoseconds { get => default; set { } }
    public long AverageIterationNanoseconds { get => default; set { } }
#endif
}

/// <summary>
/// 只在一份完整流场发布后递增，移动和可视化系统据此读取稳定快照。
/// </summary>
public struct FlowFieldRuntimeState : IComponentData
{
    public uint ActiveVersion;
    public uint ActiveRequestVersion;
}

/// <summary>
/// Cost 只随障碍物布局变化，改目标点不影响这个状态。
/// 动态墙壁变化时把 IsDirty 置 true，并请求一次流场重算。
/// </summary>
public struct FlowFieldCostState : IComponentData
{
    public bool IsDirty;
    public uint CostVersion;
}

public struct FlowFieldVisualizationSettings : IComponentData
{
    public bool Visible;
    public bool ShowCost;
    public bool ShowDirections;
    public byte PixelsPerCell;
    public float HeightOffset;
    public float Opacity;
}

public struct UnitSpatialMap : IComponentData
{
    public NativeParallelMultiHashMap<int, Entity> Map;
}
}
