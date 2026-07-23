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
/// 下达 MoveOrder 时的选中单位快照。命令消费阶段不得重新查询实时 UnitSelected，
/// 否则输入与预测更新之间的时序差会让订单绑定到后续选择。
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
    public bool EnableFatAabbCache;
    public float FatAabbCacheMargin;
    public float TimestepContactMargin;
    public bool VisualizeContactHeatmap;
    public ContactHeatmapMode ContactHeatmapMode;
    public float DiagnosticSlowMotionScale;
}

/// <summary>
/// 最近一帧 Predictive Disc Contact 求解统计。
/// 时间字段由 Job 内 Profiler 时间戳换算为纳秒，不引入主线程 Complete。
/// </summary>
public struct PredictiveDiscContactStatistics : IComponentData
{
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
    public long SolverNanoseconds;
    public long AverageSoftAvoidanceNanoseconds;
    public long AverageIterationNanoseconds;
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
/// Cost 只随障碍物布局变化。目标点变化不会修改这个状态。
/// 动态墙壁发生变化时，将 IsDirty 设为 true 并请求一次流场重算。
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
