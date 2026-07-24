namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Per-timestep telemetry emitted by the incremental contact pipeline.
///
/// These fields describe work already performed. They are not authoritative
/// simulation state and must never be read to decide cache validity, topology
/// repair, contact activation, fallback, or solver correctness. The historical
/// type name is retained during the diagnostics migration so existing recorders
/// and validation tools keep their serialized/schema contracts.
/// </summary>
public struct IncrementalContactPipelineStatistics
{
    public const int CurrentSchemaVersion = 4;

    public uint Timestep;

    // Proxy/topology gauges and per-timestep events.
    public int ProxyCount;
    public int TopologyDirtyBodyCount;
    public int MotionDirtyBodyCount;
    public int CorrectedEscapeBodyCount;
    public int LocalProxyQueryCount;

    public int PersistentNeighborPairCount;
    public int NeighborPairAddedCount;
    public int NeighborPairRemovedCount;
    public int NeighborPairRetainedCount;
    public int FullRebuildCount;
    public int IncrementalRepairCount;

    // Work counters: these count evaluations, not final-state pairs.
    public int ReclassifiedPairEvaluationCount;
    public int ClassificationReuseCount;
    public int ClassificationSkippedCount;
    public int SweptClassificationEvaluationCount;
    public int SoftAvoidancePairEvaluationCount;
    public int ActiveConstraintEvaluationCount;

    // Current-state gauges. They are recomputed from authoritative runtime
    // containers and remain observations rather than control inputs.
    public int CurrentInteractionPairCount;
    public int CurrentSoftAvoidancePairCount;
    public int CurrentSweptContactCount;
    public int CurrentDormantPairCount;
    public int CurrentApproachingPairCount;
    public int CurrentPredictivePairCount;
    public int CurrentActualPairCount;
    public int CurrentActiveConstraintCount;
    public int PeakActiveConstraintCount;

    // Unique timestep events.
    public int ScheduledWakeupCount;
    public int UniqueActivatedPairCount;
    public int UniqueCorrectedPairCount;
    public int ExpiredPairCount;

    // Correctness-oracle observations. A mismatch may request invalidation through
    // the explicit runtime-state path; the counter itself is never the authority.
    public int OraclePairCount;
    public int OracleMissingPairCount;
    public int OracleExtraPairCount;

    // Ratios intentionally combine like-for-like gauges or unique event sets.
    public float CleanProxyRatio;
    public float RetainedNeighborPairRatio;
    public float NeighborToSweptRatio;
    public float SweptToCurrentActiveRatio;
    public float ActivatedToCorrectedRatio;

    public long ProxyValidationNanoseconds;
    public long FullSweepSourceNanoseconds;
    public long PersistentPairMappingNanoseconds;
    public long LocalBroadPhaseNanoseconds;
    public long PairDiffNanoseconds;
    public long SweptClassificationNanoseconds;
    public long ContactActivationNanoseconds;
    public long FallbackNanoseconds;

    public int PersistentViewReuseCount;
    public int PersistentViewRebuildCount;
    public int InteractionEnvelopeEscapeCount;
    public int SoftAvoidanceOraclePairCount;
    public int SoftAvoidanceOracleMissingPairCount;

    public byte UsedIncrementalTopology;
    public byte UsedFullRebuild;
    public byte OracleMismatch;
    public byte Reserved;
}
}
