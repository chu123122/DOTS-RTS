namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Per-timestep telemetry emitted by the incremental contact pipeline.
///
/// These values describe work already performed. They are not authoritative
/// simulation state and must never be read to decide cache validity, topology
/// repair, contact activation, fallback, or solver correctness.
///
/// When RTS_CONTACT_DIAGNOSTICS is absent, the public schema remains source
/// compatible but every value becomes a compile-time empty observation. This
/// lets the production pipeline keep one method signature while Burst removes
/// counter arithmetic instead of paying for a runtime diagnostics branch.
/// </summary>
public struct IncrementalContactPipelineStatistics
{
    public const int CurrentSchemaVersion = 5;

#if RTS_CONTACT_DIAGNOSTICS
    // Timestep is the scheduled simulation-step identity. CacheGeneration is the
    // independently maintained age/version of persistent candidate state.
    public uint Timestep;
    public uint CacheGeneration;
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
    public int ReclassifiedPairEvaluationCount;
    public int ClassificationReuseCount;
    public int ClassificationSkippedCount;
    public int SweptClassificationEvaluationCount;
    public int SoftAvoidancePairEvaluationCount;
    public int ActiveConstraintEvaluationCount;
    public int CurrentInteractionPairCount;
    public int CurrentSoftAvoidancePairCount;
    public int CurrentSweptContactCount;
    public int CurrentDormantPairCount;
    public int CurrentApproachingPairCount;
    public int CurrentPredictivePairCount;
    public int CurrentActualPairCount;
    public int CurrentActiveConstraintCount;
    public int PeakActiveConstraintCount;
    public int ScheduledWakeupCount;
    public int UniqueActivatedPairCount;
    public int UniqueCorrectedPairCount;
    public int ExpiredPairCount;
    public int OraclePairCount;
    public int OracleMissingPairCount;
    public int OracleExtraPairCount;
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
#else
    // Keep a non-zero unmanaged layout for NativeReference compatibility until
    // the parallel pipeline ABI is split in the following migration step.
    private byte _disabledStorage;
    public uint Timestep { get => default; set { } }
    public uint CacheGeneration { get => default; set { } }
    public int ProxyCount { get => default; set { } }
    public int TopologyDirtyBodyCount { get => default; set { } }
    public int MotionDirtyBodyCount { get => default; set { } }
    public int CorrectedEscapeBodyCount { get => default; set { } }
    public int LocalProxyQueryCount { get => default; set { } }
    public int PersistentNeighborPairCount { get => default; set { } }
    public int NeighborPairAddedCount { get => default; set { } }
    public int NeighborPairRemovedCount { get => default; set { } }
    public int NeighborPairRetainedCount { get => default; set { } }
    public int FullRebuildCount { get => default; set { } }
    public int IncrementalRepairCount { get => default; set { } }
    public int ReclassifiedPairEvaluationCount { get => default; set { } }
    public int ClassificationReuseCount { get => default; set { } }
    public int ClassificationSkippedCount { get => default; set { } }
    public int SweptClassificationEvaluationCount { get => default; set { } }
    public int SoftAvoidancePairEvaluationCount { get => default; set { } }
    public int ActiveConstraintEvaluationCount { get => default; set { } }
    public int CurrentInteractionPairCount { get => default; set { } }
    public int CurrentSoftAvoidancePairCount { get => default; set { } }
    public int CurrentSweptContactCount { get => default; set { } }
    public int CurrentDormantPairCount { get => default; set { } }
    public int CurrentApproachingPairCount { get => default; set { } }
    public int CurrentPredictivePairCount { get => default; set { } }
    public int CurrentActualPairCount { get => default; set { } }
    public int CurrentActiveConstraintCount { get => default; set { } }
    public int PeakActiveConstraintCount { get => default; set { } }
    public int ScheduledWakeupCount { get => default; set { } }
    public int UniqueActivatedPairCount { get => default; set { } }
    public int UniqueCorrectedPairCount { get => default; set { } }
    public int ExpiredPairCount { get => default; set { } }
    public int OraclePairCount { get => default; set { } }
    public int OracleMissingPairCount { get => default; set { } }
    public int OracleExtraPairCount { get => default; set { } }
    public float CleanProxyRatio { get => default; set { } }
    public float RetainedNeighborPairRatio { get => default; set { } }
    public float NeighborToSweptRatio { get => default; set { } }
    public float SweptToCurrentActiveRatio { get => default; set { } }
    public float ActivatedToCorrectedRatio { get => default; set { } }
    public long ProxyValidationNanoseconds { get => default; set { } }
    public long FullSweepSourceNanoseconds { get => default; set { } }
    public long PersistentPairMappingNanoseconds { get => default; set { } }
    public long LocalBroadPhaseNanoseconds { get => default; set { } }
    public long PairDiffNanoseconds { get => default; set { } }
    public long SweptClassificationNanoseconds { get => default; set { } }
    public long ContactActivationNanoseconds { get => default; set { } }
    public long FallbackNanoseconds { get => default; set { } }
    public int PersistentViewReuseCount { get => default; set { } }
    public int PersistentViewRebuildCount { get => default; set { } }
    public int InteractionEnvelopeEscapeCount { get => default; set { } }
    public int SoftAvoidanceOraclePairCount { get => default; set { } }
    public int SoftAvoidanceOracleMissingPairCount { get => default; set { } }
    public byte UsedIncrementalTopology { get => default; set { } }
    public byte UsedFullRebuild { get => default; set { } }
    public byte OracleMismatch { get => default; set { } }
    public byte Reserved { get => default; set { } }
#endif
}
}
