# Incremental Predictive Contact Pipeline Benchmark

## Purpose

Validate that the incremental pipeline preserves the full swept-disc result while
reducing topology rebuilds, repeated candidate scans, and substep fallback cost.
The O(N^2) oracle is diagnostic-only and must report zero missing pairs before
performance numbers are accepted.

## Controlled inputs

Every comparison must reset the same:

- entity set, positions, velocities, destinations, radii and inverse masses;
- random seed, timestep, substep count and XPBD iteration count;
- flow field version and obstacle grid;
- persistent proxy, neighbor-pair and predictive-contact caches;
- RVO/soft-avoidance parameters.

Record at least 300 warm frames followed by 600 measured frames. Report average,
P50, P95, P99 and maximum rather than only the mean.

## Required scenarios

### 1. Static dense pack

- 200 and 1000 units;
- no destination changes;
- expected: topology dirty ratio approaches zero after warm-up;
- expected: full rebuild count remains zero;
- expected: persistent neighbor mapping replaces per-substep soft-avoidance hash builds.

### 2. Low-speed doorway congestion

- long-lived contact neighborhoods with small solver corrections;
- expected: mostly motion-dirty, few topology-dirty bodies;
- expected: predictive/actual contact state persists across timesteps.

### 3. High-churn crossing flows

- opposing groups with frequent neighbor changes;
- expected: dirty-ratio fuse selects full rebuild before incremental repair becomes slower;
- compare P95/P99 against full swept baseline.

### 4. Substep envelope escape

- force selected bodies outside the original timestep envelope;
- expected: small escape sets patch only incident pairs;
- expected: large escape sets perform a full rebuild;
- oracle missing pair count must remain zero.

### 5. Low candidate-utilization scene

- reproduce the existing 10-20% candidate-to-contact ratio;
- expected: dormant pairs remain persistent but do not enter each XPBD iteration;
- record scheduled wakeups and active-to-corrected ratio.

## Configurations

1. `A0_B0`: full swept InteractionSet per substep.
2. `A0_B1`: full swept source once per timestep, reused across substeps.
3. `A1_B1`: cross-frame persistent topology/classification views feeding the
   same cross-substep InteractionSet. A requires B, so `A1_B0` is invalid.

Classification reuse, the Soft-avoidance view and dormant scheduling are
derived implementation stages, not independent benchmark strategy switches.

## Metrics

### Topology

- `ProxyCount`
- `TopologyDirtyBodyCount`
- `MotionDirtyBodyCount`
- `CorrectedEscapeBodyCount`
- `LocalProxyQueryCount`
- `NeighborPairAddedCount`
- `NeighborPairRemovedCount`
- `NeighborPairRetainedCount`
- `FullRebuildCount`
- `IncrementalRepairCount`

### Contact funnel

Current-state gauges:

- `PersistentNeighborPairCount`
- `CurrentSweptContactCount`
- `CurrentDormantPairCount`
- `CurrentApproachingPairCount`
- `CurrentPredictivePairCount`
- `CurrentActualPairCount`
- `CurrentActiveConstraintCount`
- `PeakActiveConstraintCount`

Unique timestep events:

- `ScheduledWakeupCount`
- `UniqueActivatedPairCount`
- `UniqueCorrectedPairCount`
- `ExpiredPairCount`

Accumulated work counters:

- `ReclassifiedPairEvaluationCount`
- `ClassificationReuseCount`
- `ClassificationSkippedCount`
- `SweptClassificationEvaluationCount`
- `SoftAvoidancePairEvaluationCount`
- `ActiveConstraintEvaluationCount`

Derived-view gauges/events:

- `CurrentInteractionPairCount`
- `CurrentSoftAvoidancePairCount`
- `PersistentViewReuseCount`
- `PersistentViewRebuildCount`
- `InteractionEnvelopeEscapeCount`

Record neighbor-to-swept, swept-to-current-active and activated-to-corrected
ratios. Never divide a cumulative evaluation count by a current-state gauge.

### Cost

- proxy validation;
- local broadphase;
- pair diff;
- swept classification;
- contact activation;
- fallback;
- soft avoidance;
- XPBD solver total.

### Correctness

- `OracleMissingPairCount == 0`;
- `SoftAvoidanceOracleMissingPairCount == 0` in diagnostic runs;
- no NaN/Inf position or velocity;
- max penetration no worse than the full swept baseline tolerance;
- deterministic final-state hash matches repeated runs with the same seed.

## Acceptance gate

The legacy execution path can be deleted after all five scenarios satisfy:

- zero oracle false negatives;
- no deterministic replay divergence attributable to pair ordering;
- static and congestion scenes show lower P95 broadphase/contact-set cost;
- high-churn P99 is not materially worse due to the dirty-ratio fuse;
- soft avoidance no longer performs a spatial-hash rebuild per substep when the
  incremental cache is valid.

## Diagnostics schema v5

The migrated recorder writes one row per completed timestep from
`IncrementalContactPipelineSnapshot`; configuration, solver statistics, topology
deltas, contact lifecycle, timings and oracle counters therefore share the same
timestep. Existing `adaptive_tuning_result.csv` files are schema v1 legacy data
and must not be concatenated with v5 results.

The recorder skips the configured warmup interval, writes raw v5 CSV samples,
and emits a sibling `_summary.csv` containing average, P50, P95, P99 and maximum
for solver, soft avoidance, topology update, predictive contact and key count
metrics. The benchmark tuner intentionally does not reset the scene; each trial
must still be launched from the same controlled state and cleared persistent
cache.

## Legacy recorder migration

| Schema v1 field | Schema v5 replacement |
| --- | --- |
| `EnableFatAabb` / `EnableAdaptive` | `PipelineMode`, `TimestepCacheEnabled`, configuration label |
| `FatCacheMargin` | `GuardEnvelopeMargin` |
| `CacheHitRate` | `CleanProxyRatio` and `RetainedNeighborPairRatio` |
| `CacheReuse` | `UsedIncrementalTopology` plus retained-pair count |
| `ContactPairs` | neighbor, swept, active and corrected funnel stages |
| `CacheRebuild` | `FullRebuildCount` and `IncrementalRepairCount` |
| `AdaptiveRegionCount` | legacy-only; removed from the default panel |
| `PairUtilization` | neighbor-to-swept, swept-to-active and activated-to-corrected ratios |

The default debugger page is deliberately implementation-neutral. Legacy Fat
AABB counters are available only in the detailed compatibility foldout.

For the current no-sleep migration, compare `A0_B1` and `A1_B1` from the same
restored baseline. In a warmed static run, `ClassificationSkippedCount` should
approach `PersistentNeighborPairCount`, classification evaluations should
approach zero, and `PersistentViewReuseCount` should dominate rebuilds. Sleeping
variants and sleeping counters are intentionally excluded until that feature is
migrated separately.
