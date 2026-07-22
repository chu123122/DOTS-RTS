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

1. `full_swept_per_substep`: timestep cache disabled.
2. `full_swept_per_timestep`: old swept ContactSet baseline.
3. `incremental_topology`: persistent proxies and neighbor-pair delta only.
4. `incremental_predictive`: topology + persistent predictive contacts.
5. `incremental_scheduled`: complete pipeline with dormant substep scheduling.

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

- `PersistentNeighborPairCount`
- `SweptHitCount`
- `DormantPairCount`
- `PredictivePairCount`
- `ActualPairCount`
- `ScheduledWakeupCount`
- `ActiveConstraintCount`
- `CorrectedPairCount`
- neighbor-to-swept, swept-to-active and active-to-corrected ratios.

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
