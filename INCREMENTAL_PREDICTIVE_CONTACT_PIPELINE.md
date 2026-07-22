# Incremental Predictive Contact Pipeline

## Goal

Replace the all-or-nothing Fat AABB cache with an incremental contact pipeline that keeps three lifetimes separate:

1. **Persistent neighbor topology** across timesteps.
2. **Predictive contacts** for one timestep horizon.
3. **Active constraints** for the current substep.

The implementation must remain an acceleration structure. Any state that cannot be proven safe falls back to the existing full swept-disc broad phase.

## Target data flow

```text
current body state
    -> tight swept bounds for the current horizon
    -> guard-envelope validation
        -> clean proxy: retain its topology
        -> topology-dirty proxy: rebuild only its incident pairs
        -> too many dirty proxies: full rebuild
    -> PersistentNeighborPairSet
    -> exact swept-disc classification
    -> PredictiveContactCache
    -> current-substep ActiveConstraintSet
    -> XPBD solve
    -> escaped corrected bodies
        -> incremental repair for the remaining horizon
```

## Terminology

### Tight swept bounds

The AABB enclosing the body disc from the start of a prediction horizon to its predicted end. It contains no topology reuse margin.

### Guard envelope

A tight swept bound expanded by `FatAabbCacheMargin`. As long as the new tight swept bounds stay inside the old guard envelope, the old neighbor topology remains complete.

### Topology dirty

A body is topology dirty when its entity membership, validity, radius/filtering inputs, or tight swept bounds no longer fit the persistent guard envelope. Topology-dirty bodies require local pair add/remove work.

### Motion dirty

A body is motion dirty when its trajectory changed enough to require swept-contact reclassification while the guard envelope still proves that its neighbor set is complete. Motion dirty does not require broad-phase pair discovery.

## Correctness invariants

1. Every pair that can contact during the current prediction horizon is either present in `PersistentNeighborPairSet` or incident to a body awaiting topology repair.
2. A clean proxy's tight swept bounds are contained by its persistent guard envelope.
3. Any topology repair removes all stale incident pairs for the dirty body before adding the newly overlapping guard-envelope pairs.
4. `PredictiveContactCache` may only be derived from persistent neighbor pairs using the exact swept-disc classifier.
5. Every actual overlap in the current substep must appear in the active constraint set.
6. Entity identity is stable across timesteps; body indices are remapped every timestep and are never persisted as identity.
7. When validation, mapping, capacity, or dirty-ratio checks fail, the frame executes the full swept broad phase.

## Persistent structures

```text
PersistentSweptProxy
    Entity
    GuardMin / GuardMax
    LastTightMin / LastTightMax
    LastMotionVersion
    IsValid

PersistentNeighborPair
    stable Entity pair key
    LastTopologyEpoch
    LastValidatedTimestep

PersistentPredictiveContact
    stable Entity pair key
    lifecycle state
    stable normal / side
    first possible substep
    next check substep
    last seen timestep
```

Transient structures map persistent entities to current body indices and build the current `UnitCollisionPair` arrays.

## Incremental topology update

For each timestep:

1. Build the current entity-to-body-index lookup.
2. Calculate current tight swept bounds.
3. Compare them with persistent guard envelopes.
4. If the dirty ratio exceeds the configured threshold, perform a full rebuild.
5. Otherwise, for each topology-dirty body:
   - remove its incident persistent neighbor pairs;
   - replace its guard proxy;
   - test the new guard envelope against current persistent proxies;
   - add every overlapping stable pair;
   - sort and deduplicate the pair set.
6. Reclassify motion-dirty, newly added, retained predictive, and scheduled pairs.

The first implementation uses a linear proxy scan for each dirty body. This makes the correctness model explicit and avoids rebuilding a global spatial index. A later optimization may replace the local scan with persistent cell membership without changing pair semantics.

## Substep repair

After wall or contact correction, collect escaped bodies instead of immediately rebuilding the full contact set.

- A small escape set triggers local topology repair for the remaining timestep horizon.
- Existing timestep contacts incident to escaped bodies are removed.
- Repaired incident neighbor pairs are reclassified and merged into the timestep contact set.
- A large escape set triggers the existing full rebuild fallback.

## Contact activation

Broad-phase neighbor pairs are not solver constraints. The predictive layer separates:

```text
Dormant -> Approaching -> Predictive -> Actual -> Separating -> Expired
```

Dormant pairs store a conservative first possible substep and are scheduled. They are not scanned by every XPBD iteration. Predictive and actual pairs enter the active constraint set.

The first versions do not persist XPBD lambda across timesteps. Lambda warm start is intentionally deferred because it depends on timestep, substep count, compliance, and contact-normal continuity.

## Fallback policy

Full rebuild is selected when any of these conditions holds:

- no valid persistent cache exists;
- entity membership or current mapping is inconsistent;
- dirty body ratio exceeds the threshold;
- an incremental patch cannot prove pair completeness;
- persistent pair count exceeds the configured inflation budget;
- diagnostics oracle reports a false negative.

Fallback is a normal mode, not an error. The pipeline tracks its frequency and cost.

## Diagnostics funnel

### Topology

- proxy count
- topology-dirty count
- motion-dirty count
- guard escapes
- added / removed / retained neighbor pairs
- local query count
- full rebuild count

### Predictive contact

- persistent neighbor pair count
- reclassified pair count
- swept-hit count
- dormant / predictive / actual counts
- scheduled wakeups
- active constraint count
- corrected pair count

### Timings

- proxy validation
- local topology repair
- pair diff/merge
- swept classification
- contact activation
- fallback
- solver

## Delivery commits

1. Introduce stable pair keys and persistent containers.
2. Route the full swept broad phase through the persistent neighbor set.
3. Incrementally update topology for dirty proxies.
4. Build timestep predictive contacts from persistent neighbors.
5. Incrementally repair escaped substep contacts.
6. Add dormant scheduling and substep contact activation.
7. Publish incremental pipeline diagnostics and oracle counters.
8. Remove the legacy Fat/Adaptive execution path and finalize migration notes.
