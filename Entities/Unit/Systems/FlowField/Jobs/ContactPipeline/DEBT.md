# Known contact-pipeline debt

## Performance debt

- Gauss–Seidel contact projection remains serial. The Jacobi mode has a parallel pair-evaluate/body-gather path; a parallel Gauss–Seidel alternative would still require graph coloring or conflict-free batches.
- Soft Avoidance now uses pair-evaluate/body-gather instead of conflicting pair scatter. Its frame-local CSR remains separate from the active-contact CSR.
- The active BodyIndex→constraint CSR and cross-timestep Entity→persistent-pair index are versioned and reused, but deterministic list compaction, sorting and persistent-view publication remain serial coordination points.
- Local topology repair now queries a versioned persistent cell→proxy membership view instead of scanning all proxies. If the persistent membership capacity is insufficient, correctness is preserved by invalidating the view and taking the original O(D×N) fallback; production sizing still needs runtime evidence.
- Persistent pair classification now uses serial prepare, parallel pair evaluation and deterministic serial commit. The commit cost can still dominate when nearly every persistent pair changes classification in one timestep.

## Engineering debt

- The implementation remains a partial-job migration. Module files define clear responsibilities, but Native container ownership can later move into explicit state/view structs.
- Serialized solver settings retain Fat AABB field names for scene compatibility. Only the normalized runtime configuration may expose production semantics.
- Unity Editor, Collections safety checks, Burst compilation and runtime benchmarks are not executed by the lightweight static workflow.

## Scope decision

Sleeping is deliberately excluded. It requires a separate island/wake design and evidence.
