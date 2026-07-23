# Known contact-pipeline debt

## Performance debt

- A motion-dirty body can still cause an O(K) persistent-pair scan. A body-to-incident-pair
  index is the next structural optimization.
- Local topology repair is currently O(K + D×N) in the conservative path. Persistent spatial
  membership or a dedicated proxy index is required before claiming mature incremental broadphase.
- Gauss–Seidel contact projection remains serial. The Jacobi mode has a parallel pair-evaluate/body-gather path; a parallel Gauss–Seidel alternative would still require graph coloring or conflict-free batches.
- The active BodyIndex→constraint CSR index is frame-local and rebuilt when the active view changes. It does not yet repay the separate cross-timestep Entity→persistent-pair index debt.

## Engineering debt

- The implementation remains a partial job for low-risk migration. Module files define clear
  responsibilities, but Native container ownership can later move into explicit state/view structs.
- Serialized solver settings retain Fat AABB field names for scene compatibility. Only the normalized
  runtime configuration may expose production semantics.
- Unity Editor and Burst compilation are not executed by the lightweight static workflow.

## Scope decision

Sleeping is deliberately excluded. It requires a separate island/wake design and evidence.
