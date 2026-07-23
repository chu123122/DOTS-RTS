# Known contact-pipeline debt

## Performance debt

- A motion-dirty body can still cause an O(K) persistent-pair scan. A body-to-incident-pair
  index is the next structural optimization.
- Local topology repair is currently O(K + D×N) in the conservative path. Persistent spatial
  membership or a dedicated proxy index is required before claiming mature incremental broadphase.
- Gauss-Seidel remains serial. The Jacobi mode currently provides a deterministic serial reference;
  active-constraint incident indexing and the parallel evaluate/gather/apply job graph are pending.

## Engineering debt

- The implementation remains a partial job for low-risk migration. Module files define clear
  responsibilities, but Native container ownership can later move into explicit state/view structs.
- Serialized solver settings retain Fat AABB field names for scene compatibility. Only the normalized
  runtime configuration may expose production semantics.
- Unity Editor and Burst compilation are not executed by the lightweight static workflow.

## Scope decision

Sleeping is deliberately excluded. It requires a separate island/wake design and evidence.
