# Simulation Diagnostics

This directory owns control, capture/publication, presentation, recording and
experiments. Contact correctness must execute independently when
`RTS_CONTACT_DIAGNOSTICS` is not defined.

## Ownership

- Runtime observation data contracts used by scheduled jobs live in
  `Runtime/ContactPipeline/Observability/Contracts`; they are data-only and never
  control simulation.
- Solver-side capture fragments live with their owner at
  `Runtime/ContactPipeline/Stages/Solver/Observability` and are compile-gated.
- `Runtime/` here contains managed per-World debugger control/settings only.
- `Capture/` owns completed-step publication and native-to-managed readback.
- `Instrumentation/` owns compile-time profiler facades.
- `Presentation/`, `Recording/` and `Experiments/` consume immutable publication.
- Correctness oracles are observation-only. They cannot invalidate candidate state,
  issue certificates, rebuild views, or change fallback policy.

## Data flow

```text
compile-gated runtime capture
  -> completed-step ECS buffers
  -> immutable per-World snapshot
  -> panel / overlay / recorder / experiment
```

Spatial readback receives its dependencies explicitly. It does not extend
`BaseFlowMovementSystem` to reach private candidate storage.

## Diagnostics-off contract

When `RTS_CONTACT_DIAGNOSTICS` is absent:

1. candidate state, certification, escape repair and authoritative fallback remain;
2. telemetry NativeContainers, profiler reads, oracle loops and capture jobs are
   removed by preprocessing;
3. the composition root creates no diagnostics entity or publication job;
4. SoftAvoidance and Solver correctness remain guarded by `InteractionCertificate`.
