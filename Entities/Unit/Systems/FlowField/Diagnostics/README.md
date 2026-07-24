# Simulation Diagnostics

This directory owns observation, validation, recording, and presentation code for the FlowField contact pipeline. Runtime contact correctness must remain independently executable when `RTS_CONTACT_DIAGNOSTICS` is not defined.

## Ownership

- `Runtime/` contains diagnostics-only contracts, capture masks, runtime selection, and settings requests. These types coordinate observation and must never become contact correctness state.
- `Capture/` contains telemetry schemas, Stage3 diagnostic element contracts, ECS publication jobs, and the native-to-public snapshot boundary.
- `Capture/Jobs/` contains Burst jobs whose only output is diagnostic data, including solver-side debugger capture helpers.
- `Instrumentation/` contains compile-time probes such as the profiler clock facade.
- `Validation/` contains optional correctness oracles. An oracle may invalidate authoritative runtime state through an explicit command, but its counters are never control state.
- `Presentation/` contains panels, world overlays, unit picking, and camera-follow behavior. It consumes published snapshots and must not be referenced by solver modules.
- `Recording/` contains CSV and local snapshot recorders.
- `Experiments/` contains benchmark overrides and adaptive tuning scenarios.

## Intended data flow

```text
runtime jobs
  -> block-local telemetry
  -> frame capture buffers
  -> one unified current-frame diagnostics snapshot
  -> managed double-slot handoff
  -> panel / overlay / CSV / benchmark
```

Detailed pair and heat-map samples are bounded current-frame captures. Long-term history and RingBuffer storage are intentionally outside this refactor.

## Compile boundary

When `RTS_CONTACT_DIAGNOSTICS` is absent:

1. runtime cache state, dirty sets, escape detection, lifecycle, epochs, repair, and fallback remain;
2. profiler reads, counters, oracle work, selected-pair capture, heat samples, publication, and presentation are absent from the gameplay path;
3. the gameplay scheduler creates no diagnostics entity, persistent diagnostics container, TempJob diagnostics container, publisher job, or telemetry `NativeReference`.

## Migration status

Source ownership is split into runtime control, capture, instrumentation, validation, presentation, recording, and experiment layers. Resource owners and telemetry types now use role-based names; historical Stage3 and P1P6 names are intentionally deferred until runtime differential tests are available. Gameplay-only builds allocate and publish no diagnostics data; diagnostics builds publish one unified current-frame snapshot consumed by all presentation and recording surfaces. Historical RingBuffer storage is not implemented.
