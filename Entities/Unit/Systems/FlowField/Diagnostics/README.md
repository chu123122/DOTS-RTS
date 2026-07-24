# Simulation Diagnostics

This directory owns observation, validation, recording, and presentation code for the FlowField contact pipeline. Runtime contact correctness must remain independently executable when `RTS_CONTACT_DIAGNOSTICS` is not defined.

## Ownership

- `Capture/` contains telemetry schemas, ECS publication jobs, and the native-to-public snapshot boundary.
- `Capture/Jobs/` contains Burst jobs whose only output is diagnostic data.
- `Instrumentation/` contains compile-time probes such as the profiler clock facade.
- `Validation/` contains optional correctness oracles. An oracle may invalidate authoritative runtime state through an explicit command, but its counters are never control state.
- Presentation, recording, and experiment-control code remain consumers of the public snapshot and must not be referenced by the solver.

## Intended data flow

```text
runtime jobs
  -> block-local telemetry
  -> frame capture buffers
  -> one published diagnostics snapshot
  -> managed double buffer
  -> panel / overlay / CSV / benchmark / ring-buffer rollups
```

Detailed pair and heat-map samples are bounded captures. Long-running monitoring stores frame summaries and down-sampled aggregates rather than retaining every body or pair sample.

## Compile boundary

When `RTS_CONTACT_DIAGNOSTICS` is absent:

1. runtime cache state, dirty sets, escape detection, lifecycle, epochs, repair, and fallback remain;
2. profiler reads, counters, oracle work, selected-pair capture, heat samples, publication, and presentation must be absent from the gameplay path;
3. diagnostic schemas may keep source-compatible empty forms during migration, but they must not become correctness inputs.

## Migration status

The source ownership split is complete for capture, instrumentation, and validation files. The remaining migration work is to remove the telemetry `NativeReference` ABI and diagnostic scratch allocations from gameplay-only builds, then introduce bounded frame capture and ring-buffer rollups.
