# Contact Pipeline modules

The runtime has one authoritative flow:

`candidate state -> certification -> certified views -> soft/motion/solver -> result`.

## Physical ownership

```text
Runtime/ContactPipeline/
├── Contracts/
│   ├── Body/                    # one timestep body product per file
│   ├── Certification/           # evidence, certificate, violations
│   ├── Execution/               # immutable configuration and execution state
│   └── Interaction/             # BodyPair, ContactConstraint, proxy/schedule types
├── State/
│   ├── Persistent/              # cross-step candidate owner
│   └── Frame/                   # focused TempJob resource owners
├── Kernels/                     # container-free/shared Burst-compatible algorithms
├── Scheduling/
│   ├── CrowdContactPipelineScheduler.cs
│   ├── Serial/                  # reserved for serial scheduling composition
│   └── Parallel/
│       ├── ParallelContactPipelineScheduler.cs
│       └── Jobs/                # executable parallel job algorithms
├── Stages/
│   ├── Lifecycle/
│   ├── Certification/
│   │   ├── BroadPhase/
│   │   ├── Persistent/
│   │   ├── Prediction/
│   │   └── Validation/
│   ├── SoftAvoidance/
│   ├── Motion/
│   └── Solver/
│       └── Observability/       # compile-gated solver capture fragments
└── Observability/
    └── Contracts/               # small data-only runtime observation ABI
```

The public namespace remains `RTS.Unit.FlowField.Jobs` during this migration to
avoid an unrelated API rename. Physical ownership, not the historical namespace,
is the module boundary.

## Rules

- `BaseFlowMovementSystem` is the World composition root and never implements pair,
  certificate, solver, or diagnostics algorithms.
- Resource owners only allocate/dispose their lifetime and construct their focused
  stage job. There is no aggregate NativeContainer bag.
- Scheduled job structs keep NativeContainers as direct fields for Collections
  Safety; container-bearing view wrappers are forbidden. Every direct container
  capability is constructed for the lifetime of the stage ABI, even when the
  selected operation does not consume it; Unity validates the complete scheduled
  job layout, not the active enum branch.
- The scheduler owns only job construction and `JobHandle` order. Parallel job
  algorithms live under `Scheduling/Parallel/Jobs`.
- Persistent state is candidate input. Only Certification may accept, repair,
  rebuild, or commit consumer views.
- SoftAvoidance and Solver are scheduled only after an issued certificate passes
  the consumer-view gate.
- Diagnostics presentation/recording/control never participate in correctness.
  `RTS_CONTACT_DIAGNOSTICS` removes telemetry containers and capture algorithms.
