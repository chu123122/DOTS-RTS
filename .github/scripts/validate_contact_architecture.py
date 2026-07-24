from pathlib import Path

ROOT = Path("Entities/Unit/Systems/FlowField")
PIPE = ROOT / "Jobs/ContactPipeline"

paths = {
    "base": ROOT / "BaseFlowMovementSystem.cs",
    "composition": ROOT / "BaseFlowMovementComposition.cs",
    "resources": ROOT / "ContactPipelineResources.cs",
    "frame": ROOT / "Jobs/FlowMovementFrameState.cs",
    "intent": ROOT / "Jobs/CalculateIndependentFlowForceJob.cs",
    "body_contracts": PIPE / "Core/CrowdSimulationDataContracts.cs",
    "environment": PIPE / "Core/CrowdEnvironmentViews.cs",
    "configuration": PIPE / "Core/ContactPipelineConfiguration.cs",
    "pair_types": PIPE / "Core/ContactPairTypes.cs",
    "certificate_contracts": PIPE / "Prediction/InteractionCertificationContracts.cs",
    "certifier": PIPE / "Prediction/InteractionCorrectnessCertifier.cs",
    "timestep": PIPE / "Prediction/TimestepContactSet.cs",
    "guard": PIPE / "Prediction/ContactEnvelopeGuard.cs",
    "soft": PIPE / "SoftAvoidance/SoftAvoidanceSubstep.cs",
    "motion": PIPE / "Motion/ContactMotionIntegration.cs",
    "wall": PIPE / "Solver/WallConstraintSolver.cs",
    "parallel": PIPE / "Solver/ParallelContactPipelineP1P6.cs",
    "telemetry": ROOT / "Diagnostics/Capture/ContactPipelineTelemetry.cs",
    "pipeline_snapshot": ROOT / "Diagnostics/Capture/IncrementalContactPipelineDiagnostics.cs",
    "oracle": ROOT / "Diagnostics/Validation/IncrementalContactOracle.cs",
}

for name, path in paths.items():
    if not path.exists():
        raise SystemExit(f"Missing architecture contract target {name}: {path}")

text = {name: path.read_text(encoding="utf-8") for name, path in paths.items()}

for required in (
    "CrowdBodySnapshot",
    "CrowdNavigationState",
    "CrowdMotionIntent",
    "CrowdMotionEvidence",
    "CrowdBodyStepState",
    "CrowdBodyResult",
):
    if required not in text["body_contracts"]:
        raise SystemExit(f"Crowd data contract missing: {required}")

for required in (
    "InteractionCertificate",
    "InteractionCertificationEvidence",
    "InteractionCertificateViolation",
    "CertifiedInteractionProductDescriptor",
):
    if required not in text["certificate_contracts"]:
        raise SystemExit(f"Certification contract missing: {required}")

for required in (
    "IssueInteractionCertificate(",
    "IssueCertificateForCommittedViews(",
    "RevokeInteractionCertificate(",
    "CalculateBodySetFingerprint(",
):
    if required not in text["certifier"]:
        raise SystemExit(f"Certifier capability missing: {required}")

if "IssueCertificateForCommittedViews(" not in text["timestep"]:
    raise SystemExit("Compact contact views are committed without a certificate")
if "CommitTimestepContactViews(" not in text["timestep"]:
    raise SystemExit("Certified product has no common commit boundary")

for required in (
    "InteractionCertificate = new NativeReference<InteractionCertificate>",
    "InteractionViolations = new NativeList<InteractionCertificateViolation>",
):
    if required not in text["resources"]:
        raise SystemExit(f"Timestep certificate resource missing: {required}")

for required in (
    "ComposeContactSolverJob(",
    "ContactPipelineConfiguration.Create(",
    "NextSimulationStepId(",
    "FlowGridGeometry",
):
    if required not in text["base"]:
        raise SystemExit(f"Composition root contract missing: {required}")
if "new SolveXpbdUnitContactsJob" in text["base"]:
    raise SystemExit("BaseFlowMovementSystem again expands the solver ABI directly")
if "new SolveXpbdUnitContactsJob" not in text["composition"]:
    raise SystemExit("Dedicated solver ABI composition boundary is missing")

for required in ("WorldId", "SimulationStepId", "CalculateCertificationFingerprint"):
    if required not in text["configuration"]:
        raise SystemExit(f"Same-step certification identity missing: {required}")
if "completedStep.SimulationStepId = statistics.Timestep" in text["pipeline_snapshot"]:
    raise SystemExit("SimulationStepId is again derived from persistent-cache age")
for required in ("CacheGeneration", "statistics.Timestep = completedStep.SimulationStepId"):
    if required not in text["telemetry"] + text["pipeline_snapshot"]:
        raise SystemExit(f"Step/cache identity separation missing: {required}")

for required in (
    "public CrowdBodySnapshot Body;",
    "public CrowdNavigationState Navigation;",
    "public CrowdMotionIntent MotionIntent;",
):
    if required not in text["frame"]:
        raise SystemExit(f"Frame state did not adopt explicit data contract: {required}")
for forbidden in ("FlowFieldCell", "NavigationCell", "public FlowFieldCell Cell"):
    if forbidden in text["frame"]:
        raise SystemExit(f"Frame state again retains complete navigation cells: {forbidden}")
for required in ("Navigation.IsBlocked", "MotionIntent.PreferredVelocity"):
    if required not in text["intent"] + text["parallel"]:
        raise SystemExit(f"Compact navigation semantic missing: {required}")

for required in (
    "ContactConstraintDefinition",
    "ContactConstraintRuntime",
    "ContactConstraintHistory",
):
    if required not in text["pair_types"]:
        raise SystemExit(f"Contact-pair lifetime split missing: {required}")

for required in ("FlowNavigationView", "GridObstacleView", "FlowGridGeometry"):
    if required not in text["environment"]:
        raise SystemExit(f"Environment semantic view missing: {required}")
for stage in ("soft", "motion", "wall"):
    if ".Cost" in text[stage] or "state.Cell" in text[stage]:
        raise SystemExit(f"{stage} again interprets FlowField cell semantics directly")
for forbidden in ("state.Cell", "Grid[checkIndex].Cost"):
    if forbidden in text["parallel"]:
        raise SystemExit(f"P1-P6 again bypasses compact environment semantics: {forbidden}")
if "FlowNavigationView" not in text["intent"]:
    raise SystemExit("Navigation intent bypasses FlowNavigationView")
if text["parallel"].count("GridObstacleView.IsBlocked(") < 2:
    raise SystemExit("Parallel soft/hard wall stages bypass GridObstacleView")

for required in (
    "BaseMotionEnvelopeEscape",
    "PredictedContactEnvelopeEscape",
    "SolverCorrectionEnvelopeEscape",
):
    if required not in text["guard"]:
        raise SystemExit(f"Certificate violation evidence missing: {required}")
if "RevokeInteractionCertificate(" not in text["guard"]:
    raise SystemExit("Envelope validation does not revoke its expired certificate")
if "ValidateSolverCorrectionContactEnvelope(" not in text["parallel"]:
    raise SystemExit("P1-P6 solver correction bypasses the certificate guard")

persistent_tokens = (
    "PersistentSweptProxies",
    "PersistentNeighborPairs",
    "PersistentPredictiveContacts",
    "IncrementalCacheState",
)
for stage in ("soft", "motion", "wall"):
    for token in persistent_tokens:
        if token in text[stage]:
            raise SystemExit(f"Lower consumer {stage} reaches candidate cache: {token}")

if "IncrementalCacheState" in text["oracle"] or ".IsValid = 0" in text["oracle"]:
    raise SystemExit("Diagnostics oracle controls gameplay candidate state")

print("Contact architecture contracts passed")
