from pathlib import Path
import re

ROOT = Path("Entities/Unit/Systems/FlowField")
PIPE = ROOT / "Jobs/ContactPipeline"

paths = {
    "base": ROOT / "BaseFlowMovementSystem.cs",
    "stage_jobs": PIPE / "Core/ContactPipelineStageJobs.cs",
    "scheduler": PIPE / "Core/CrowdContactPipelineScheduler.cs",
    "lifecycle": PIPE / "Core/ContactPipelineLifecycleJob.cs",
    "candidate_store": ROOT / "InteractionCandidateStore.cs",
    "body_resources": ROOT / "CrowdStepBodyResources.cs",
    "certification_resources": ROOT / "InteractionCertificationFrameResources.cs",
    "soft_resources": ROOT / "SoftAvoidanceFrameResources.cs",
    "solver_resources": ROOT / "ConstraintSolverFrameResources.cs",
    "execution_resources": ROOT / "ContactPipelineExecutionResources.cs",
    "intent": ROOT / "Jobs/BuildCrowdMotionIntentJob.cs",
    "result": ROOT / "Jobs/BuildCrowdBodyResultsJob.cs",
    "apply": ROOT / "Jobs/ApplyFlowMovementJob.cs",
    "body_contracts": PIPE / "Core/CrowdSimulationDataContracts.cs",
    "environment": PIPE / "Core/CrowdEnvironmentViews.cs",
    "configuration": PIPE / "Core/ContactPipelineConfiguration.cs",
    "body_pair": PIPE / "Core/BodyPair.cs",
    "contact_constraint": PIPE / "Core/ContactConstraint.cs",
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
    "BuildCertificationEvidence(",
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
    if required not in text["certification_resources"]:
        raise SystemExit(f"Timestep certificate resource missing: {required}")

for required in (
    "ContactPipelineConfiguration.Create(",
    "NextSimulationStepId(",
    "new CrowdContactPipelineScheduler",
    "InteractionCandidateStore.Create(",
    "CrowdStepBodyResources.Create(",
    "InteractionCertificationFrameResources.Create(",
    "SoftAvoidanceFrameResources.Create(",
    "ConstraintSolverFrameResources.Create(",
    "ContactPipelineExecutionResources.Create(",
):
    if required not in text["base"]:
        raise SystemExit(f"Composition root contract missing: {required}")
for required in (
    "InteractionCertificationJob",
    "MotionIntegrationJob",
    "SoftAvoidanceJob",
    "ConstraintSolverJob",
):
    if required not in text["stage_jobs"]:
        raise SystemExit(f"Focused stage job missing: {required}")
if "public partial struct CrowdContactPipelineScheduler" not in text["scheduler"]:
    raise SystemExit("Pipeline scheduling composition is missing")
if ": IJob" in text["scheduler"]:
    raise SystemExit("Scheduling composition became another scheduled mega-job")

for owner, required in {
    "candidate_store": ("CreateLifecycleJob(", "NativeList<PersistentNeighborPair> NeighborPairs"),
    "body_resources": ("CreateMotionJob(", "NativeArray<CrowdBodyResult> Results"),
    "certification_resources": ("CreateJob(", "NativeList<BodyPair> SoftAvoidancePairs"),
    "soft_resources": ("CreateJob(", "NativeList<SoftAvoidancePairContribution> PairContributions"),
    "solver_resources": ("CreateJob(", "NativeList<JacobiPairCorrection> JacobiPairCorrections"),
    "execution_resources": ("NativeReference<SerialContactPipelineControlState> SerialControlState",),
}.items():
    for token in required:
        if token not in text[owner]:
            raise SystemExit(f"Focused resource owner {owner} misses: {token}")
for retired in (
    ROOT / "BaseFlowMovementComposition.cs",
    ROOT / "BaseFlowMovementComposition.cs.meta",
    ROOT / "ContactPipelineResources.cs",
    ROOT / "ContactPipelineResources.cs.meta",
):
    if retired.exists():
        raise SystemExit(f"Retired aggregate composition/resource path still exists: {retired}")
for forbidden in ("ContactPersistentState", "ContactFrameResources", "ComposeContactPipelineScheduler("):
    for cs_path in ROOT.rglob("*.cs"):
        if forbidden in cs_path.read_text(encoding="utf-8"):
            raise SystemExit(f"Retired aggregate composition/resource symbol remains in {cs_path}: {forbidden}")
for forbidden in ("SolveXpbdUnitContactsJob", "ComposeContactSolverJob("):
    for cs_path in ROOT.rglob("*.cs"):
        if forbidden in cs_path.read_text(encoding="utf-8"):
            raise SystemExit(f"Retired solver compatibility symbol remains in {cs_path}: {forbidden}")
for retired in (
    PIPE / "Core/SolveXpbdUnitContactsJob.cs",
    PIPE / "Core/SolveXpbdUnitContactsJob.cs.meta",
    PIPE / "Core/CrowdEnvironmentAccess.cs",
    PIPE / "Core/CrowdEnvironmentAccess.cs.meta",
):
    if retired.exists():
        raise SystemExit(f"Retired solver compatibility path still exists: {retired}")

for required in ("WorldId", "SimulationStepId", "CalculateCertificationFingerprint"):
    if required not in text["configuration"]:
        raise SystemExit(f"Same-step certification identity missing: {required}")
if "completedStep.SimulationStepId = statistics.Timestep" in text["pipeline_snapshot"]:
    raise SystemExit("SimulationStepId is again derived from persistent-cache age")
for required in ("CacheGeneration", "statistics.Timestep = completedStep.SimulationStepId"):
    if required not in text["telemetry"] + text["pipeline_snapshot"]:
        raise SystemExit(f"Step/cache identity separation missing: {required}")

retired_body_paths = (
    ROOT / "Jobs/FlowMovementFrameState.cs",
    ROOT / "Jobs/FlowMovementFrameState.cs.meta",
    ROOT / "Jobs/CalculateIndependentFlowForceJob.cs",
    ROOT / "Jobs/CalculateIndependentFlowForceJob.cs.meta",
)
for retired in retired_body_paths:
    if retired.exists():
        raise SystemExit(f"Retired body compatibility path still exists: {retired}")

for required in (
    "NativeArray<CrowdBodySnapshot> Bodies",
    "NativeArray<CrowdNavigationState> NavigationStates",
    "NativeArray<CrowdMotionIntent> MotionIntents",
):
    if required not in text["intent"]:
        raise SystemExit(f"Movement-intent stage misses direct product: {required}")
for required in (
    "NativeArray<CrowdMotionEvidence> MotionEvidence",
    "NativeArray<CrowdBodyStepState> StepStates",
):
    if required not in text["body_resources"]:
        raise SystemExit(f"Crowd-step product is not directly owned: {required}")
if "NativeArray<CrowdBodyResult> Results" not in text["result"] + text["apply"]:
    raise SystemExit("Detached crowd results are not the ECS writeback product")
for forbidden in ("FlowMovementFrameState", "CalculateIndependentFlowForceJob", "IndependentForce"):
    for cs_path in ROOT.rglob("*.cs"):
        if forbidden in cs_path.read_text(encoding="utf-8"):
            raise SystemExit(f"Retired body compatibility symbol remains in {cs_path}: {forbidden}")
if "FlowNavigationView" not in text["intent"]:
    raise SystemExit("Navigation intent bypasses FlowNavigationView")

retired_pair_paths = (
    PIPE / "Core/ContactPairTypes.cs",
    PIPE / "Core/ContactPairTypes.cs.meta",
)
for retired in retired_pair_paths:
    if retired.exists():
        raise SystemExit(f"Retired contact-pair compatibility path still exists: {retired}")

for required in ("public struct BodyPair", "public struct BodyPairComparer"):
    if required not in text["body_pair"]:
        raise SystemExit(f"Body-pair contract missing: {required}")
for required in (
    "public enum ContactConstraintMode",
    "public struct ContactConstraint",
    "public struct ContactConstraintComparer",
    "public float Lambda;",
    "public byte WasActivated;",
):
    if required not in text["contact_constraint"]:
        raise SystemExit(f"Contact-constraint contract missing: {required}")
for forbidden in (
    "UnitCollisionPair",
    "UnitCollisionPairComparer",
    "UnitContactMode",
    "ContactConstraintDefinition",
    "ContactConstraintRuntime",
    "ContactConstraintHistory",
):
    for cs_path in ROOT.rglob("*.cs"):
        if forbidden in cs_path.read_text(encoding="utf-8"):
            raise SystemExit(f"Retired pair compatibility symbol remains in {cs_path}: {forbidden}")
for required in (
    "NativeList<BodyPair> TimestepInteractionPairs",
    "NativeList<BodyPair> SoftAvoidancePairs",
    "NativeList<ContactConstraint> TimestepContactPairs",
):
    if required not in text["certification_resources"]:
        raise SystemExit(f"Pair product is wired to the wrong lifetime: {required}")
if "NativeArray<BodyPair> candidates" not in text["soft"]:
    raise SystemExit("Soft avoidance does not consume the pure BodyPair view")

for required in ("FlowNavigationView", "GridObstacleView", "FlowGridGeometry"):
    if required not in text["environment"]:
        raise SystemExit(f"Environment semantic view missing: {required}")
for stage in ("soft", "motion", "wall"):
    if ".Cost" in text[stage] or "state.Cell." in text[stage]:
        raise SystemExit(f"{stage} again interprets FlowField cell semantics directly")
for forbidden in ("state.Cell.", "Grid[checkIndex].Cost"):
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
certification_source = "\n".join(
    path.read_text(encoding="utf-8")
    for folder in (PIPE / "BroadPhase", PIPE / "Persistent", PIPE / "Prediction")
    for path in folder.rglob("*.cs"))
if "ValidateSolverCorrectionContactEnvelope(" not in certification_source:
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

stage_sources = {
    "certification": "\n".join(
        path.read_text(encoding="utf-8")
        for folder in (PIPE / "BroadPhase", PIPE / "Persistent", PIPE / "Prediction")
        for path in folder.rglob("*.cs")),
    "soft_stage": "\n".join(path.read_text(encoding="utf-8") for path in (PIPE / "SoftAvoidance").rglob("*.cs")),
    "motion_stage": "\n".join(path.read_text(encoding="utf-8") for path in (PIPE / "Motion").rglob("*.cs")),
    "solver_stage": "\n".join(
        source for path in (PIPE / "Solver").rglob("*.cs")
        if "partial struct ConstraintSolverJob" in (source := path.read_text(encoding="utf-8"))),
}
for lower in ("soft_stage", "motion_stage", "solver_stage"):
    for token in persistent_tokens:
        if token in stage_sources[lower]:
            raise SystemExit(f"Focused lower stage {lower} reaches candidate cache: {token}")
if "public InteractionCertificationJob Certification;" not in text["scheduler"]:
    raise SystemExit("Scheduler does not compose the certifier explicitly")
if "public ConstraintSolverJob ConstraintSolver;" not in text["scheduler"]:
    raise SystemExit("Scheduler does not compose the solver explicitly")

if "IncrementalCacheState" in text["oracle"] or ".IsValid = 0" in text["oracle"]:
    raise SystemExit("Diagnostics oracle controls gameplay candidate state")

print("Contact architecture contracts passed")
