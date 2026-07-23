#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs"
text = PATH.read_text(encoding="utf-8")


def after_once(anchor: str, insertion: str) -> None:
    global text
    if anchor not in text:
        raise RuntimeError(f"missing anchor: {anchor[:120]!r}")
    if insertion.strip() in text:
        return
    text = text.replace(anchor, anchor + insertion, 1)


def replace_once(old: str, new: str) -> None:
    global text
    if old not in text:
        raise RuntimeError(f"missing replacement: {old[:120]!r}")
    text = text.replace(old, new, 1)


def insert_after_call_containing(token: str, insertion: str) -> None:
    global text
    if insertion.strip() in text:
        return
    token_index = text.find(token)
    if token_index < 0:
        raise RuntimeError(f"missing call token: {token}")
    call_start = text.rfind("        solverScratchDisposeHandle =", 0, token_index)
    if call_start < 0:
        raise RuntimeError(f"missing call start before: {token}")
    call_end = text.find(");", token_index)
    if call_end < 0:
        raise RuntimeError(f"missing call end after: {token}")
    call_end += 3 if text[call_end + 2:call_end + 3] == "\n" else 2
    text = text[:call_end] + insertion + text[call_end:]


allocation_anchor = '''        var jacobiPairCorrections = new NativeList<JacobiPairCorrection>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
'''
after_once(
    allocation_anchor,
    '''        var envelopeEscapeFlags = new NativeArray<byte>(
            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var parallelBodyStatistics = new NativeArray<ParallelBodyStageStatistics>(
            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var softIncidentOffsets = new NativeArray<int>(
            unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var softIncidentWriteCursors = new NativeArray<int>(
            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var softIncidentPairIndices = new NativeList<int>(
            math.max(unitCount * 8, 1), Allocator.TempJob);
        var softPairContributions = new NativeList<SoftAvoidancePairContribution>(
            math.max(unitCount * 4, 1), Allocator.TempJob);
        var activeIncidentIndexState =
            new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob);
''',
)

replace_once(
    '''            JacobiPairCorrections = jacobiPairCorrections,
            CurrentIncrementalProxies = currentIncrementalProxies,
''',
    '''            JacobiPairCorrections = jacobiPairCorrections,
            EnvelopeEscapeFlags = envelopeEscapeFlags,
            ParallelBodyStatistics = parallelBodyStatistics,
            SoftIncidentOffsets = softIncidentOffsets,
            SoftIncidentWriteCursors = softIncidentWriteCursors,
            SoftIncidentPairIndices = softIncidentPairIndices,
            SoftPairContributions = softPairContributions,
            ActiveIncidentIndexState = activeIncidentIndexState,
            CurrentIncrementalProxies = currentIncrementalProxies,
''',
)
replace_once(
    "            ? solveContactJob.ScheduleParallelJacobi(\n",
    "            ? solveContactJob.ScheduleParallelJacobiP1P6(\n",
)

dispose_anchor = '''        JobHandle jacobiPairCorrectionDisposeHandle =
            jacobiPairCorrections.Dispose(applyMovementHandle);
'''
after_once(
    dispose_anchor,
    '''        JobHandle envelopeEscapeFlagDisposeHandle =
            envelopeEscapeFlags.Dispose(applyMovementHandle);
        JobHandle parallelBodyStatisticsDisposeHandle =
            parallelBodyStatistics.Dispose(applyMovementHandle);
        JobHandle softIncidentOffsetDisposeHandle =
            softIncidentOffsets.Dispose(applyMovementHandle);
        JobHandle softIncidentWriteCursorDisposeHandle =
            softIncidentWriteCursors.Dispose(applyMovementHandle);
        JobHandle softIncidentPairIndexDisposeHandle =
            softIncidentPairIndices.Dispose(applyMovementHandle);
        JobHandle softPairContributionDisposeHandle =
            softPairContributions.Dispose(applyMovementHandle);
        JobHandle activeIncidentIndexStateDisposeHandle =
            activeIncidentIndexState.Dispose(applyMovementHandle);
''',
)

insert_after_call_containing(
    "jacobiPairCorrectionDisposeHandle,",
    '''        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            envelopeEscapeFlagDisposeHandle,
            parallelBodyStatisticsDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            softIncidentOffsetDisposeHandle,
            softIncidentWriteCursorDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            softIncidentPairIndexDisposeHandle,
            softPairContributionDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            activeIncidentIndexStateDisposeHandle);
''',
)

required = [
    "ScheduleParallelJacobiP1P6",
    "EnvelopeEscapeFlags = envelopeEscapeFlags",
    "SoftPairContributions = softPairContributions",
    "activeIncidentIndexState.Dispose",
    "softPairContributions.Dispose",
    "envelopeEscapeFlagDisposeHandle",
]
missing = [token for token in required if token not in text]
if missing:
    raise RuntimeError("P2 verification failed: " + ", ".join(missing))

PATH.write_text(text, encoding="utf-8", newline="\n")
