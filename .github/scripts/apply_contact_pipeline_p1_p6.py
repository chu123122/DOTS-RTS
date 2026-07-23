#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def write(path: str, content: str) -> None:
    p = ROOT / path
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content, encoding="utf-8", newline="\n")


def replace_once(path: str, old: str, new: str) -> None:
    content = read(path)
    count = content.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:100]!r}")
    write(path, content.replace(old, new, 1))


def phase1() -> None:
    base = "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs"
    replace_once(
        base,
        "using RTS.Unit.FlowField.Jobs;\nusing UnityEngine;\n",
        "using RTS.Unit.FlowField.Jobs;\n",
    )
    replace_once(
        base,
        '''        Debug.Log(\n            $"SingletonSolver={SystemAPI.GetSingleton<UnitContactSolverSettings>().ContactPositionSolver}, " +\n            $"ExperimentOverride={IncrementalContactPipelineExperimentRuntime.OverrideEnabled}, " +\n            $"ExperimentSolver={IncrementalContactPipelineExperimentRuntime.ContactPositionSolver}, " +\n            $"EffectiveSolver={contactSolverSettings.ContactPositionSolver}, " +\n            $"CaptureMask={SimulationDebuggerRuntime.CaptureMask}, " +\n            $"ParallelJacobi={useParallelJacobi}");\n''',
        "",
    )

    path = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
    content = read(path)
    content = content.replace(
        "    public float SecondaryTotal;\n    public int Count;",
        "    public float SecondaryTotal;\n    public float TertiaryTotal;\n    public int Count;",
    )
    content = content.replace(
        "                FromSolvedPosition = 0\n",
        "                FromSolvedPosition = 0,\n"
        "                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,\n"
        "                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,\n"
        "                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,\n"
        "                RvoTimeHorizon = Configuration.RvoTimeHorizon\n",
        1,
    )
    content = content.replace(
        "                    FromSolvedPosition = 1\n",
        "                    FromSolvedPosition = 1,\n"
        "                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,\n"
        "                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,\n"
        "                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,\n"
        "                    RvoTimeHorizon = Configuration.RvoTimeHorizon\n",
        1,
    )
    content = content.replace(
        "                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0)\n",
        "                Enabled = (byte)(Configuration.EnableTimestepContactSetCache ? 1 : 0),\n"
        "                PredictiveSkin = Configuration.PredictiveSkin,\n"
        "                TimestepContactMargin = Configuration.TimestepContactMargin,\n"
        "                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,\n"
        "                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,\n"
        "                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,\n"
        "                RvoTimeHorizon = Configuration.RvoTimeHorizon\n",
        1,
    )
    content = content.replace(
        "        public byte FromSolvedPosition;\n",
        "        public byte FromSolvedPosition;\n"
        "        public float SoftAvoidanceShell;\n"
        "        public float SoftAvoidanceResponseRate;\n"
        "        public SoftAvoidanceVelocitySolverMode SoftSolverMode;\n"
        "        public float RvoTimeHorizon;\n",
        1,
    )
    content = content.replace(
        '''            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;\n            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;\n            state.TimestepInteractionEnvelopeMin = state.TimestepEnvelopeMin;\n            state.TimestepInteractionEnvelopeMax = state.TimestepEnvelopeMax;\n''',
        '''            state.TimestepEnvelopeMin = math.min(start.xz, end.xz) - extent;\n            state.TimestepEnvelopeMax = math.max(start.xz, end.xz) + extent;\n            CalculateInteractionBounds(\n                state,\n                Skin,\n                Margin,\n                SoftAvoidanceShell,\n                SoftAvoidanceResponseRate,\n                SoftSolverMode,\n                RvoTimeHorizon,\n                out state.TimestepInteractionEnvelopeMin,\n                out state.TimestepInteractionEnvelopeMax);\n''',
        1,
    )
    content = content.replace(
        '''        public NativeArray<byte> EscapeFlags;\n        public byte Enabled;\n''',
        '''        public NativeArray<byte> EscapeFlags;\n        public byte Enabled;\n        public float PredictiveSkin;\n        public float TimestepContactMargin;\n        public float SoftAvoidanceShell;\n        public float SoftAvoidanceResponseRate;\n        public SoftAvoidanceVelocitySolverMode SoftSolverMode;\n        public float RvoTimeHorizon;\n''',
        1,
    )
    content = content.replace(
        '''            float extent = math.max(0f, state.Radius);\n            float2 predictedEnd = (state.PredictedPosition + state.BasePredictedVelocity).xz;\n            float2 min = math.min(state.PredictedPosition.xz, predictedEnd) - extent;\n            float2 max = math.max(state.PredictedPosition.xz, predictedEnd) + extent;\n''',
        '''            CalculateValidationBounds(\n                state,\n                PredictiveSkin,\n                TimestepContactMargin,\n                SoftAvoidanceShell,\n                SoftAvoidanceResponseRate,\n                SoftSolverMode,\n                RvoTimeHorizon,\n                out float2 min,\n                out float2 max);\n''',
        1,
    )
    content = content.replace(
        '''            state.TimestepEnvelopeMin = math.min(state.StartPosition.xz, state.PredictedPosition.xz) - extent;\n            state.TimestepEnvelopeMax = math.max(state.StartPosition.xz, state.PredictedPosition.xz) + extent;\n            state.TimestepInteractionEnvelopeMin = state.TimestepEnvelopeMin;\n            state.TimestepInteractionEnvelopeMax = state.TimestepEnvelopeMax;\n''',
        '''            state.TimestepEnvelopeMin = math.min(state.StartPosition.xz, state.PredictedPosition.xz) - extent;\n            state.TimestepEnvelopeMax = math.max(state.StartPosition.xz, state.PredictedPosition.xz) + extent;\n            // The interaction envelope was prepared for the authoritative timestep\n            // view. In B0 mode it is rebuilt by the following serial view builder.\n            state.TimestepInteractionEnvelopeMin = state.TimestepEnvelopeMin;\n            state.TimestepInteractionEnvelopeMax = state.TimestepEnvelopeMax;\n''',
        1,
    )
    content = content.replace(
        '''                SecondaryTotal = math.length(state.VelocityBeforeContact),\n                Count = 1,\n                ActivatedCount = (int)math.round(math.length(state.IntegratedVelocity) * 100000f)\n''',
        '''                SecondaryTotal = math.length(state.VelocityBeforeContact),\n                TertiaryTotal = math.length(state.IntegratedVelocity),\n                Count = 1\n''',
        1,
    )
    content = content.replace(
        "            speedAfter += body.ActivatedCount / 100000f;",
        "            speedAfter += body.TertiaryTotal;",
        1,
    )
    content = content.replace(
        '''        if (!EnablePersistentContactCache)\n        {\n            PersistentSweptProxies.Clear();\n            PersistentNeighborPairs.Clear();\n            PersistentPredictiveContacts.Clear();\n            PersistentIncidentPairLookup.Clear();\n            PersistentIncidentLookupEpoch.Value = 0;\n            IncrementalCacheState.Value = default;\n        }\n''',
        '''        if (!EnablePersistentContactCache)\n        {\n            PersistentSweptProxies.Clear();\n            PersistentNeighborPairs.Clear();\n            PersistentPredictiveContacts.Clear();\n            if (PersistentIncidentPairLookup.IsCreated)\n                PersistentIncidentPairLookup.Clear();\n            if (PersistentIncidentLookupEpoch.IsCreated)\n                PersistentIncidentLookupEpoch.Value = 0;\n            IncrementalCacheState.Value = default;\n        }\n''',
        1,
    )
    content = content.replace(
        '''        if (!EnablePersistentContactCache || !PersistentIncidentPairLookup.IsCreated)\n            return;\n        uint epoch = IncrementalCacheState.Value.TopologyEpoch;\n''',
        '''        if (!EnablePersistentContactCache ||\n            !PersistentIncidentPairLookup.IsCreated ||\n            !PersistentIncidentLookupEpoch.IsCreated)\n            return;\n        uint epoch = IncrementalCacheState.Value.TopologyEpoch;\n''',
        1,
    )
    content = content.replace(
        "        return math.all(innerMin >= outerMin) && math.all(innerMax <= outerMax);",
        "        const float tolerance = 0.00001f;\n"
        "        return math.all(innerMin >= outerMin - tolerance) &&\n"
        "               math.all(innerMax <= outerMax + tolerance);",
        1,
    )
    content = content.replace(
        '''    private static float3 DeterministicPairNormal(int a, int b)\n    {\n        uint hash = math.hash(new int2(math.min(a, b), math.max(a, b)));\n        float angle = (hash & 0xFFFFu) * (2f * math.PI / 65536f);\n        return new float3(math.cos(angle), 0f, math.sin(angle));\n    }\n''',
        '''    private static float3 DeterministicPairNormal(int a, int b)\n    {\n        return DeterministicFallbackNormal(a, b);\n    }\n''',
        1,
    )
    helper_anchor = "    private static bool SoftOutputInsideEnvelope(\n"
    helpers = '''    private static void CalculateInteractionBounds(\n        FlowMovementFrameState state,\n        float predictiveSkin,\n        float margin,\n        float softShell,\n        float softResponseRate,\n        SoftAvoidanceVelocitySolverMode softSolverMode,\n        float rvoTimeHorizon,\n        out float2 min,\n        out float2 max)\n    {\n        CalculatePathBounds(\n            state,\n            softShell,\n            softResponseRate,\n            softSolverMode,\n            rvoTimeHorizon,\n            out float2 pathMin,\n            out float2 pathMax);\n        float contactPadding = math.max(0f, predictiveSkin) +\n                               math.max(0f, margin) * 2f;\n        float avoidancePadding = math.max(0f, softShell) * 0.5f;\n        float extent = math.max(0f, state.Radius) +\n                       math.max(contactPadding, avoidancePadding);\n        min = pathMin - extent;\n        max = pathMax + extent;\n    }\n\n    private static void CalculateValidationBounds(\n        FlowMovementFrameState state,\n        float predictiveSkin,\n        float margin,\n        float softShell,\n        float softResponseRate,\n        SoftAvoidanceVelocitySolverMode softSolverMode,\n        float rvoTimeHorizon,\n        out float2 min,\n        out float2 max)\n    {\n        CalculatePathBounds(\n            state,\n            softShell,\n            softResponseRate,\n            softSolverMode,\n            rvoTimeHorizon,\n            out float2 pathMin,\n            out float2 pathMax);\n        float contactPadding = math.max(0f, predictiveSkin) + math.max(0f, margin);\n        float avoidancePadding = math.max(0f, softShell) * 0.5f;\n        float extent = math.max(0f, state.Radius) +\n                       math.max(contactPadding, avoidancePadding);\n        min = pathMin - extent;\n        max = pathMax + extent;\n    }\n\n    private static void CalculatePathBounds(\n        FlowMovementFrameState state,\n        float softShell,\n        float softResponseRate,\n        SoftAvoidanceVelocitySolverMode softSolverMode,\n        float rvoTimeHorizon,\n        out float2 min,\n        out float2 max)\n    {\n        min = math.min(\n            state.TimestepStartPosition.xz,\n            math.min(\n                state.TimestepPredictedPosition.xz,\n                math.min(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));\n        max = math.max(\n            state.TimestepStartPosition.xz,\n            math.max(\n                state.TimestepPredictedPosition.xz,\n                math.max(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));\n        if (softSolverMode != SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||\n            softShell <= 0f || softResponseRate <= 0f)\n            return;\n        float2 horizonEnd = state.PredictedPosition.xz +\n                            state.BasePredictedVelocity.xz * math.max(0f, rvoTimeHorizon);\n        min = math.min(min, horizonEnd);\n        max = math.max(max, horizonEnd);\n    }\n\n'''
    if helper_anchor not in content:
        raise RuntimeError("helper anchor missing")
    content = content.replace(helper_anchor, helpers + helper_anchor, 1)
    write(path, content)


def phase2() -> None:
    base = "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs"
    replace_once(
        base,
        '''        var jacobiPairCorrections = new NativeList<JacobiPairCorrection>(\n            math.max(unitCount * 4, 1),\n            Allocator.TempJob);\n''',
        '''        var jacobiPairCorrections = new NativeList<JacobiPairCorrection>(\n            math.max(unitCount * 4, 1),\n            Allocator.TempJob);\n        var envelopeEscapeFlags = new NativeArray<byte>(\n            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);\n        var parallelBodyStatistics = new NativeArray<ParallelBodyStageStatistics>(\n            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);\n        var softIncidentOffsets = new NativeArray<int>(\n            unitCount + 1, Allocator.TempJob, NativeArrayOptions.ClearMemory);\n        var softIncidentWriteCursors = new NativeArray<int>(\n            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);\n        var softIncidentPairIndices = new NativeList<int>(\n            math.max(unitCount * 8, 1), Allocator.TempJob);\n        var softPairContributions = new NativeList<SoftAvoidancePairContribution>(\n            math.max(unitCount * 4, 1), Allocator.TempJob);\n        var activeIncidentIndexState =\n            new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob);\n''',
    )
    replace_once(
        base,
        '''            JacobiPairCorrections = jacobiPairCorrections,\n            CurrentIncrementalProxies = currentIncrementalProxies,\n''',
        '''            JacobiPairCorrections = jacobiPairCorrections,\n            EnvelopeEscapeFlags = envelopeEscapeFlags,\n            ParallelBodyStatistics = parallelBodyStatistics,\n            SoftIncidentOffsets = softIncidentOffsets,\n            SoftIncidentWriteCursors = softIncidentWriteCursors,\n            SoftIncidentPairIndices = softIncidentPairIndices,\n            SoftPairContributions = softPairContributions,\n            ActiveIncidentIndexState = activeIncidentIndexState,\n            CurrentIncrementalProxies = currentIncrementalProxies,\n''',
    )
    replace_once(
        base,
        '''            ? solveContactJob.ScheduleParallelJacobi(\n''',
        '''            ? solveContactJob.ScheduleParallelJacobiP1P6(\n''',
    )
    replace_once(
        base,
        '''        JobHandle jacobiPairCorrectionDisposeHandle =\n            jacobiPairCorrections.Dispose(applyMovementHandle);\n''',
        '''        JobHandle jacobiPairCorrectionDisposeHandle =\n            jacobiPairCorrections.Dispose(applyMovementHandle);\n        JobHandle envelopeEscapeFlagDisposeHandle =\n            envelopeEscapeFlags.Dispose(applyMovementHandle);\n        JobHandle parallelBodyStatisticsDisposeHandle =\n            parallelBodyStatistics.Dispose(applyMovementHandle);\n        JobHandle softIncidentOffsetDisposeHandle =\n            softIncidentOffsets.Dispose(applyMovementHandle);\n        JobHandle softIncidentWriteCursorDisposeHandle =\n            softIncidentWriteCursors.Dispose(applyMovementHandle);\n        JobHandle softIncidentPairIndexDisposeHandle =\n            softIncidentPairIndices.Dispose(applyMovementHandle);\n        JobHandle softPairContributionDisposeHandle =\n            softPairContributions.Dispose(applyMovementHandle);\n        JobHandle activeIncidentIndexStateDisposeHandle =\n            activeIncidentIndexState.Dispose(applyMovementHandle);\n''',
    )
    replace_once(
        base,
        '''        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n   solverScratchDisposeHandle,\n   jacobiPairCorrectionDisposeHandle,\n   incrementalStatisticsDisposeHandle);\n''',
        '''        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n   solverScratchDisposeHandle,\n   jacobiPairCorrectionDisposeHandle,\n   incrementalStatisticsDisposeHandle);\n        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n            solverScratchDisposeHandle,\n            envelopeEscapeFlagDisposeHandle,\n            parallelBodyStatisticsDisposeHandle);\n        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n            solverScratchDisposeHandle,\n            softIncidentOffsetDisposeHandle,\n            softIncidentWriteCursorDisposeHandle);\n        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n            solverScratchDisposeHandle,\n            softIncidentPairIndexDisposeHandle,\n            softPairContributionDisposeHandle);\n        solverScratchDisposeHandle = JobHandle.CombineDependencies(\n            solverScratchDisposeHandle,\n            activeIncidentIndexStateDisposeHandle);\n''',
    )


def phase3() -> None:
    # Soft Avoidance is already part of the staged graph. Record the contract in DEBT.
    path = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/DEBT.md"
    content = read(path)
    content = content.replace(
        "- Gauss–Seidel contact projection remains serial. The Jacobi mode has a parallel pair-evaluate/body-gather path; a parallel Gauss–Seidel alternative would still require graph coloring or conflict-free batches.\n",
        "- Gauss–Seidel contact projection remains serial. The Jacobi mode has a parallel pair-evaluate/body-gather path; a parallel Gauss–Seidel alternative would still require graph coloring or conflict-free batches.\n"
        "- Soft Avoidance now uses pair-evaluate/body-gather instead of conflicting pair scatter. Its frame-local CSR remains separate from the active-contact CSR.\n",
    )
    write(path, content)


def phase4() -> None:
    path = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ActiveConstraintIncidentIndex.cs"
    content = read(path)
    start = content.index("    private void RebuildActiveConstraintIncidentIndexIfNeeded()")
    end = content.index("    }\n}", start)
    method = '''    private void RebuildActiveConstraintIncidentIndexIfNeeded()\n    {\n        EnsureActiveConstraintIncidentIndexP1P6();\n    }\n'''
    content = content[:start] + method + content[end + 6:]
    write(path, content)


def phase5() -> None:
    support = "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.PersistentIncidentIndex.cs"
    write(
        support,
        '''using Unity.Collections;\nusing Unity.Entities;\nusing Unity.Mathematics;\n\nnamespace RTS.Unit.FlowField.Systems\n{\npublic abstract partial class BaseFlowMovementSystem\n{\n    private NativeParallelMultiHashMap<Entity, int> _persistentIncidentPairLookup;\n    private NativeReference<uint> _persistentIncidentLookupEpoch;\n\n    private void CreatePersistentIncidentLookup()\n    {\n        _persistentIncidentPairLookup =\n            new NativeParallelMultiHashMap<Entity, int>(1, Allocator.Persistent);\n        _persistentIncidentLookupEpoch =\n            new NativeReference<uint>(Allocator.Persistent);\n    }\n\n    private void EnsurePersistentIncidentLookupCapacity(int unitCount)\n    {\n        int required = math.max(\n            1,\n            math.max(unitCount * 64, _persistentNeighborPairs.Length * 2 + 1));\n        if (_persistentIncidentPairLookup.Capacity >= required)\n            return;\n        Dependency.Complete();\n        _persistentIncidentPairLookup.Capacity = required;\n    }\n\n    private void DisposePersistentIncidentLookup()\n    {\n        if (_persistentIncidentPairLookup.IsCreated)\n            _persistentIncidentPairLookup.Dispose();\n        if (_persistentIncidentLookupEpoch.IsCreated)\n            _persistentIncidentLookupEpoch.Dispose();\n    }\n}\n}\n''',
    )
    base = "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs"
    replace_once(
        base,
        '''        _incrementalContactCacheState =\n            new NativeReference<IncrementalContactCacheState>(Allocator.Persistent);\n''',
        '''        _incrementalContactCacheState =\n            new NativeReference<IncrementalContactCacheState>(Allocator.Persistent);\n        CreatePersistentIncidentLookup();\n''',
    )
    replace_once(
        base,
        '''        if (_incrementalContactCacheState.IsCreated)\n            _incrementalContactCacheState.Dispose();\n''',
        '''        if (_incrementalContactCacheState.IsCreated)\n            _incrementalContactCacheState.Dispose();\n        DisposePersistentIncidentLookup();\n''',
    )
    replace_once(
        base,
        '''        int unitCount = _movementQuery.CalculateEntityCount();\n        if (unitCount == 0) return;\n''',
        '''        int unitCount = _movementQuery.CalculateEntityCount();\n        if (unitCount == 0) return;\n        EnsurePersistentIncidentLookupCapacity(unitCount);\n''',
    )
    replace_once(
        base,
        '''            IncrementalCacheState = _incrementalContactCacheState,\n            IncrementalStatistics = incrementalStatistics,\n''',
        '''            IncrementalCacheState = _incrementalContactCacheState,\n            IncrementalStatistics = incrementalStatistics,\n            PersistentIncidentPairLookup = _persistentIncidentPairLookup,\n            PersistentIncidentLookupEpoch = _persistentIncidentLookupEpoch,\n''',
    )
    replace_once(
        base,
        '''        _incrementalContactCacheState.Value = default;\n''',
        '''        _incrementalContactCacheState.Value = default;\n        if (_persistentIncidentPairLookup.IsCreated)\n            _persistentIncidentPairLookup.Clear();\n        if (_persistentIncidentLookupEpoch.IsCreated)\n            _persistentIncidentLookupEpoch.Value = 0;\n''',
    )

    pipeline = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs"
    content = read(pipeline)
    old_start = content.index("    private bool MapDirtyIncidentNeighborPairsToCurrentBodies()")
    old_end = content.index("    private void ClassifyAndPatchDirtyIncidentContacts", old_start)
    replacement = '''    private bool MapDirtyIncidentNeighborPairsToCurrentBodies()\n    {\n        Pairs.Clear();\n        if (EnableDiagnostics)\n            PairDiagnostics.Clear();\n\n        RebuildPersistentIncidentPairLookupIfNeededP1P6();\n        if (!PersistentIncidentPairLookup.IsCreated)\n            return false;\n\n        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)\n        {\n            int dirtyBodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;\n            Entity entity = States[dirtyBodyIndex].Entity;\n            NativeParallelMultiHashMapIterator<Entity> iterator;\n            if (!PersistentIncidentPairLookup.TryGetFirstValue(\n                    entity, out int persistentPairIndex, out iterator))\n                continue;\n            do\n            {\n                if ((uint)persistentPairIndex >= (uint)PersistentNeighborPairs.Length)\n                    return false;\n                StableEntityPairKey key =\n                    PersistentNeighborPairs[persistentPairIndex].Key;\n                if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||\n                    !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))\n                    return false;\n                Pairs.Add(new UnitCollisionPair\n                {\n                    BodyA = math.min(bodyA, bodyB),\n                    BodyB = math.max(bodyA, bodyB)\n                });\n            }\n            while (PersistentIncidentPairLookup.TryGetNextValue(\n                out persistentPairIndex, ref iterator));\n        }\n\n        SortAndDeduplicateBodyPairs(Pairs);\n        return true;\n    }\n\n'''
    content = content[:old_start] + replacement + content[old_end:]
    write(pipeline, content)


def phase6() -> None:
    path = "Entities/Unit/Systems/FlowField/Diagnostics/SimulationDebuggerSnapshotPublishing.cs"
    replace_once(
        path,
        '''        // Diagnostics are explicitly opt-in. Completing here keeps the snapshot internally\n        // consistent without reintroducing a synchronization point when all debugger views\n        // are closed.\n        Dependency.Complete();\n''',
        '''        // Never turn optional diagnostics into a blocking point. The existing A/B\n        // managed snapshots are published only after the previous solver dependency has\n        // naturally completed; otherwise this sample is skipped and retried next update.\n        if (!Dependency.IsCompleted)\n            return;\n        Dependency.Complete();\n''',
    )

    path = "Entities/Unit/Systems/FlowField/Diagnostics/IncrementalContactPipelineDiagnostics.cs"
    replace_once(
        path,
        '''[UpdateInGroup(typeof(PresentationSystemGroup))]\npublic partial class IncrementalContactPipelineDiagnosticsSystem : SystemBase\n''',
        '''// Read the previous completed snapshot before the current frame schedules a new\n// writer. This keeps the existing view one frame behind without forcing the\n// Presentation group to wait on the current solver chain.\n[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]\n[UpdateBefore(typeof(RTS.Unit.FlowField.Systems.LocalUnitFlowMovementSystem))]\npublic partial class IncrementalContactPipelineDiagnosticsSystem : SystemBase\n''',
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", type=int, required=True, choices=range(1, 7))
    args = parser.parse_args()
    globals()[f"phase{args.phase}"]()


if __name__ == "__main__":
    main()
