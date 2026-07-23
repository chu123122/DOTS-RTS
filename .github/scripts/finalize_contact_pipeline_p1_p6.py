#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STAGED = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
PERSISTENT = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs"

staged = STAGED.read_text(encoding="utf-8")

double_stat = '''        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
'''
single_stat = '''        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
'''
if double_stat in staged:
    staged = staged.replace(double_stat, single_stat, 1)

# This job became semantically unsafe once B0 validation was restored to its
# original order. B0 now calls PrepareSubstepContactPrediction only after the
# pre-soft envelope has been validated.
start = staged.find('''    [BurstCompile]
    private struct PrepareSubstepContactPredictionBodiesJob : IJobParallelFor
''')
if start >= 0:
    end = staged.find('''    [BurstCompile]
    private struct ValidatePredictedContactEnvelopeBodiesJob''', start)
    if end < 0:
        raise RuntimeError("unable to locate unused B0 preparation job end")
    staged = staged[:start] + staged[end:]

lookup_anchor = '''        uint epoch = IncrementalCacheState.Value.TopologyEpoch;
        if (PersistentIncidentLookupEpoch.Value == epoch &&
            PersistentIncidentPairLookup.Count() == PersistentNeighborPairs.Length * 2)
            return;
        PersistentIncidentPairLookup.Clear();
'''
lookup_replacement = '''        uint epoch = IncrementalCacheState.Value.TopologyEpoch;
        int requiredEntryCount = PersistentNeighborPairs.Length * 2;
        if (requiredEntryCount > PersistentIncidentPairLookup.Capacity)
        {
            // Never publish a partial incident index. The repair caller detects
            // the invalid epoch and takes the authoritative full-rebuild path.
            PersistentIncidentPairLookup.Clear();
            PersistentIncidentLookupEpoch.Value = uint.MaxValue;
            return;
        }
        if (PersistentIncidentLookupEpoch.Value == epoch &&
            PersistentIncidentPairLookup.Count() == requiredEntryCount)
            return;
        PersistentIncidentPairLookup.Clear();
'''
if lookup_anchor in staged:
    staged = staged.replace(lookup_anchor, lookup_replacement, 1)
elif "requiredEntryCount > PersistentIncidentPairLookup.Capacity" not in staged:
    raise RuntimeError("persistent incident overflow anchor missing")

if staged.count("statistics.SoftAvoidanceFatAabbUseCount++;") != 1:
    raise RuntimeError("soft-avoidance cache statistic must be emitted exactly once")
if staged.count("if (substepDeltaTime <= 0f)") != 1:
    raise RuntimeError("zero-delta guard missing or duplicated")
if "PrepareSubstepContactPredictionBodiesJob : IJobParallelFor" in staged:
    raise RuntimeError("unused pre-validation B0 job remains")

STAGED.write_text(staged, encoding="utf-8", newline="\n")

persistent = PERSISTENT.read_text(encoding="utf-8")
map_anchor = '''        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        if (!PersistentIncidentPairLookup.IsCreated)
            return false;
'''
map_replacement = '''        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        if (!PersistentIncidentPairLookup.IsCreated ||
            !PersistentIncidentLookupEpoch.IsCreated ||
            PersistentIncidentLookupEpoch.Value !=
                IncrementalCacheState.Value.TopologyEpoch)
            return false;
'''
if map_anchor in persistent:
    persistent = persistent.replace(map_anchor, map_replacement, 1)
elif "PersistentIncidentLookupEpoch.Value !=" not in persistent:
    raise RuntimeError("persistent repair fallback anchor missing")
PERSISTENT.write_text(persistent, encoding="utf-8", newline="\n")
