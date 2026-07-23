#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def write(path: str, text: str) -> None:
    (ROOT / path).write_text(text, encoding="utf-8", newline="\n")


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:120]!r}")
    write(path, text.replace(old, new, 1))


BASE = "Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs"
STAGED = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
PERSISTENT = "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs"

# P5C: split the initial persistent path into prepare -> parallel classify -> commit.
replace_once(
    STAGED,
    '''        handle = new BuildInitialP1P6ContactSetJob
        {
            Solver = this,
            RuntimeState = runtimeState
        }.Schedule(handle);
''',
    '''        if (Configuration.EnableTimestepContactSetCache &&
            Configuration.EnablePersistentContactCache)
        {
            handle = ScheduleInitialPersistentContactSetP1P6(
                runtimeState,
                handle);
        }
        else
        {
            handle = new BuildInitialP1P6ContactSetJob
            {
                Solver = this,
                RuntimeState = runtimeState
            }.Schedule(handle);
        }
''')

# Frame-local scratch for parallel classification and spatial-query de-duplication.
replace_once(
    BASE,
    '''        var activeIncidentIndexState =
            new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob);
''',
    '''        var activeIncidentIndexState =
            new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob);
        var persistentClassificationResults =
            new NativeList<PersistentPairClassificationResult>(
                math.max(unitCount * 8, 1), Allocator.TempJob);
        var persistentClassificationState =
            new NativeReference<ParallelPersistentClassificationState>(Allocator.TempJob);
        var persistentSpatialVisitStampByProxy = new NativeArray<uint>(
            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var persistentSpatialVisitStamp =
            new NativeReference<uint>(Allocator.TempJob);
''')

replace_once(
    BASE,
    '''            PersistentIncidentPairLookup = _persistentIncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentIncidentLookupEpoch,
''',
    '''            PersistentIncidentPairLookup = _persistentIncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentIncidentLookupEpoch,
            PersistentSpatialMembership = _persistentSpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentSpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = persistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = persistentSpatialVisitStamp,
            PersistentClassificationResults = persistentClassificationResults,
            PersistentClassificationState = persistentClassificationState,
''')

replace_once(
    BASE,
    '''        JobHandle activeIncidentIndexStateDisposeHandle =
            activeIncidentIndexState.Dispose(applyMovementHandle);
''',
    '''        JobHandle activeIncidentIndexStateDisposeHandle =
            activeIncidentIndexState.Dispose(applyMovementHandle);
        JobHandle persistentClassificationResultDisposeHandle =
            persistentClassificationResults.Dispose(applyMovementHandle);
        JobHandle persistentClassificationStateDisposeHandle =
            persistentClassificationState.Dispose(applyMovementHandle);
        JobHandle persistentSpatialVisitStampArrayDisposeHandle =
            persistentSpatialVisitStampByProxy.Dispose(applyMovementHandle);
        JobHandle persistentSpatialVisitStampDisposeHandle =
            persistentSpatialVisitStamp.Dispose(applyMovementHandle);
''')

replace_once(
    BASE,
    '''        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            activeIncidentIndexStateDisposeHandle);
''',
    '''        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            activeIncidentIndexStateDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            persistentClassificationResultDisposeHandle,
            persistentClassificationStateDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            persistentSpatialVisitStampArrayDisposeHandle,
            persistentSpatialVisitStampDisposeHandle);
''')

replace_once(
    BASE,
    '''        if (_persistentIncidentLookupEpoch.IsCreated)
            _persistentIncidentLookupEpoch.Value = 0;
''',
    '''        if (_persistentIncidentLookupEpoch.IsCreated)
            _persistentIncidentLookupEpoch.Value = 0;
        if (_persistentSpatialMembership.IsCreated)
            _persistentSpatialMembership.Clear();
        if (_persistentSpatialMembershipEpoch.IsCreated)
            _persistentSpatialMembershipEpoch.Value = 0;
''')

# P5B: keep the persistent membership view coherent after authoritative rebuilds.
replace_once(
    PERSISTENT,
    '''            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            long buildElapsed = TimestampToNanoseconds(
''',
    '''            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            RebuildPersistentSpatialMembershipP1P6(
                IncrementalCacheState.Value.TopologyEpoch);
            long buildElapsed = TimestampToNanoseconds(
''')

old_query = '''        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            if ((GetDirtyFlags(dirty.BodyIndex) &
                 IncrementalBodyDirtyFlags.Topology) == 0)
                continue;

            FlowMovementFrameState dirtyState = States[dirty.BodyIndex];
            if (!TryFindPersistentProxy(
                    dirtyState.Entity,
                    out PersistentSweptProxy dirtyProxy) ||
                dirtyProxy.IsValid == 0)
                continue;

            for (int proxyIndex = 0; proxyIndex < PersistentSweptProxies.Length; proxyIndex++)
            {
                PersistentSweptProxy other = PersistentSweptProxies[proxyIndex];
                if (other.IsValid == 0 || other.Entity == dirtyProxy.Entity)
                    continue;
                incrementalStatistics.LocalProxyQueryCount++;
                if (!AabbOverlaps(
                        dirtyProxy.GuardMin,
                        dirtyProxy.GuardMax,
                        other.GuardMin,
                        other.GuardMax))
                    continue;

                IncrementalNeighborPairScratch.Add(new PersistentNeighborPair
                {
                    Key = StableEntityPairKey.Create(dirtyProxy.Entity, other.Entity),
                    TopologyEpoch = nextTopologyEpoch,
                    LastValidatedTimestep = nextTimestep
                });
            }
        }
'''
new_query = '''        bool spatialMembershipReady =
            RebuildPersistentSpatialMembershipP1P6(nextTopologyEpoch);
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            if ((GetDirtyFlags(dirty.BodyIndex) &
                 IncrementalBodyDirtyFlags.Topology) == 0)
                continue;

            FlowMovementFrameState dirtyState = States[dirty.BodyIndex];
            if (!TryFindPersistentProxy(
                    dirtyState.Entity,
                    out PersistentSweptProxy dirtyProxy) ||
                dirtyProxy.IsValid == 0)
                continue;

            int dirtyProxyIndex = FindPersistentProxyIndex(dirtyState.Entity);
            if (spatialMembershipReady && dirtyProxyIndex >= 0 &&
                TryAppendPersistentSpatialNeighborsP1P6(
                    dirtyProxyIndex,
                    nextTopologyEpoch,
                    ref incrementalStatistics))
                continue;

            // Capacity failure or an invalid membership epoch takes the original
            // authoritative O(N) path. This is a correctness fallback, not a partial query.
            for (int proxyIndex = 0; proxyIndex < PersistentSweptProxies.Length; proxyIndex++)
            {
                PersistentSweptProxy other = PersistentSweptProxies[proxyIndex];
                if (other.IsValid == 0 || other.Entity == dirtyProxy.Entity)
                    continue;
                incrementalStatistics.LocalProxyQueryCount++;
                if (!AabbOverlaps(
                        dirtyProxy.GuardMin,
                        dirtyProxy.GuardMax,
                        other.GuardMin,
                        other.GuardMax))
                    continue;

                IncrementalNeighborPairScratch.Add(new PersistentNeighborPair
                {
                    Key = StableEntityPairKey.Create(dirtyProxy.Entity, other.Entity),
                    TopologyEpoch = nextTopologyEpoch,
                    LastValidatedTimestep = nextTimestep
                });
            }
        }
'''
replace_once(PERSISTENT, old_query, new_query)

# Add a binary-search index helper next to the existing proxy lookup code.
anchor = '''    private bool IsTopologyDirtyEntity(Entity entity)
    {
        if (!TryFindCurrentBodyIndex(entity, out int bodyIndex))
            return true;
        return (GetDirtyFlags(bodyIndex) & IncrementalBodyDirtyFlags.Topology) != 0;
    }
'''
insert = anchor + '''
    private int FindPersistentProxyIndex(Entity entity)
    {
        int low = 0;
        int high = PersistentSweptProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = StableEntityPairKey.CompareEntity(
                PersistentSweptProxies[middle].Entity,
                entity);
            if (comparison == 0)
                return middle;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }
'''
replace_once(PERSISTENT, anchor, insert)

# Static checks before committing the generated patch.
base = read(BASE)
staged = read(STAGED)
persistent = read(PERSISTENT)
required = {
    "parallel persistent scheduler": "ScheduleInitialPersistentContactSetP1P6" in staged,
    "classification scratch": "PersistentClassificationResults = persistentClassificationResults" in base,
    "spatial membership wiring": "PersistentSpatialMembership = _persistentSpatialMembership" in base,
    "spatial query": "TryAppendPersistentSpatialNeighborsP1P6" in persistent,
    "fallback full scan": "authoritative O(N) path" in persistent,
}
missing = [name for name, ok in required.items() if not ok]
if missing:
    raise RuntimeError("P5B/P5C verification failed: " + ", ".join(missing))
