#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
P5 = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/PersistentParallelClassificationP1P6.cs"
PERSISTENT = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs"
STAGED = ROOT / "Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, got {count}: {old[:120]!r}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8", newline="\n")


replace_once(
    P5,
    '''public struct ParallelPersistentClassificationState
{
    public long BuildStartTimestamp;
    public byte NeedsCommit;
}
''',
    '''public struct ParallelPersistentClassificationState
{
    public long BuildStartTimestamp;
    public long ClassificationStartTimestamp;
    public uint Timestep;
    public uint ClassificationEpoch;
    public byte NeedsCommit;
}
''')

replace_once(
    P5,
    '''            DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
            Results = PersistentClassificationResults.AsDeferredJobArray(),
''',
    '''            DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
            PhaseState = PersistentClassificationState,
            Results = PersistentClassificationResults.AsDeferredJobArray(),
''')

replace_once(
    P5,
    '''            SubstepCount = math.max(1, Configuration.SubstepCount),
            Timestep = IncrementalCacheState.Value.Timestep,
            ClassificationEpoch = CalculateClassificationEpoch(),
            ScheduleStartSubstep = 0
''',
    '''            SubstepCount = math.max(1, Configuration.SubstepCount),
            ScheduleStartSubstep = 0
''')

replace_once(
    P5,
    '''        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        public NativeArray<PersistentPairClassificationResult> Results;
''',
    '''        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeReference<ParallelPersistentClassificationState> PhaseState;
        public NativeArray<PersistentPairClassificationResult> Results;
''')

replace_once(
    P5,
    '''        public int SubstepCount;
        public uint Timestep;
        public uint ClassificationEpoch;
        public int ScheduleStartSubstep;
''',
    '''        public int SubstepCount;
        public int ScheduleStartSubstep;
''')

replace_once(
    P5,
    '''            UnitCollisionPair rawPair = RawPairs[pairIndex];
            FlowMovementFrameState bodyA = States[rawPair.BodyA];
''',
    '''            ParallelPersistentClassificationState phase = PhaseState.Value;
            UnitCollisionPair rawPair = RawPairs[pairIndex];
            FlowMovementFrameState bodyA = States[rawPair.BodyA];
''')

replace_once(
    P5,
    '''                            previous.ClassificationEpoch == ClassificationEpoch &&
''',
    '''                            previous.ClassificationEpoch == phase.ClassificationEpoch &&
''')

replace_once(
    P5,
    '''                previous.LastSeenTimestep = Timestep;
''',
    '''                previous.LastSeenTimestep = phase.Timestep;
''')

replace_once(
    P5,
    '''                    Timestep,
                    ClassificationEpoch,
''',
    '''                    phase.Timestep,
                    phase.ClassificationEpoch,
''')

replace_once(
    P5,
    '''        if (needsClassification)
        {
            PersistentClassificationResults.ResizeUninitialized(
                TimestepInteractionPairs.Length);
            phase.NeedsCommit = 1;
        }
''',
    '''        if (needsClassification)
        {
            phase.ClassificationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
            phase.Timestep = IncrementalCacheState.Value.Timestep;
            phase.ClassificationEpoch = CalculateClassificationEpoch();
            PersistentClassificationResults.ResizeUninitialized(
                TimestepInteractionPairs.Length);
            phase.NeedsCommit = 1;
        }
''')

replace_once(
    P5,
    '''        incremental.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - phase.BuildStartTimestamp);
''',
    '''        incremental.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - phase.ClassificationStartTimestamp);
''')

replace_once(
    P5,
    '''    private bool TryAppendPersistentSpatialNeighborsP1P6(
        int dirtyProxyIndex,
        uint expectedEpoch,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
''',
    '''    private bool TryAppendPersistentSpatialNeighborsP1P6(
        int dirtyProxyIndex,
        uint expectedEpoch,
        uint validatedTimestep,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
''')

replace_once(
    P5,
    '''                        LastValidatedTimestep =
                            IncrementalCacheState.Value.Timestep + 1u
''',
    '''                        LastValidatedTimestep = validatedTimestep
''')

replace_once(
    PERSISTENT,
    '''                    dirtyProxyIndex,
                    nextTopologyEpoch,
                    ref incrementalStatistics))
''',
    '''                    dirtyProxyIndex,
                    nextTopologyEpoch,
                    nextTimestep,
                    ref incrementalStatistics))
''')

replace_once(
    STAGED,
    '''            PersistentPredictiveContacts.Clear();
            if (PersistentIncidentPairLookup.IsCreated)
''',
    '''            PersistentPredictiveContacts.Clear();
            if (PersistentSpatialMembership.IsCreated)
                PersistentSpatialMembership.Clear();
            if (PersistentSpatialMembershipEpoch.IsCreated)
                PersistentSpatialMembershipEpoch.Value = 0;
            if (PersistentIncidentPairLookup.IsCreated)
''')

text = P5.read_text(encoding="utf-8")
checks = {
    "no main-thread cache-state read": "Timestep = IncrementalCacheState.Value.Timestep," not in text.split("private struct EvaluatePersistentPairClassificationsP1P6Job", 1)[0],
    "phase state reader": "PhaseState = PersistentClassificationState" in text,
    "classification timing": "phase.ClassificationStartTimestamp" in text,
    "validated timestep parameter": "uint validatedTimestep" in text and "LastValidatedTimestep = validatedTimestep" in text,
}
failed = [name for name, ok in checks.items() if not ok]
if failed:
    raise RuntimeError("P5B/P5C semantic verification failed: " + ", ".join(failed))
