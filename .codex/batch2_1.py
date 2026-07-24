from pathlib import Path

BASE = Path('Entities/Unit/Systems/FlowField/BaseFlowMovementSystem.cs')
CORE = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Core/SolveXpbdUnitContactsJob.cs')
P1 = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs')


def rep(text, old, new, name):
    if old not in text:
        raise RuntimeError('missing ' + name)
    return text.replace(old, new, 1)

base = BASE.read_text()
base = rep(base,
'    private NativeList<PersistentSweptProxy> _persistentSweptProxies;\n',
'    private NativeList<PersistentSweptProxy> _persistentSweptProxies;\n    private NativeList<int> _persistentProxyIndexByBody;\n', 'base field')
base = rep(base,
'        _persistentSweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent);\n',
'        _persistentSweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent);\n        _persistentProxyIndexByBody = new NativeList<int>(Allocator.Persistent);\n', 'base create')
base = rep(base,
'        if (_persistentSweptProxies.IsCreated)\n            _persistentSweptProxies.Dispose();\n',
'        if (_persistentSweptProxies.IsCreated)\n            _persistentSweptProxies.Dispose();\n        if (_persistentProxyIndexByBody.IsCreated)\n            _persistentProxyIndexByBody.Dispose();\n', 'base destroy')
base = rep(base,
'        EnsurePersistentIncidentLookupCapacity(unitCount);\n',
'        EnsurePersistentIncidentLookupCapacity(unitCount);\n        if (_persistentProxyIndexByBody.Capacity < unitCount)\n            _persistentProxyIndexByBody.Capacity = unitCount;\n', 'base capacity')
base = rep(base,
'            PersistentSweptProxies = _persistentSweptProxies,\n',
'            PersistentSweptProxies = _persistentSweptProxies,\n            PersistentProxyIndexByBody = _persistentProxyIndexByBody,\n', 'base assign')
base = rep(base,
'        _persistentSweptProxies.Clear();\n        _persistentNeighborPairs.Clear();\n',
'        _persistentSweptProxies.Clear();\n        _persistentProxyIndexByBody.Clear();\n        _persistentNeighborPairs.Clear();\n', 'base reset')
BASE.write_text(base)

core = CORE.read_text()
core = rep(core,
'    public NativeList<PersistentSweptProxy> PersistentSweptProxies;\n',
'    public NativeList<PersistentSweptProxy> PersistentSweptProxies;\n    public NativeList<int> PersistentProxyIndexByBody;\n', 'core field')
core = rep(core,
'            PersistentSweptProxies.Clear();\n            PersistentNeighborPairs.Clear();\n',
'            PersistentSweptProxies.Clear();\n            PersistentProxyIndexByBody.Clear();\n            PersistentNeighborPairs.Clear();\n', 'core clear')
core = rep(core,
'            PrepareTimestepContactPrediction(DeltaTime, false);\n            long initialContactSetStart',
'            PrepareTimestepContactPrediction(DeltaTime, false);\n            if (EnablePersistentContactCache)\n                PrepareInitialPersistentDirtyBodySet();\n            long initialContactSetStart', 'serial dirty')
CORE.write_text(core)

p = P1.read_text()
old = '''            handle = new PrepareTimestepPredictionBodiesJob
            {
                States = States,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                FromSolvedPosition = 0,
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);'''
new = '''            handle = new PrepareTimestepPredictionBodiesJob
            {
                States = States,
                PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
                PersistentProxyIndexByBody = PersistentProxyIndexByBody.AsDeferredJobArray(),
                PersistentCacheState = IncrementalCacheState,
                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                Duration = Configuration.DeltaTime,
                Skin = Configuration.PredictiveSkin,
                Margin = Configuration.TimestepContactMargin,
                GuardMargin = Configuration.GuardEnvelopeMargin,
                GridOrigin = GridOrigin,
                CellRadius = CellRadius,
                FromSolvedPosition = 0,
                DetectPersistentDirty = (byte)(Configuration.EnablePersistentContactCache ? 1 : 0),
                SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = Configuration.RvoTimeHorizon
            }.Schedule(States.Length, ParallelBodyBatchSize, handle);

            if (Configuration.EnablePersistentContactCache)
            {
                handle = new CountInitialP1P6DirtyBodyBlocksJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);
                handle = new PrefixInitialP1P6DirtyBodiesJob
                {
                    BlockOffsetsAndCounts = SoftIncidentWriteCursors,
                    DirtyBodies = IncrementalDirtyBodies,
                    BlockCount = escapeBlockCount
                }.Schedule(handle);
                handle = new ScatterInitialP1P6DirtyBodiesJob
                {
                    DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
                    BlockOffsets = SoftIncidentWriteCursors,
                    DirtyBodies = IncrementalDirtyBodies.AsDeferredJobArray(),
                    BodyCount = States.Length
                }.Schedule(escapeBlockCount, 1, handle);
            }'''
p = rep(p, old, new, 'initial prediction')
old = '''                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon
                }.Schedule(States.Length, ParallelBodyBatchSize, handle);'''
new = '''                    SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
                    SoftSolverMode = Configuration.SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon = Configuration.RvoTimeHorizon,
                    DetectPersistentDirty = 0
                }.Schedule(States.Length, ParallelBodyBatchSize, handle);'''
p = rep(p, old, new, 'substep prediction')
old = '''        public NativeArray<FlowMovementFrameState> States;
        public float Duration;
        public float Skin;
        public float Margin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;'''
new = '''        public NativeArray<FlowMovementFrameState> States;
        [ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;
        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
        [ReadOnly] public NativeReference<IncrementalContactCacheState> PersistentCacheState;
        public NativeArray<byte> DirtyFlagsByBody;
        public float Duration;
        public float Skin;
        public float Margin;
        public float GuardMargin;
        public float3 GridOrigin;
        public float CellRadius;
        public byte FromSolvedPosition;
        public byte DetectPersistentDirty;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftSolverMode;
        public float RvoTimeHorizon;'''
p = rep(p, old, new, 'prediction fields')
p = rep(p,
'''            if (!state.IsInsideGrid)
                return;

            float3 start = FromSolvedPosition != 0''',
'''            if (!state.IsInsideGrid)
            {
                if (DetectPersistentDirty != 0)
                    DirtyFlagsByBody[bodyIndex] = (byte)ClassifyAndUpdatePersistentProxyForBodyP1P6(
                        bodyIndex, state, PersistentProxies, PersistentProxyIndexByBody,
                        PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                        SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
                return;
            }

            float3 start = FromSolvedPosition != 0''', 'invalid state')
p = rep(p,
'''            if (FromSolvedPosition == 0)
                state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }
    }

    [BurstCompile]
    private struct PrepareBaseVelocityBodiesJob''',
'''            if (FromSolvedPosition == 0)
                state.TimestepEscaped = 0;
            States[bodyIndex] = state;
            if (DetectPersistentDirty != 0)
                DirtyFlagsByBody[bodyIndex] = (byte)ClassifyAndUpdatePersistentProxyForBodyP1P6(
                    bodyIndex, state, PersistentProxies, PersistentProxyIndexByBody,
                    PersistentCacheState.Value, GuardMargin, SoftAvoidanceShell,
                    SoftAvoidanceResponseRate, SoftSolverMode, RvoTimeHorizon);
        }
    }

    [BurstCompile]
    private struct CountInitialP1P6DirtyBodyBlocksJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        public NativeArray<int> BlockOffsetsAndCounts;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int count = 0;
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
                count += DirtyFlagsByBody[bodyIndex] != 0 ? 1 : 0;
            BlockOffsetsAndCounts[blockIndex] = count;
        }
    }

    [BurstCompile]
    private struct PrefixInitialP1P6DirtyBodiesJob : IJob
    {
        public NativeArray<int> BlockOffsetsAndCounts;
        public NativeList<IncrementalDirtyBody> DirtyBodies;
        public int BlockCount;
        public void Execute()
        {
            int offset = 0;
            for (int blockIndex = 0; blockIndex < BlockCount; blockIndex++)
            {
                int count = BlockOffsetsAndCounts[blockIndex];
                BlockOffsetsAndCounts[blockIndex] = offset;
                offset += count;
            }
            DirtyBodies.ResizeUninitialized(offset);
        }
    }

    [BurstCompile]
    private struct ScatterInitialP1P6DirtyBodiesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeArray<int> BlockOffsets;
        [NativeDisableParallelForRestriction] public NativeArray<IncrementalDirtyBody> DirtyBodies;
        public int BodyCount;
        public void Execute(int blockIndex)
        {
            int begin = blockIndex * ParallelBodyBatchSize;
            int end = math.min(begin + ParallelBodyBatchSize, BodyCount);
            int writeIndex = BlockOffsets[blockIndex];
            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)
            {
                IncrementalBodyDirtyFlags flags = (IncrementalBodyDirtyFlags)DirtyFlagsByBody[bodyIndex];
                if (flags == IncrementalBodyDirtyFlags.None)
                    continue;
                DirtyBodies[writeIndex++] = new IncrementalDirtyBody { BodyIndex = bodyIndex, Flags = flags };
            }
        }
    }

    [BurstCompile]
    private struct PrepareBaseVelocityBodiesJob''', 'compact jobs')
P1.write_text(p)
print('batch2_1 ok')
