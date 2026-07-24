from pathlib import Path

path = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs')
text = path.read_text(encoding='utf-8')

# Repair prediction must scale with D, not N.
old = '''            }.Schedule(States.Length, ParallelBodyBatchSize, handle);\n\n            handle = new PrepareP1P6SubstepRepairClassificationJob'''
new = '''            }.Schedule(IncrementalDirtyBodies, ParallelBodyBatchSize, handle);\n\n            handle = new PrepareP1P6SubstepRepairClassificationJob'''
if text.count(old) != 1:
    raise RuntimeError(f'repair schedule anchor count={text.count(old)}')
text = text.replace(old, new, 1)

# Count only. Dirty flags are cleared from the previous compacted workset in Prefix.
text = text.replace(
'''                BlockOffsetsAndCounts = SoftIncidentWriteCursors,\n                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,\n                BodyCount = States.Length,''',
'''                BlockOffsetsAndCounts = SoftIncidentWriteCursors,\n                BodyCount = States.Length,''')

old_count_fields = '''        public NativeArray<int> BlockOffsetsAndCounts;\n        [NativeDisableParallelForRestriction]\n        public NativeArray<byte> DirtyFlagsByBody;\n        public int BodyCount;'''
new_count_fields = '''        public NativeArray<int> BlockOffsetsAndCounts;\n        public int BodyCount;'''
if text.count(old_count_fields) != 1:
    raise RuntimeError(f'count fields anchor count={text.count(old_count_fields)}')
text = text.replace(old_count_fields, new_count_fields, 1)

old_count_loop = '''            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)\n            {\n                DirtyFlagsByBody[bodyIndex] = 0;\n                if (Enabled != 0 && EscapeFlags[bodyIndex] != 0)\n                    count++;\n            }'''
new_count_loop = '''            for (int bodyIndex = begin; bodyIndex < end; bodyIndex++)\n            {\n                if (Enabled != 0 && EscapeFlags[bodyIndex] != 0)\n                    count++;\n            }'''
if text.count(old_count_loop) != 1:
    raise RuntimeError(f'count loop anchor count={text.count(old_count_loop)}')
text = text.replace(old_count_loop, new_count_loop, 1)

# Prefix clears only the previous D dirty flags before replacing the compacted list.
old_prefix_init = '''                BlockOffsetsAndCounts = SoftIncidentWriteCursors,\n                DirtyBodies = IncrementalDirtyBodies,\n                BlockCount = escapeBlockCount'''
new_prefix_init = '''                BlockOffsetsAndCounts = SoftIncidentWriteCursors,\n                DirtyBodies = IncrementalDirtyBodies,\n                DirtyFlagsByBody = IncrementalDirtyFlagsByBody,\n                BlockCount = escapeBlockCount'''
if text.count(old_prefix_init) != 2:
    raise RuntimeError(f'prefix initializer count={text.count(old_prefix_init)}')
text = text.replace(old_prefix_init, new_prefix_init)

old_prefix_fields = '''        public NativeArray<int> BlockOffsetsAndCounts;\n        public NativeList<IncrementalDirtyBody> DirtyBodies;\n        public int BlockCount;'''
new_prefix_fields = '''        public NativeArray<int> BlockOffsetsAndCounts;\n        public NativeList<IncrementalDirtyBody> DirtyBodies;\n        public NativeArray<byte> DirtyFlagsByBody;\n        public int BlockCount;'''
if text.count(old_prefix_fields) != 1:
    raise RuntimeError(f'prefix fields anchor count={text.count(old_prefix_fields)}')
text = text.replace(old_prefix_fields, new_prefix_fields, 1)

old_prefix_execute = '''        public void Execute()\n        {\n            int offset = 0;'''
new_prefix_execute = '''        public void Execute()\n        {\n            for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)\n            {\n                int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;\n                if ((uint)bodyIndex < (uint)DirtyFlagsByBody.Length)\n                    DirtyFlagsByBody[bodyIndex] = 0;\n            }\n\n            int offset = 0;'''
# There are multiple Execute methods with this shape; constrain via nearby struct slice.
prefix_start = text.index('    private struct PrefixP1P6EnvelopeEscapesJob')
prefix_end = text.index('    private struct ScatterP1P6EnvelopeEscapesJob', prefix_start)
prefix_slice = text[prefix_start:prefix_end]
if prefix_slice.count(old_prefix_execute) != 1:
    raise RuntimeError(f'prefix execute anchor count={prefix_slice.count(old_prefix_execute)}')
prefix_slice = prefix_slice.replace(old_prefix_execute, new_prefix_execute, 1)
text = text[:prefix_start] + prefix_slice + text[prefix_end:]

# Escape is an observed correction/motion hotspot. Topology is promoted only after Guard validation.
old_flags = '''                const IncrementalBodyDirtyFlags flags =\n                    IncrementalBodyDirtyFlags.Topology |\n                    IncrementalBodyDirtyFlags.Motion;'''
new_flags = '''                const IncrementalBodyDirtyFlags flags =\n                    IncrementalBodyDirtyFlags.Motion |\n                    IncrementalBodyDirtyFlags.CorrectedEscape;'''
if text.count(old_flags) != 1:
    raise RuntimeError(f'escape flags anchor count={text.count(old_flags)}')
text = text.replace(old_flags, new_flags, 1)

# Deferred repair job indexes DirtyBodies, then updates only that body.
old_execute = '''        public void Execute(int bodyIndex)\n        {\n            if (Enabled == 0 || DirtyBodies.Length == 0)\n                return;\n\n            FlowMovementFrameState state = States[bodyIndex];'''
new_execute = '''        public void Execute(int dirtyIndex)\n        {\n            if (Enabled == 0)\n                return;\n\n            int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;\n            if ((uint)bodyIndex >= (uint)States.Length)\n                return;\n            FlowMovementFrameState state = States[bodyIndex];'''
repair_start = text.index('    private struct PrepareP1P6RepairPredictionBodiesJob')
repair_end = text.index('    private struct PrepareP1P6SubstepRepairClassificationJob', repair_start)
repair_slice = text[repair_start:repair_end]
if repair_slice.count(old_execute) != 1:
    raise RuntimeError(f'repair execute anchor count={repair_slice.count(old_execute)}')
repair_slice = repair_slice.replace(old_execute, new_execute, 1)
text = text[:repair_start] + repair_slice + text[repair_end:]

# Static assertions for the intended batch boundary.
assert '.Schedule(IncrementalDirtyBodies, ParallelBodyBatchSize, handle);' in text
assert 'DirtyFlagsByBody[bodyIndex] = 0;\n                if (Enabled' not in text
assert 'IncrementalBodyDirtyFlags.Topology |\n                    IncrementalBodyDirtyFlags.Motion' not in text
assert 'int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;' in repair_slice

path.write_text(text, encoding='utf-8')
print('Applied simulation integration batch 1')
