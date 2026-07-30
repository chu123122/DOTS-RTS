using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PrepareCurrentBodyIndexJob : IJob
{
    public NativeParallelHashMap<Unity.Entities.Entity, int>
        CurrentBodyIndexByEntity;
    public int BodyCount;

    public void Execute()
    {
        CurrentBodyIndexByEntity.Clear();
        if (CurrentBodyIndexByEntity.Capacity < BodyCount)
            CurrentBodyIndexByEntity.Capacity = BodyCount;
    }
}

[BurstCompile]
internal struct BuildCurrentBodyIndexJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeParallelHashMap<Unity.Entities.Entity, int>.ParallelWriter
        CurrentBodyIndexByEntity;

    public void Execute(int bodyIndex)
    {
        CurrentBodyIndexByEntity.TryAdd(Bodies[bodyIndex].Entity, bodyIndex);
    }
}

public struct DirtyContactScheduleBlock
{
    public int ContactCount;
    public int ScheduleCount;
}

[BurstCompile]
internal struct PrepareDirtyContactScheduleBlocksJob : IJob
{
    [ReadOnly] public NativeList<PersistentPredictiveContact> Contacts;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> Schedule;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeList<DirtyContactScheduleBlock> BlockCounts;
    public NativeList<DirtyContactScheduleBlock> BlockOffsets;
    public int BlockSize;

    public void Execute()
    {
        if (DirtyBodies.Length == 0)
        {
            BlockCounts.Clear();
            BlockOffsets.Clear();
            return;
        }
        int itemCount = math.max(Contacts.Length, Schedule.Length);
        int blockCount = (itemCount + BlockSize - 1) / BlockSize;
        BlockCounts.ResizeUninitialized(blockCount);
        BlockOffsets.ResizeUninitialized(blockCount);
    }
}

[BurstCompile]
internal struct CountDirtyContactScheduleJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    [ReadOnly] public NativeArray<PredictiveContactScheduleEntry> Schedule;
    [ReadOnly] public NativeParallelHashMap<Unity.Entities.Entity, int>
        CurrentBodyIndexByEntity;
    [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
    public NativeArray<DirtyContactScheduleBlock> BlockCounts;
    [ReadOnly] public NativeReference<int> ScheduleCursor;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        DirtyContactScheduleBlock count = default;
        int begin = blockIndex * BlockSize;
        int end = begin + BlockSize;
        for (int contactIndex = begin;
             contactIndex < math.min(end, Contacts.Length);
             contactIndex++)
        {
            StableEntityPairKey key = Contacts[contactIndex].Key;
            if (!IsDirty(key.EntityA) && !IsDirty(key.EntityB))
                count.ContactCount++;
        }
        int scheduleBegin = math.max(
            begin, math.clamp(ScheduleCursor.Value, 0, Schedule.Length));
        for (int scheduleIndex = scheduleBegin;
             scheduleIndex < math.min(end, Schedule.Length);
             scheduleIndex++)
        {
            StableEntityPairKey key = Schedule[scheduleIndex].Key;
            if (!IsDirty(key.EntityA) && !IsDirty(key.EntityB))
                count.ScheduleCount++;
        }
        BlockCounts[blockIndex] = count;
    }

    private bool IsDirty(Unity.Entities.Entity entity)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out int bodyIndex) &&
               (uint)bodyIndex < (uint)DirtyFlagsByBody.Length &&
               DirtyFlagsByBody[bodyIndex] != 0;
    }
}

[BurstCompile]
internal struct PrefixDirtyContactScheduleJob : IJob
{
    [ReadOnly] public NativeArray<DirtyContactScheduleBlock> BlockCounts;
    public NativeArray<DirtyContactScheduleBlock> BlockOffsets;
    public NativeList<PersistentPredictiveContact> ContactScratch;
    public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;

    public void Execute()
    {
        int contactOffset = 0;
        int scheduleOffset = 0;
        for (int blockIndex = 0; blockIndex < BlockCounts.Length; blockIndex++)
        {
            BlockOffsets[blockIndex] = new DirtyContactScheduleBlock
            {
                ContactCount = contactOffset,
                ScheduleCount = scheduleOffset
            };
            contactOffset += BlockCounts[blockIndex].ContactCount;
            scheduleOffset += BlockCounts[blockIndex].ScheduleCount;
        }
        ContactScratch.ResizeUninitialized(contactOffset);
        ScheduleScratch.ResizeUninitialized(scheduleOffset);
    }
}

[BurstCompile]
internal struct ScatterDirtyContactScheduleJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    [ReadOnly] public NativeArray<PredictiveContactScheduleEntry> Schedule;
    [ReadOnly] public NativeParallelHashMap<Unity.Entities.Entity, int>
        CurrentBodyIndexByEntity;
    [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
    [ReadOnly] public NativeArray<DirtyContactScheduleBlock> BlockCounts;
    [ReadOnly] public NativeArray<DirtyContactScheduleBlock> BlockOffsets;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentPredictiveContact> ContactScratch;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactScheduleEntry> ScheduleScratch;
    [ReadOnly] public NativeReference<int> ScheduleCursor;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        DirtyContactScheduleBlock offsets = BlockOffsets[blockIndex];
        int contactWrite = offsets.ContactCount;
        int scheduleWrite = offsets.ScheduleCount;
        int begin = blockIndex * BlockSize;
        int end = begin + BlockSize;
        for (int contactIndex = begin;
             contactIndex < math.min(end, Contacts.Length);
             contactIndex++)
        {
            PersistentPredictiveContact contact = Contacts[contactIndex];
            if (IsDirty(contact.Key.EntityA) || IsDirty(contact.Key.EntityB))
                continue;
            ContactScratch[contactWrite++] = contact;
        }
        int scheduleBegin = math.max(
            begin, math.clamp(ScheduleCursor.Value, 0, Schedule.Length));
        for (int scheduleIndex = scheduleBegin;
             scheduleIndex < math.min(end, Schedule.Length);
             scheduleIndex++)
        {
            PredictiveContactScheduleEntry entry = Schedule[scheduleIndex];
            if (IsDirty(entry.Key.EntityA) || IsDirty(entry.Key.EntityB))
                continue;
            ScheduleScratch[scheduleWrite++] = entry;
        }
    }

    private bool IsDirty(Unity.Entities.Entity entity)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out int bodyIndex) &&
               (uint)bodyIndex < (uint)DirtyFlagsByBody.Length &&
               DirtyFlagsByBody[bodyIndex] != 0;
    }
}

[BurstCompile]
internal struct CommitDirtyContactScheduleJob : IJob
{
    [ReadOnly] public NativeList<PersistentPredictiveContact> ContactScratch;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeList<PersistentPredictiveContact> Contacts;
    public NativeParallelHashMap<StableEntityPairKey, int> ContactIndex;
    public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeReference<int> ScheduleCursor;

    public void Execute()
    {
        if (DirtyBodies.Length == 0)
            return;
        Contacts.Clear();
        Contacts.AddRange(ContactScratch.AsArray());
        if (ContactIndex.IsCreated)
            ContactIndex.Clear();
        Schedule.Clear();
        Schedule.AddRange(ScheduleScratch.AsArray());
        ScheduleCursor.Value = 0;
    }
}

[BurstCompile]
internal struct BuildDirtyContactIndexJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    public NativeParallelHashMap<StableEntityPairKey, int>.ParallelWriter
        ContactIndex;

    public void Execute(int contactIndex)
    {
        ContactIndex.TryAdd(Contacts[contactIndex].Key, contactIndex);
    }
}
}
