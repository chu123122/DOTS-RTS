using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PrepareActiveIncidentIndexJob : IJob
{
    [ReadOnly] public NativeReference<InteractionCertificate> Certificate;
    [ReadOnly] public NativeList<ContactConstraint> Pairs;
    public NativeList<byte> BodyWorkset;
    public NativeList<byte> PairWorkset;
    public NativeList<int> IncidentPairIndices;
    public NativeReference<ActiveIncidentIndexState> State;
    public int BodyCount;
    public byte Enabled;

    public void Execute()
    {
        if (Enabled == 0)
        {
            BodyWorkset.Clear();
            PairWorkset.Clear();
            IncidentPairIndices.Clear();
            State.Value = default;
            return;
        }

        InteractionCertificate certificate = Certificate.Value;
        ulong fingerprint = 1469598103934665603UL;
        fingerprint = Mix(fingerprint, certificate.SimulationStepId);
        fingerprint = Mix(fingerprint, certificate.TopologyEpoch);
        fingerprint = Mix(
            fingerprint, certificate.ClassificationFingerprint);
        fingerprint = Mix(
            fingerprint,
            ((ulong)certificate.StartSubstep << 48) |
            ((ulong)certificate.EndSubstepExclusive << 32) |
            unchecked((uint)Pairs.Length));
        fingerprint = Mix(
            fingerprint,
            ((ulong)(uint)certificate.Flags << 32) |
            unchecked((uint)certificate.ContactConstraintCount));

        ActiveIncidentIndexState state = State.Value;
        if (state.IsValid != 0 &&
            state.Fingerprint == fingerprint &&
            state.PairCount == Pairs.Length &&
            state.BodyCount == BodyCount)
        {
            BodyWorkset.Clear();
            PairWorkset.Clear();
            return;
        }

        BodyWorkset.ResizeUninitialized(BodyCount);
        PairWorkset.ResizeUninitialized(Pairs.Length);
        state.Fingerprint = fingerprint;
        state.PairCount = Pairs.Length;
        state.BodyCount = BodyCount;
        state.IsValid = 2;
        State.Value = state;
    }

    private static ulong Mix(ulong hash, ulong value) =>
        (hash ^ value) * 1099511628211UL;
}

[BurstCompile]
internal struct ClearActiveIncidentCountsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    public NativeArray<int> Counts;

    public void Execute(int bodyIndex)
    {
        Counts[bodyIndex] = 0;
    }
}

[BurstCompile]
internal unsafe struct CountActiveIncidentPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> Pairs;
    [NativeDisableParallelForRestriction]
    public NativeArray<int> Counts;

    public void Execute(int pairIndex)
    {
        ContactConstraint pair = Pairs[pairIndex];
        Increment(pair.BodyA);
        Increment(pair.BodyB);
    }

    private void Increment(int bodyIndex)
    {
        ref int count = ref UnsafeUtility.ArrayElementAsRef<int>(
            NativeArrayUnsafeUtility.GetUnsafePtr(Counts),
            bodyIndex);
        Interlocked.Increment(ref count);
    }
}

[BurstCompile]
internal struct PrefixActiveIncidentPairsJob : IJob
{
    public NativeArray<int> CountsAndWriteCursors;
    public NativeArray<int> Offsets;
    public NativeList<int> IncidentPairIndices;
    public NativeReference<ActiveIncidentIndexState> State;
    [ReadOnly] public NativeList<ContactConstraint> Pairs;
    public int BodyCount;

    public void Execute()
    {
        ActiveIncidentIndexState state = State.Value;
        if (state.IsValid != 2)
            return;

        int entryCount = 0;
        Offsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < BodyCount; bodyIndex++)
        {
            entryCount += CountsAndWriteCursors[bodyIndex];
            Offsets[bodyIndex + 1] = entryCount;
            CountsAndWriteCursors[bodyIndex] = Offsets[bodyIndex];
        }
        IncidentPairIndices.ResizeUninitialized(entryCount);
        state.PairCount = Pairs.Length;
        state.BodyCount = BodyCount;
        state.IsValid = 1;
        State.Value = state;
    }
}

[BurstCompile]
internal unsafe struct ScatterActiveIncidentPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> Pairs;
    [NativeDisableParallelForRestriction]
    public NativeArray<int> WriteCursors;
    [NativeDisableParallelForRestriction]
    public NativeArray<int> IncidentPairIndices;

    public void Execute(int pairIndex)
    {
        ContactConstraint pair = Pairs[pairIndex];
        IncidentPairIndices[Reserve(pair.BodyA)] = pairIndex;
        IncidentPairIndices[Reserve(pair.BodyB)] = pairIndex;
    }

    private int Reserve(int bodyIndex)
    {
        ref int cursor = ref UnsafeUtility.ArrayElementAsRef<int>(
            NativeArrayUnsafeUtility.GetUnsafePtr(WriteCursors),
            bodyIndex);
        return Interlocked.Increment(ref cursor) - 1;
    }
}

[BurstCompile]
internal struct SortActiveIncidentRangesJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<int> Offsets;
    [NativeDisableParallelForRestriction]
    public NativeArray<int> IncidentPairIndices;

    public void Execute(int bodyIndex)
    {
        int begin = Offsets[bodyIndex];
        int end = Offsets[bodyIndex + 1];
        for (int index = begin + 1; index < end; index++)
        {
            int value = IncidentPairIndices[index];
            int insert = index - 1;
            while (insert >= begin &&
                   IncidentPairIndices[insert] > value)
            {
                IncidentPairIndices[insert + 1] =
                    IncidentPairIndices[insert];
                insert--;
            }
            IncidentPairIndices[insert + 1] = value;
        }
    }
}

[BurstCompile]
internal struct ResizeParallelContactWorksetsJob : IJob
{
    [ReadOnly] public NativeList<ContactConstraint> Pairs;
    public NativeList<JacobiPairCorrection> Corrections;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeList<ParallelSimulationDebuggerPairCapture>
        DebuggerPairCandidates;
    public NativeList<JacobiBlockTelemetry> Blocks;
#endif
    public byte Enabled;

    public void Execute()
    {
        if (Enabled == 0)
            return;
        Corrections.ResizeUninitialized(Pairs.Length);
#if RTS_CONTACT_DIAGNOSTICS
        if (DebuggerPairCandidates.IsCreated)
            DebuggerPairCandidates.ResizeUninitialized(Pairs.Length);
        Blocks.ResizeUninitialized(
            (Pairs.Length +
             CrowdContactPipelineScheduler.JacobiPairBatchSize - 1) /
            CrowdContactPipelineScheduler.JacobiPairBatchSize);
#endif
    }
}
}
