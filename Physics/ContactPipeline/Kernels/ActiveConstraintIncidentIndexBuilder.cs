using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 由 body 槽位到该 timestep 活跃接触对下标的帧级 CSR 索引。仅在活跃接触视图变化时重建，
/// 由串行参考路径与并行 Jacobi 收集路径共享。
/// </summary>
internal static class ActiveConstraintIncidentIndexBuilder
{
    internal static void Ensure(
        ContactPositionSolverMode solverMode,
        int bodyCount,
        NativeList<ContactConstraint> constraints,
        NativeReference<ActiveIncidentIndexState> indexState,
        NativeArray<int> offsets,
        NativeArray<int> writeCursors,
        NativeList<int> incidentPairIndices)
    {
        if (solverMode != ContactPositionSolverMode.Jacobi)
            return;

        ulong fingerprint = 1469598103934665603UL;
        for (int pairIndex = 0; pairIndex < constraints.Length; pairIndex++)
        {
            ContactConstraint pair = constraints[pairIndex];
            fingerprint = (fingerprint ^ (uint)pair.BodyA) * 1099511628211UL;
            fingerprint = (fingerprint ^ (uint)pair.BodyB) * 1099511628211UL;
        }

        ActiveIncidentIndexState state = indexState.Value;
        if (state.IsValid != 0 &&
            state.Fingerprint == fingerprint &&
            state.PairCount == constraints.Length &&
            state.BodyCount == bodyCount)
            return;

        for (int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
            writeCursors[bodyIndex] = 0;
        for (int pairIndex = 0; pairIndex < constraints.Length; pairIndex++)
        {
            ContactConstraint pair = constraints[pairIndex];
            writeCursors[pair.BodyA]++;
            writeCursors[pair.BodyB]++;
        }

        int entries = 0;
        offsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
        {
            entries += writeCursors[bodyIndex];
            offsets[bodyIndex + 1] = entries;
            writeCursors[bodyIndex] = offsets[bodyIndex];
        }

        incidentPairIndices.ResizeUninitialized(entries);
        for (int pairIndex = 0; pairIndex < constraints.Length; pairIndex++)
        {
            ContactConstraint pair = constraints[pairIndex];
            incidentPairIndices[writeCursors[pair.BodyA]++] = pairIndex;
            incidentPairIndices[writeCursors[pair.BodyB]++] = pairIndex;
        }

        // 不变量：每对恰好贡献两个事件项（BodyA + BodyB），所以事件列表长度必须是 2 * constraints.Length。
        // 不一致会导致 GatherAndApplyParallelJacobiBodiesJob 用本次 offsets 越界。
        // 在源头断言，比让 Burst 在消费者里抛 IndexOutOfRangeException 更容易定位。
        if (entries != constraints.Length * 2)
            throw new System.IndexOutOfRangeException(
                "Active incident index rebuild produced " + entries +
                " entries for " + constraints.Length +
                " constraints; expected exactly " + (constraints.Length * 2) +
                " (each pair contributes BodyA + BodyB).");

        state.Fingerprint = fingerprint;
        state.PairCount = constraints.Length;
        state.BodyCount = bodyCount;
        state.IsValid = 1;
        indexState.Value = state;
    }
}
}
