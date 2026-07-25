using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Frame-local CSR index from a body slot to active timestep contact-pair indices.
/// It is rebuilt only when the active contact view changes and is shared by the
/// serial reference and parallel Jacobi gather paths.
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

        state.Fingerprint = fingerprint;
        state.PairCount = constraints.Length;
        state.BodyCount = bodyCount;
        state.IsValid = 1;
        indexState.Value = state;
    }
}

public partial struct InteractionCertificationJob
{
    private void EnsureActiveConstraintIncidentIndexP1P6()
    {
        ActiveConstraintIncidentIndexBuilder.Ensure(
            ContactPositionSolver,
            Bodies.Length,
            TimestepContactPairs,
            ActiveIncidentIndexState,
            ActiveIncidentOffsets,
            ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices);
    }
}

public partial struct ConstraintSolverJob
{
    private void EnsureActiveConstraintIncidentIndexP1P6()
    {
        ActiveConstraintIncidentIndexBuilder.Ensure(
            ContactPositionSolver,
            Bodies.Length,
            TimestepContactPairs,
            ActiveIncidentIndexState,
            ActiveIncidentOffsets,
            ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices);
    }

    private void RebuildActiveConstraintIncidentIndexIfNeeded()
    {
        EnsureActiveConstraintIncidentIndexP1P6();
    }
}
}
