using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void EnsureActiveConstraintIncidentIndex()
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
        EnsureActiveConstraintIncidentIndex();
    }
}
}
