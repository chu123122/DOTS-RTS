using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
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
