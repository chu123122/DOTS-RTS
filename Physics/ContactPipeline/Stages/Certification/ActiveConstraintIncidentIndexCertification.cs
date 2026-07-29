using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationAlgorithms
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
}
}
