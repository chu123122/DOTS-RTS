using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
    private void EnsureActiveConstraintIncidentIndex()
    {
        ActiveConstraintIncidentIndexBuilder.Ensure(
            Configuration.ContactPositionSolver,
            Bodies.Length,
            TimestepContactPairs,
            ActiveIncidentIndexState,
            ActiveIncidentOffsets,
            ActiveIncidentWriteCursors,
            ActiveIncidentPairIndices);
    }
}
}
