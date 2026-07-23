namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Frame-local CSR index from a body slot to active timestep contact-pair indices.
/// It is rebuilt only when the active contact view changes and is shared by the
/// serial reference and parallel Jacobi gather paths.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void RebuildActiveConstraintIncidentIndexIfNeeded()
    {
        EnsureActiveConstraintIncidentIndexP1P6();
    }
}
}
