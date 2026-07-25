using Unity.Entities;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    /// <summary>
    /// Resets Native capture outputs only when diagnostics are compiled.
    /// Gameplay-only jobs must never touch default Native containers.
    /// </summary>
    private void ResetContactDiagnosticsCapture()
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (IterationDiagnostics.IsCreated)
            IterationDiagnostics.Clear();
        if (PairDiagnostics.IsCreated)
            PairDiagnostics.Clear();
        if (SelectedBodyDiagnostic.IsCreated)
            SelectedBodyDiagnostic.Value = default;
        ResetSimulationDebuggerCapture();
#endif
    }
}
}
