namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// Compile-time status exposed to runtime UI and validation code without using
/// editor APIs. When the diagnostics macro is absent this class is the only
/// diagnostics build artifact that needs to remain available.
/// </summary>
public static class SimulationDiagnosticsCompileStatus
{
#if RTS_CONTACT_DIAGNOSTICS
    public const bool Enabled = true;
    public const string Label = "Enabled";
#else
    public const bool Enabled = false;
    public const string Label = "Disabled";
#endif
}
}
