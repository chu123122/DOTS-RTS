namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Authoritative gameplay feature options. Diagnostics may submit overrides, but
/// the solver never reads presentation-owned global state.
/// </summary>
public static class ContactPipelineRuntimeOptions
{
    private static byte _timestepContactSetCacheEnabled = 1;

    public static bool TimestepContactSetCacheEnabled
    {
        get => _timestepContactSetCacheEnabled != 0;
        set => _timestepContactSetCacheEnabled = (byte)(value ? 1 : 0);
    }
}
}
