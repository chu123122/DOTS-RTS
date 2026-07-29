namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// 编译期状态暴露给运行时 UI 和校验（不依赖 Editor API）；未定义宏时仅此类型保留。
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
