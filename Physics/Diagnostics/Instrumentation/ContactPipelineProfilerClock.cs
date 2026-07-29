namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 接触管线计时编译门面：诊断构建代理到 Unity Profiler；非诊断构建返回常量，便于 Burst 消除计时。
/// </summary>
internal static class ProfilerUnsafeUtility
{
    internal readonly struct TimestampConversionRatio
    {
        public readonly long Numerator;
        public readonly long Denominator;

        public TimestampConversionRatio(long numerator, long denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    public static long Timestamp
    {
        get
        {
#if RTS_CONTACT_DIAGNOSTICS
            return Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp;
#else
            return 0L;
#endif
        }
    }

    public static TimestampConversionRatio TimestampToNanosecondsConversionRatio
    {
        get
        {
#if RTS_CONTACT_DIAGNOSTICS
            var ratio = Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility
                .TimestampToNanosecondsConversionRatio;
            return new TimestampConversionRatio(ratio.Numerator, ratio.Denominator);
#else
            return new TimestampConversionRatio(0L, 1L);
#endif
        }
    }
}
}
