namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Compile-time facade for contact-pipeline timing.
///
/// Job files intentionally reference this type by its simple name. Because it
/// lives in the job namespace it takes precedence over the imported Unity type.
/// Diagnostics builds forward to Unity's profiler clock; gameplay-only builds
/// return constants, allowing Burst to remove timing expressions.
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
