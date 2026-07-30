using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>Pure accounting math shared by diagnostic stage boundaries.</summary>
internal static class ContactPipelineDiagnosticsMath
{
    internal static long AccountedCandidateNanoseconds(
        IncrementalContactPipelineStatistics statistics) =>
        statistics.ProxyValidationNanoseconds +
        statistics.FullSweepSourceNanoseconds +
        statistics.PersistentPairMappingNanoseconds +
        statistics.LocalBroadPhaseNanoseconds +
        statistics.PairDiffNanoseconds +
        statistics.FallbackNanoseconds +
        statistics.SweptClassificationNanoseconds +
        statistics.ContactActivationNanoseconds;
}
}
