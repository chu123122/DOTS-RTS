using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Persistent-pair mapping and classification worksets only.</summary>
internal struct ContactClassificationFrameResources
{
    public NativeList<BodyPair> BodyPairs;
    public NativeList<PersistentPairClassificationResult> Results;
    public NativeReference<PersistentClassificationPhaseState> State;
    public NativeList<ClassificationPublicationRecord> PublicationRecords;
    public NativeList<ClassificationPublicationBlock> PublicationBlocks;
    public NativeList<byte> PublicationBlockWorkset;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> Telemetry;
#endif

    public static ContactClassificationFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        return new ContactClassificationFrameResources
        {
            BodyPairs = new NativeList<BodyPair>(
                math.max(bodyCount * 8, 1), Allocator.TempJob),
            Results =
                new NativeList<PersistentPairClassificationResult>(
                    math.max(bodyCount * 8, 1), Allocator.TempJob),
            State = new NativeReference<PersistentClassificationPhaseState>(
                Allocator.TempJob),
            PublicationRecords =
                new NativeList<ClassificationPublicationRecord>(
                    math.max(bodyCount * 8, 1), Allocator.TempJob),
            PublicationBlocks =
                new NativeList<ClassificationPublicationBlock>(
                    one, Allocator.TempJob),
            PublicationBlockWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
#if RTS_CONTACT_DIAGNOSTICS
            Telemetry =
                new NativeReference<PersistentClassificationTelemetryState>(
                    Allocator.TempJob),
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = BodyPairs.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined, Results.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, State.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PublicationRecords.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PublicationBlocks.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PublicationBlockWorkset.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        combined = JobHandle.CombineDependencies(
            combined, Telemetry.Dispose(finalReader));
#endif
        return combined;
    }
}
}
