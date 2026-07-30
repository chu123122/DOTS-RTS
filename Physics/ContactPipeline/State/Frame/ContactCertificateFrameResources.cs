using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Predictive scheduling and certificate publication state only.</summary>
internal struct ContactCertificateFrameResources
{
    public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;
    public NativeReference<int> ScheduleCursor;
    public NativeReference<InteractionCertificate> Certificate;
    public NativeList<InteractionCertificateViolation> Violations;
    public NativeList<PredictiveContactActivationRecord> ActivationRecords;
    public NativeList<byte> ActivationRecordWorkset;
    public NativeList<PredictiveContactActivationBlock> ActivationBlocks;
    public NativeList<byte> ActivationBlockWorkset;
    public NativeList<ContactConstraint> ActivatedContacts;
    public NativeReference<PredictiveContactActivationSummary>
        ActivationSummary;
    public NativeReference<long> ActivationStartTimestamp;

    public static ContactCertificateFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        int scheduleCapacity = math.max(bodyCount * 2, 1);
        return new ContactCertificateFrameResources
        {
            Schedule =
                new NativeList<PredictiveContactScheduleEntry>(
                    scheduleCapacity, Allocator.TempJob),
            ScheduleScratch =
                new NativeList<PredictiveContactScheduleEntry>(
                    scheduleCapacity, Allocator.TempJob),
            ScheduleCursor =
                new NativeReference<int>(Allocator.TempJob),
            Certificate =
                new NativeReference<InteractionCertificate>(
                    Allocator.TempJob),
            Violations =
                new NativeList<InteractionCertificateViolation>(
                    one, Allocator.TempJob),
            ActivationRecords =
                new NativeList<PredictiveContactActivationRecord>(
                    scheduleCapacity, Allocator.TempJob),
            ActivationRecordWorkset =
                new NativeList<byte>(
                    scheduleCapacity, Allocator.TempJob),
            ActivationBlocks =
                new NativeList<PredictiveContactActivationBlock>(
                    one, Allocator.TempJob),
            ActivationBlockWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            ActivatedContacts =
                new NativeList<ContactConstraint>(
                    scheduleCapacity, Allocator.TempJob),
            ActivationSummary =
                new NativeReference<PredictiveContactActivationSummary>(
                    Allocator.TempJob),
            ActivationStartTimestamp =
                new NativeReference<long>(Allocator.TempJob)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = Schedule.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined, ScheduleScratch.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ScheduleCursor.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Certificate.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Violations.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationRecords.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationRecordWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationBlocks.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationBlockWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivatedContacts.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationSummary.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ActivationStartTimestamp.Dispose(finalReader));
        return combined;
    }
}
}
