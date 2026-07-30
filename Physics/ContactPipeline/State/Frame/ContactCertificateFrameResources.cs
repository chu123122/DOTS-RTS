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

    public static ContactCertificateFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        return new ContactCertificateFrameResources
        {
            Schedule =
                new NativeList<PredictiveContactScheduleEntry>(
                    math.max(bodyCount * 2, 1), Allocator.TempJob),
            ScheduleScratch =
                new NativeList<PredictiveContactScheduleEntry>(
                    one, Allocator.TempJob),
            ScheduleCursor =
                new NativeReference<int>(Allocator.TempJob),
            Certificate =
                new NativeReference<InteractionCertificate>(
                    Allocator.TempJob),
            Violations =
                new NativeList<InteractionCertificateViolation>(
                    one, Allocator.TempJob)
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
        return combined;
    }
}
}
