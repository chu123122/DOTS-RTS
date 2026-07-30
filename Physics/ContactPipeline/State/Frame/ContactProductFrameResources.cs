using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>BroadPhase writes; NarrowPhase reads.</summary>
public readonly struct BroadPhaseCandidateBatch
{
    [ReadOnly] public readonly NativeList<BodyPair> Pairs;

    public BroadPhaseCandidateBatch(NativeList<BodyPair> pairs)
    {
        Pairs = pairs;
    }
}

/// <summary>NarrowPhase writes; Solver reads.</summary>
public readonly struct NarrowPhaseConstraintBatch
{
    [ReadOnly] public readonly NativeList<BodyPair> SoftInteractions;
    [ReadOnly] public readonly NativeList<ContactConstraint> HardContacts;

    public NarrowPhaseConstraintBatch(
        NativeList<BodyPair> softInteractions,
        NativeList<ContactConstraint> hardContacts)
    {
        SoftInteractions = softInteractions;
        HardContacts = hardContacts;
    }
}
}

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Certified consumer products and their previous-step contact view.</summary>
internal struct ContactProductFrameResources
{
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> SoftAvoidancePairs;

    public BroadPhaseCandidateBatch BroadPhaseCandidates =>
        new BroadPhaseCandidateBatch(TimestepInteractionPairs);

    public NarrowPhaseConstraintBatch NarrowPhaseConstraints =>
        new NarrowPhaseConstraintBatch(
            SoftAvoidancePairs,
            TimestepContactPairs);

    public static ContactProductFrameResources Create(int bodyCount)
    {
        return new ContactProductFrameResources
        {
            TimestepContactPairs = new NativeList<ContactConstraint>(
                math.max(bodyCount * 4, 1), Allocator.TempJob),
            PreviousTimestepContactPairs =
                new NativeList<ContactConstraint>(
                    math.max(bodyCount * 8, 1), Allocator.TempJob),
            TimestepInteractionPairs = new NativeList<BodyPair>(
                math.max(bodyCount * 8, 1), Allocator.TempJob),
            SoftAvoidancePairs = new NativeList<BodyPair>(
                math.max(bodyCount * 4, 1), Allocator.TempJob)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = TimestepContactPairs.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined,
            PreviousTimestepContactPairs.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined,
            TimestepInteractionPairs.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined,
            SoftAvoidancePairs.Dispose(finalReader));
        return combined;
    }
}
}
