using System.Collections.Generic;
using Unity.Collections;

namespace RTS.Unit.FlowField.Jobs
{
internal struct ContactViewCandidate
{
    public ContactConstraint Contact;
    public byte IsValid;
    public byte IsPrevious;
    public byte PreviousWasDirty;
}

internal struct ContactViewPublicationBlock
{
    public int OutputCount;
    public int OutputOffset;
    public int FallbackCount;
}

internal struct ContactViewCandidateComparer :
    IComparer<ContactViewCandidate>
{
    public int Compare(ContactViewCandidate left, ContactViewCandidate right)
    {
        if (left.IsValid != right.IsValid)
            return right.IsValid.CompareTo(left.IsValid);

        int bodyA = left.Contact.BodyA.CompareTo(right.Contact.BodyA);
        if (bodyA != 0)
            return bodyA;
        int bodyB = left.Contact.BodyB.CompareTo(right.Contact.BodyB);
        if (bodyB != 0)
            return bodyB;

        return right.IsPrevious.CompareTo(left.IsPrevious);
    }
}

internal static class TimestepContactRepairViewKernel
{
    internal static bool IsGroupStart(
        NativeArray<ContactViewCandidate> candidates,
        int candidateIndex)
    {
        if (candidateIndex < 0 ||
            candidateIndex >= candidates.Length ||
            candidates[candidateIndex].IsValid == 0)
            return false;
        if (candidateIndex == 0)
            return true;

        ContactViewCandidate previous = candidates[candidateIndex - 1];
        ContactViewCandidate current = candidates[candidateIndex];
        return previous.IsValid == 0 ||
               previous.Contact.BodyA != current.Contact.BodyA ||
               previous.Contact.BodyB != current.Contact.BodyB;
    }

    internal static int FindGroupEnd(
        NativeArray<ContactViewCandidate> candidates,
        int groupStart)
    {
        ContactViewCandidate first = candidates[groupStart];
        int groupEnd = groupStart + 1;
        while (groupEnd < candidates.Length)
        {
            ContactViewCandidate candidate = candidates[groupEnd];
            if (candidate.IsValid == 0 ||
                candidate.Contact.BodyA != first.Contact.BodyA ||
                candidate.Contact.BodyB != first.Contact.BodyB)
                break;
            groupEnd++;
        }
        return groupEnd;
    }

    internal static bool TrySelectRepairContact(
        NativeArray<ContactViewCandidate> candidates,
        int groupStart,
        out ContactConstraint contact,
        out byte wasFallback)
    {
        int groupEnd = FindGroupEnd(candidates, groupStart);
        ContactViewCandidate previous = default;
        ContactViewCandidate current = default;
        bool hasPrevious = false;
        bool hasCurrent = false;
        for (int candidateIndex = groupStart;
             candidateIndex < groupEnd;
             candidateIndex++)
        {
            ContactViewCandidate candidate = candidates[candidateIndex];
            if (candidate.IsPrevious != 0)
            {
                if (!hasPrevious)
                {
                    previous = candidate;
                    hasPrevious = true;
                }
            }
            else if (!hasCurrent)
            {
                current = candidate;
                hasCurrent = true;
            }
        }

        if (hasCurrent)
        {
            contact = current.Contact;
            if (hasPrevious)
            {
                CopyTimestepRuntime(previous.Contact, ref contact);
                wasFallback = 0;
            }
            else
            {
                contact.WasAddedByFallback = 1;
                wasFallback = 1;
            }
            return true;
        }

        if (hasPrevious && previous.PreviousWasDirty == 0)
        {
            contact = previous.Contact;
            wasFallback = 0;
            return true;
        }

        contact = default;
        wasFallback = 0;
        return false;
    }

    internal static bool TrySelectActivationContact(
        NativeArray<ContactViewCandidate> candidates,
        int groupStart,
        out ContactConstraint contact)
    {
        int groupEnd = FindGroupEnd(candidates, groupStart);
        ContactViewCandidate current = default;
        bool hasCurrent = false;
        for (int candidateIndex = groupStart;
             candidateIndex < groupEnd;
             candidateIndex++)
        {
            ContactViewCandidate candidate = candidates[candidateIndex];
            if (candidate.IsPrevious != 0)
            {
                contact = candidate.Contact;
                return true;
            }
            if (!hasCurrent)
            {
                current = candidate;
                hasCurrent = true;
            }
        }

        contact = current.Contact;
        return hasCurrent;
    }

    internal static bool IsDirty(
        ContactConstraint contact,
        NativeArray<byte> dirtyFlagsByBody)
    {
        return IncrementalDirtyBodyStore.IsDirtyBodyIndex(
                   dirtyFlagsByBody,
                   contact.BodyA) ||
               IncrementalDirtyBodyStore.IsDirtyBodyIndex(
                   dirtyFlagsByBody,
                   contact.BodyB);
    }

    internal static void CopyTimestepRuntime(
        ContactConstraint previous,
        ref ContactConstraint current)
    {
        current.Runtime = previous.Runtime;
    }
}
}
