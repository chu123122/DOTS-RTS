using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Shared frame-local utilities used by the authoritative contact pipeline.
/// These helpers have no ownership of persistent topology or legacy Fat AABB state.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void PrepareCurrentBodyLookup()
    {
        CurrentBodyIndexByEntity.Clear();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
  CurrentBodyIndexByEntity.TryAdd(States[bodyIndex].Entity, bodyIndex);
    }

    private bool TryFindCurrentBodyIndex(Entity entity, out int bodyIndex)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
     bodyIndex >= 0 && bodyIndex < States.Length;
    }

    private void CalculateNeighborPathBounds(
        FlowMovementFrameState state,
        out float2 pathMin,
        out float2 pathMax)
    {
        pathMin = math.min(
  state.TimestepStartPosition.xz,
  math.min(
      state.TimestepPredictedPosition.xz,
      math.min(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        pathMax = math.max(
  state.TimestepStartPosition.xz,
  math.max(
      state.TimestepPredictedPosition.xz,
      math.max(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        if (SoftAvoidanceVelocitySolver !=
      SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
  SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
  return;

        float2 horizonEnd = state.PredictedPosition.xz +
                  state.BasePredictedVelocity.xz * math.max(0f, RvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }

    private void ResetCorrectedBodyTracking()
    {
        for (int i = 0; i < CorrectedBodyIndices.Length; i++)
  CorrectedBodyFlags[CorrectedBodyIndices[i]] = 0;
        CorrectedBodyIndices.Clear();
    }

    private void MarkCorrectedBody(int bodyIndex)
    {
        if (CorrectedBodyFlags[bodyIndex] != 0)
  return;
        CorrectedBodyFlags[bodyIndex] = 1;
        CorrectedBodyIndices.Add(bodyIndex);
    }

    private static void SortAndDeduplicateBodyPairs(NativeList<UnitCollisionPair> pairs)
    {
        if (pairs.Length <= 1)
  return;
        pairs.AsArray().Sort(new UnitCollisionPairComparer());
        int writeIndex = 1;
        UnitCollisionPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
  UnitCollisionPair current = pairs[readIndex];
  if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
      continue;
  pairs[writeIndex++] = current;
  previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private static bool TryFindProxy(
        NativeList<ShadowFatBodyProxy> proxies,
        Entity entity,
        out ShadowFatBodyProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
  int middle = (low + high) >> 1;
  ShadowFatBodyProxy candidate = proxies[middle];
  int comparison = ShadowEntityOrdering.Compare(candidate.Entity, entity);
  if (comparison == 0)
  {
      proxy = candidate;
      return proxy.IsValid != 0;
  }
  if (comparison < 0)
      low = middle + 1;
  else
      high = middle - 1;
        }
        proxy = default;
        return false;
    }

    private static bool AabbContains(
        float2 outerMin,
        float2 outerMax,
        float2 innerMin,
        float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
     math.all(innerMax <= outerMax + tolerance);
    }

    private static bool AabbOverlaps(
        float2 minA,
        float2 maxA,
        float2 minB,
        float2 maxB)
    {
        return math.all(maxA >= minB) && math.all(maxB >= minA);
    }
}
}
