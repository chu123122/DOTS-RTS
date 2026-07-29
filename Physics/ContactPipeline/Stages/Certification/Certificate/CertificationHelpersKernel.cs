using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
    private void PrepareCurrentBodyLookup()
    {
        CurrentBodyIndexByEntity.Clear();
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
            CurrentBodyIndexByEntity.TryAdd(Bodies[bodyIndex].Entity, bodyIndex);
    }


    private bool TryFindCurrentBodyIndex(Entity entity, out int bodyIndex)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
               bodyIndex >= 0 && bodyIndex < Bodies.Length;
    }
}
}
