using Unity.Entities;

namespace _RePlaySystem.Base
{
    public struct LocalInstance:IComponentData
    {
        public int Id;
    }

    /// <summary>
    /// Local World 中的稳定运行时编号源。只用于本次游戏会话内的实体/UI 关联。
    /// </summary>
    public struct LocalInstanceIdSequence : IComponentData
    {
        public int NextId;
    }
}
