using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;

namespace _RePlaySystem.Base
{
    // 指令类型：移动 或 生成
    public enum RTSCommandType : byte
    {
        Move = 0,
        Spawn = 1
    }

    // 【核心】替代原来的 PlayerInputCommand (JSON)
    // 这是一个 BufferElement，DOTS 会自动高效管理内存
    [InternalBufferCapacity(128)]
    public struct ReplayCommandElement : IBufferElementData
    {
        public double TimeOffset;     // 距离开始录制过了多久 (相对时间)
        public RTSCommandType Type;   // 指令类型
        public float3 Position;       // 目标点 或 出生点
        public int UnitCount;         // (可选) 如果是生成指令，生成多少个
    }

    // 单例组件：控制录制/回放状态
    public struct ReplaySystemState : IComponentData
    {
        public bool IsRecording;
        public bool IsPlaying;
        
        public double RecordingStartTime; // 录制开始时的系统时间
        public double ReplayStartTime;    // 回放开始时的系统时间
        public int PlaybackIndex;         // 当前播放到第几条指令
    }
}