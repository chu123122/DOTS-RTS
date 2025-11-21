
using DefaultNamespace;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;


namespace _RePlaySystem.Base
{
    public enum InputCommandType
    {
        Move,
        Create,
    }

   
    public struct PlayerInputCommandData:IServiceSystemLocator
    {
        public InputCommandType Type;
        public int PlayerNetWorkId;
        public float3 Position;
        public FixedString128Bytes Units;
    }
    
}