using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.Serialization;

namespace 通用
{
    public struct BasicUnitTag : IComponentData
    {
    }

    public struct NewBasicUnitTag : IComponentData
    {
    }

    public struct RtsTeam : IComponentData
    {
        [GhostField] public TeamType Value;
    }

    public struct UnitMoveSpeed : IComponentData
    {
        public float Value;
    }

    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct UnitMoveTargetPosition : IInputComponentData
    {
        [GhostField(Quantization = 0)]public float3 Value;
        [GhostField(Quantization = 0)] public bool GetMoveInput;
    }
    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct UnitSelected : IInputComponentData
    {
        [GhostField]public bool Value;
    }

   
    
   
  
    public struct IsUserUnitTag:IComponentData,IEnableableComponent
    {}

   
}