using UI.EcoUI;
using UnityEngine;

namespace UI.EcoUI.Events
{
    public struct EntityCreateEvent
    {
        public EcoCost EcoCost;
    }
    
    public struct PopulationDecreaseEvent
    {
        public EcoCost EcoCost;
    }

    public struct GasAddEvent
    {
        public EcoCollect EcoCollect;
    }

    public struct MineralAddEvent
    {
        public EcoCollect EcoCollect;
    }

    public struct PopulationMaxAddEvent
    {
        public int Value;
    }
}
