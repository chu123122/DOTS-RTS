using QFramework;
using UnityEngine;

namespace UI.EcoUI.Commands
{
    public class PopulationMaxAddCommand : AbstractCommand
    {
        private readonly int _value;
            
        public PopulationMaxAddCommand(int value)
        {
            _value = value;
        }
        
        protected override void OnExecute()
        {
            this.GetModel<EcoUIModel>().PopulationMax += _value;
        }
    }    
}

