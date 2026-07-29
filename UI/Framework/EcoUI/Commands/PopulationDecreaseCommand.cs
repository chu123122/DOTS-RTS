using QFramework;
using UnityEngine;

namespace UI.EcoUI.Commands
{
    public class PopulationDecreaseCommand : AbstractCommand
    {
        private readonly int _value;
            
        public PopulationDecreaseCommand(int value)
        {
            _value = value;
        }
        
        protected override void OnExecute()
        {
            this.GetModel<EcoUIModel>().PopulationSum -= _value;
        }
    }    
}

