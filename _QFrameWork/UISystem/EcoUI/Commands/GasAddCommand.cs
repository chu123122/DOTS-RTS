using QFramework;
using UnityEngine;

namespace UI.EcoUI.Commands
{
    public class GasAddCommand : AbstractCommand
    {
        private readonly int _value;
            
        public GasAddCommand(int value)
        {
            _value = value;
        }
        
        protected override void OnExecute()
        {
            this.GetModel<EcoUIModel>().GasSum += _value;
        }
    }    
}

