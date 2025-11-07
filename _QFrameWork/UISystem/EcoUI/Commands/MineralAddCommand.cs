using QFramework;
using UnityEngine;

namespace UI.EcoUI.Commands
{
    public class MineralAddCommand : AbstractCommand
    {
        private readonly int _value;
                
        public MineralAddCommand(int value)
        {
            _value = value;
        }
            
        protected override void OnExecute()
        {
            this.GetModel<EcoUIModel>().MineralSum += _value;
        }
    }    
}

