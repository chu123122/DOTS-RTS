using QFramework;
using UnityEngine;

namespace UI.EcoUI.Commands
{
    public class EntityCreateCommand : AbstractCommand
    {
        private readonly EcoCost _ecoCost;
        
        public EntityCreateCommand(EcoCost ecoCost)
        {
            _ecoCost = ecoCost;
        }
        
        protected override void OnExecute()
        {
            EcoUIModel ecoUIModel = this.GetModel<EcoUIModel>();

            ecoUIModel.MineralSum -= _ecoCost.Mineral;
            ecoUIModel.GasSum -= _ecoCost.Gas;
            ecoUIModel.PopulationSum += _ecoCost.Population;
        }
    }    
}

