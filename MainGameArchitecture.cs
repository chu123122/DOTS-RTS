using System.Runtime.InteropServices;
using QFramework.BuildingManagement.Utils;
using QFramework.Systems.BuildingSystem.Utility;
using UI.EcoUI;
using UI.MapUI;

namespace DefaultNamespace
{
    using QFramework;
    public class MainGameArchitecture:Architecture<MainGameArchitecture>
    {
        protected override void Init()
        {
            this.RegisterUtility<IBuildingUtility>(new BuildingUtility());
            
            this.RegisterModel(new MapUIModel());
            
            this.RegisterModel(new EcoUIModel());
        }
    }
}