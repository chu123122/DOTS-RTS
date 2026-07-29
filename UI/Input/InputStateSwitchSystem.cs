using a;
using DefaultNamespace.InputSystem._Events;
using QFramework;
using UnityEngine;
using 简单战斗.ServiceLocator;

namespace DefaultNamespace.InputSystem
{
    public class InputStateSwitchSystem:ServiceObject<InputStateSwitchSystem>,IController
    {
        public PlayerAction PlayerAction;

        protected override void Awake()
        {
            base.Awake();
            PlayerAction = new PlayerAction();
            this.RegisterEvent<EnterControlStateEvent>((e) =>
            {

            });
            this.RegisterEvent<EnterBuildingStateEvent>((e) =>
            {

            });
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}