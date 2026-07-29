using DefaultNamespace;
using QFramework;
using UI.EcoUI;
using UI.EcoUI.Events;
using UnityEngine;

public class EcoUITest : MonoBehaviour, ICanSendEvent
{
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < 100; i++)
            {
                this.SendEvent(new PopulationDecreaseEvent()
                {
                    EcoCost = EcoUICostSetting.SimpleSolider
                });                
            }
        }
    }

    public IArchitecture GetArchitecture()
    {
        return MainGameArchitecture.Interface;
    }
}
