using System;
using DefaultNamespace;
using QFramework;
using TMPro;
using UI.EcoUI.Commands;
using UI.EcoUI.Events;
using UnityEngine;

namespace UI.EcoUI
{
    public class EcoUIController : MonoBehaviour, IController, ICanSendEvent
    {
        public TextMeshProUGUI mineralTxt; // 矿物相关文本
        public TextMeshProUGUI gasTxt; // 气相关文本
        public TextMeshProUGUI populationTxt; // 人口相关文本
        
        private EcoUIModel _ecoUIModel; // 经济相关数据

        private void Start()
        {
            _ecoUIModel = this.GetModel<EcoUIModel>();
            
            UpdateView();
            RegisterEvents();
        }

        /// <summary>
        /// 更新UI
        /// </summary>
        private void UpdateView()
        {
            mineralTxt.text = _ecoUIModel.MineralSum.ToString();
            gasTxt.text = _ecoUIModel.GasSum.ToString();
            populationTxt.text = _ecoUIModel.PopulationSum + "/" + _ecoUIModel.PopulationMax;
        }
        
        #region 接收事件函数

        private void OnEntityCreateEvent(EcoCost ecoCost)
        {
            this.SendCommand(new EntityCreateCommand(ecoCost));
            
            UpdateView();
        }

        private void OnPopulationDecreaseEvent(EcoCost ecoCost)
        {
            this.SendCommand(new PopulationDecreaseCommand(ecoCost.Population));
            
            UpdateView();
        }

        private void OnGasAddEvent(EcoCollect ecoCollect)
        {
            this.SendCommand(new GasAddCommand(ecoCollect.Gas));
            
            UpdateView();
        }

        private void OnMineralAddEvent(EcoCollect ecoCollect)
        {
            this.SendCommand(new MineralAddCommand(ecoCollect.Mineral));
            
            UpdateView();
        }

        private void OnPopulationMaxAddEvent(int value)
        {
            this.SendCommand(new PopulationMaxAddCommand(value));
            
            UpdateView();
        }

        #endregion
        
        /// <summary>
        /// 注册事件
        /// </summary>
        private void RegisterEvents()
        {
            this.RegisterEvent<EntityCreateEvent>(e =>
            {
                OnEntityCreateEvent(e.EcoCost);
            }).UnRegisterWhenDisabled(this);

            this.RegisterEvent<PopulationDecreaseEvent>(e =>
            {
                OnPopulationDecreaseEvent(e.EcoCost);
            }).UnRegisterWhenDisabled(this);

            this.RegisterEvent<MineralAddEvent>(e =>
            {
                OnMineralAddEvent(e.EcoCollect);
            }).UnRegisterWhenDisabled(this);

            this.RegisterEvent<GasAddEvent>(e =>
            {
                OnGasAddEvent(e.EcoCollect);
            }).UnRegisterWhenDisabled(this);

            this.RegisterEvent<PopulationMaxAddEvent>(e =>
            {
                OnPopulationMaxAddEvent(e.Value);
            }).UnRegisterWhenDisabled(this);
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }    
}

