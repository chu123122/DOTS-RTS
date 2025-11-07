using QFramework;
using UnityEngine;

namespace UI.EcoUI
{
    public class EcoUIModel : AbstractModel
    {
        private int _mineralSum; // 矿物总量
        private int _gasSum; // 气总量
        private int _populationSum; // 人口总量
        private int _populationMax; // 人口上限

        #region Get Set

        public int MineralSum
        {
            get => _mineralSum;
            set
            {
                if(value < 0)
                    Debug.LogError("不正当操作，矿消耗超过底线，请检查消耗前是否判负");
                else 
                    _mineralSum = value;
            }
        }
        public int GasSum
        {
            get => _gasSum;
            set
            {
                if(value < 0)
                    Debug.LogError("不正当操作，气消耗超过底线，请检查消耗前是否判负");
                else 
                    _gasSum = value;
            }
        }
        public int PopulationSum
        {
            get => _populationSum;
            set
            {
                if(value < 0)
                    Debug.LogError("不正当操作，人口删除过多，请检查事件调用次数或者静态变量是否调用错误");
                else
                {
                    if(value > _populationMax)
                        Debug.LogWarning("人口超过上限，请检查该次超过上限是否合规");
                    _populationSum = value;
                }
            }
        }
        public int PopulationMax
        {
            get => _populationMax;
            set
            {
                if(value < 0)
                    Debug.LogWarning("不正当操作，人口上限低过底线，请检查事件调用次数或者静态变量是否调用错误");
                else 
                    _populationMax = value;
            }
        }

        #endregion

        /// <summary>
        /// 检测兵种，建筑是否能创造
        /// </summary>
        /// <param name="ecoCost"> 传入要检测的信息 </param>
        /// <returns></returns>
        public bool IfCanCreate(EcoCost ecoCost)
        {
            if (_populationSum + ecoCost.Population > _populationMax)
            {
                return false;
            }
            else if (ecoCost.Gas > _gasSum)
            {
                return false;
            }
            else if (ecoCost.Mineral > _mineralSum)
            {
                return false;
            }

            return true;
        }
        
        protected override void OnInit()
        {
            _mineralSum = 100;
            _gasSum = 100;
            _populationSum = 0;

            _populationMax = 10;
        }
    }    
}

