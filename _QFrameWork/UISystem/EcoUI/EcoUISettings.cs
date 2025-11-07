
using System;

namespace UI.EcoUI
{

    #region 数据

    /// <summary>
    /// 经济UI消耗相关数据
    /// </summary>
    public static class EcoUICostSetting
    {
        public static readonly EcoCost SimpleSolider = new EcoCost(50, 10, 1);
    }

    /// <summary>
    /// 经济UI收集相关数据
    /// </summary>
    public static class EcoUICollentSetting
    {
        public static readonly EcoCollect SimpleFarmer = new EcoCollect(12, 8);
    }

    /// <summary>
    /// 经济UI添加相关数据
    /// </summary>
    public static class EcoUIAddSetting
    {
        public static int SimpleHero = 20;
    }

    #endregion


    #region 数据结构

    /// <summary>
    /// 经济消耗相关数据结构
    /// </summary>
    public struct EcoCost
    {
        public readonly int Mineral; // 所需的矿
        public readonly int Gas; // 所需的气
        public readonly int Population; // 所占的人口

        public EcoCost(int mineral, int gas, int population)
        {
            Mineral = mineral;
            Gas = gas;
            Population = population;
        }
    }

    /// <summary>
    /// 经济采集相关数据结构
    /// </summary>
    public struct EcoCollect
    {
        public readonly int Mineral;
        public readonly int Gas;

        public EcoCollect(int mineral, int gas)
        {
            Mineral = mineral;
            Gas = gas;
        }
    }

    #endregion

}