using UnityEngine;

namespace UI.MapUI
{
    public static class MapUISetting
    {
        public static Color PlayerColor = new Color(0.02f, 0.83f, 0); // 玩家操控的单位在小地图上的颜色
        public static Color EnemyColor = new Color(0.73f, 0.1f, 0.18f); // 敌方操控的单位在小地图上的颜色

        public static Vector2 BuildingSize = new Vector2(20, 20); // 建筑在小地图上的大小
        public static Vector2 HeroSize = new Vector2(10, 10); // 英雄在小地图上的大小
        public static Vector2 SoliderSize = new Vector2(5, 5); // 士兵在小地图上的大小
    }
    
    // 小地图实体相关类型
    public enum MapUIEntityType
    {
        Building, Hero, Solider // 建筑，英雄，普通兵种
    }
}
