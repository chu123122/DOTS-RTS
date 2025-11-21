using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Entities.Unit.System.FlowFieldSystem
{
    [BurstCompile]
    public struct GenerateVectorFieldJob : IJobParallelFor
    {
        public int2 GridDimensions;
        [NativeDisableParallelForRestriction]
        public NativeArray<FlowFieldCell> Grid;

        public void Execute(int index)
        {
            int2 currentPos = FlowFieldUtils.GetCellPosFromIndex(index, GridDimensions);
            FlowFieldCell currentCell = Grid[index];

            if (currentCell.Cost == 0 || currentCell.BestDirectionIndex == 0)
            {
                currentCell.BestDirectionIndex = 0xFF;
                Grid[index] = currentCell;
                return;
            }

            int bestDirIndex = 0xFF;
            ushort minCost = currentCell.IntegrationValue;

            for (int i = 0; i < 8; i++)
            {
                int2 offset = GetDirectionOffset(i);
                int2 neighborPos = currentPos + offset;

                if (neighborPos.x >= 0 && neighborPos.x < GridDimensions.x &&
                    neighborPos.y >= 0 && neighborPos.y < GridDimensions.y)
                {
                    int neighborIndex = FlowFieldUtils.GetFlatIndex(neighborPos, GridDimensions);
                    ushort neighborCost = Grid[neighborIndex].IntegrationValue;

                    if (neighborCost < minCost)
                    {
                        minCost = neighborCost;
                        bestDirIndex = i;
                    }
                }

                currentCell.BestDirectionIndex = (byte)bestDirIndex;
                Grid[index] = currentCell;
            }
        }

        private int2 GetDirectionOffset(int index)
        {
            // 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
            switch (index)
            {
                case 0: return new int2(0, 1);
                case 1: return new int2(1, 1);
                case 2: return new int2(1, 0);
                case 3: return new int2(1, -1);
                case 4: return new int2(0, -1);
                case 5: return new int2(-1, -1);
                case 6: return new int2(-1, 0);
                case 7: return new int2(-1, 1);
                default: return int2.zero;
            }
        }
    }

    [BurstCompile]
    public struct ResetGridJob : IJobParallelFor
    {
        public NativeArray<FlowFieldCell> Grid;

        public void Execute(int index)
        {
            var cell = Grid[index];

            cell.IntegrationValue = ushort.MaxValue;
            cell.BestDirectionIndex = 0xFF;
            cell.Cost = 1; //TODO:没有考虑障碍物情况

            Grid[index] = cell;
        }
    }

    // [BurstCompile]
    public struct GenerateIntegrationFieldJob : IJob
    {
        [ReadOnly] public int2 GridDimensions;
        public NativeArray<FlowFieldCell> Grid;
        public NativeQueue<int2> Queue; // 队列里存的是坐标(x,y)

        [ReadOnly] public int2 TargetCell;

        public void Execute()
        {
            // 1. 统一使用 Utils 计算 Index
            int targetIndex = FlowFieldUtils.GetFlatIndex(TargetCell, GridDimensions);

            // 安全检查
            if (targetIndex < 0 || targetIndex >= Grid.Length) return;

            if (Grid[targetIndex].Cost == 0)
                return;

            // 设置目标点
            var targetCellData = Grid[targetIndex];
            targetCellData.IntegrationValue = 0;
            Grid[targetIndex] = targetCellData;

            // 【修复】入队的是 坐标(int2)，不是 Index(int)
            Queue.Enqueue(TargetCell);

            // 定义 8 个方向
            NativeArray<int2> neighbors = new NativeArray<int2>(8, Allocator.Temp);
            neighbors[0] = new int2(0, 1);
            neighbors[1] = new int2(1, 1);
            neighbors[2] = new int2(1, 0);
            neighbors[3] = new int2(1, -1);
            neighbors[4] = new int2(0, -1);
            neighbors[5] = new int2(-1, -1);
            neighbors[6] = new int2(-1, 0);
            neighbors[7] = new int2(-1, 1);

            while (Queue.TryDequeue(out int2 currentCellPos))
            {
                // 获取当前格子的 Cost，用于累加
                int currentIndex = FlowFieldUtils.GetFlatIndex(currentCellPos, GridDimensions);
                ushort currentIntegrationCost = Grid[currentIndex].IntegrationValue;

                for (int index = 0; index < neighbors.Length; index++)
                {
                    int2 neighborPos = currentCellPos + neighbors[index];

                    // 边界检查
                    if (neighborPos.x < 0 || neighborPos.x >= GridDimensions.x ||
                        neighborPos.y < 0 || neighborPos.y >= GridDimensions.y)
                        continue;

                    // 计算邻居 Index
                    int neighborIndex = FlowFieldUtils.GetFlatIndex(neighborPos, GridDimensions);
                    var neighborCellData = Grid[neighborIndex];

                    // 核心 BFS 逻辑：
                    // 1. 不是障碍物 (Cost != 0)
                    // 2. 还没被访问过 (IntegrationValue == Max)
                    if (neighborCellData.Cost != 0 && neighborCellData.IntegrationValue == ushort.MaxValue)
                    {
                        // 累加代价 (这里简单 +1，也可以 +Cost)
                        neighborCellData.IntegrationValue = (ushort)(currentIntegrationCost + 1);
                        Grid[neighborIndex] = neighborCellData; // 写回数组

                        // 【修复】入队邻居的 坐标(int2)
                        Queue.Enqueue(neighborPos);
                    }
                }
            }

            neighbors.Dispose();
        }
    }
}