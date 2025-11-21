using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.NetCode;
using Unity.Jobs; // 引用 Job
using UnityEngine;
using 通用; 
using DefaultNamespace;
using Entities.Unit.System.FlowFieldSystem;
using Unity.Burst;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    public partial class CommandReplayingSystem : SystemBase
    {
        protected override void OnCreate()
        {
            RequireForUpdate<ReplaySystemState>();
            RequireForUpdate<RtsLocalPrefabs>();
        }

        protected override void OnUpdate()
        {
            if (Input.GetKeyDown(KeyCode.L))
            {
                if (SystemAPI.TryGetSingletonRW<ReplaySystemState>(out var state_a))
                {
                    state_a.ValueRW.IsRecording = true;
                    state_a.ValueRW.IsPlaying = false;
                    state_a.ValueRW.RecordingStartTime = SystemAPI.Time.ElapsedTime;
                    
                    var stateEntity = SystemAPI.GetSingletonEntity<ReplaySystemState>();
                    SystemAPI.GetBuffer<ReplayCommandElement>(stateEntity).Clear();
                    
                    Debug.Log(">>> 开始录制 (Start Recording)");
                }
            }

            if (Input.GetKeyDown(KeyCode.R))
            {
                ToggleReplayState();
            }

            if (SystemAPI.TryGetSingletonRW<ReplaySystemState>(out var state))
            {
                if (state.ValueRO.IsPlaying)
                {
                    ProcessReplayFrame(state);
                }
            }
        }

        private void ToggleReplayState()
        {
            if (!SystemAPI.TryGetSingletonRW<ReplaySystemState>(out var state)) return;

            bool isPlaying = state.ValueRO.IsPlaying;
            var stateEntity = SystemAPI.GetSingletonEntity<ReplaySystemState>();

            if (!isPlaying)
            {
                Debug.Log(">>> 开始回放 (Start Replay)");

                // 1. 断开服务器
                DisconnectServer();

                // 2. 状态设置
                state.ValueRW.IsPlaying = true;
                state.ValueRW.IsRecording = false;
                state.ValueRW.PlaybackIndex = 0;
                state.ValueRW.ReplayStartTime = SystemAPI.Time.ElapsedTime;
                
                // 3. 激活本地系统
                if (!EntityManager.HasComponent<LocalInstance>(stateEntity))
                {
                    EntityManager.AddComponentData(stateEntity, new LocalInstance());
                }

                // 4. 清场
                var unitQuery = EntityManager.CreateEntityQuery(typeof(BasicUnitTag));
                var entities = unitQuery.ToEntityArray(Allocator.Temp);
                EntityManager.DestroyEntity(entities);
                entities.Dispose();

                // 5. 【关键修复】暴力清空 FlowField 网格
                // 我们不等 BakeSystem 了，直接在这里把网格抹平，让单位"瞎"掉，原地不动
                if (SystemAPI.TryGetSingletonRW<FlowFieldGrid>(out var grid))
                {
                    // 将 Target 重置为 0 (虽然此时网格无效，但数据要对齐)
                    if (SystemAPI.TryGetSingletonEntity<FlowFieldGlobalTarget>(out var targetEntity))
                    {
                         SystemAPI.SetComponent(targetEntity, new FlowFieldGlobalTarget { TargetPosition = float3.zero });
                    }

                    // 立即运行一个 Job 来填充网格为 "无效 (0xFF)"
                    // 这样新生成的单位读取网格时，BestDirectionIndex 就是 255，它们会原地待命
                    new ClearGridJob 
                    { 
                        Grid = grid.ValueRW.Grid 
                    }.Run(grid.ValueRW.Grid.Length); // .Run() 是在主线程立即执行，不等待
                }
            }
            else
            {
                Debug.Log("<<< 停止回放");
                state.ValueRW.IsPlaying = false;
                EntityManager.RemoveComponent<LocalInstance>(stateEntity);
            }
        }

        private void DisconnectServer()
        {
            if (SystemAPI.TryGetSingletonRW<NetworkStreamDriver>(out var driver))
            {
                using var connectionQuery = EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamConnection>());
                var connections = connectionQuery.ToComponentDataArray<NetworkStreamConnection>(Allocator.Temp);
                foreach (var connection in connections)
                {
                    driver.ValueRW.DriverStore.Disconnect(connection);
                }
                connections.Dispose();
            }
        }

        private void ProcessReplayFrame(RefRW<ReplaySystemState> state)
        {
            var stateEntity = SystemAPI.GetSingletonEntity<ReplaySystemState>();
            var buffer = SystemAPI.GetBuffer<ReplayCommandElement>(stateEntity);

            int idx = state.ValueRO.PlaybackIndex;
            if (idx >= buffer.Length)
            {
                Debug.Log("回放结束");
                state.ValueRW.IsPlaying = false;
                return;
            }

            double currentReplayTime = SystemAPI.Time.ElapsedTime - state.ValueRO.ReplayStartTime;
            
            while (idx < buffer.Length)
            {
                var cmd = buffer[idx];
                if (currentReplayTime < cmd.TimeOffset) break;
                ExecuteReplayCommand(cmd);
                idx++;
            }
            state.ValueRW.PlaybackIndex = idx;
        }

        private void ExecuteReplayCommand(ReplayCommandElement cmd)
        {
            if (cmd.Type == RTSCommandType.Spawn) 
            {
                var prefabEntity = SystemAPI.GetSingleton<RtsLocalPrefabs>().Entity;
                var newUnit = EntityManager.Instantiate(prefabEntity);
                var transform = LocalTransform.FromPosition(cmd.Position);
                transform.Scale = 0.5f;
                EntityManager.SetComponentData(newUnit, transform);
                
                // 此时单位会读取 FlowField。
                // 因为我们在 Toggle 时清空了网格，它会待命。
                // 直到读到下一条 Move 指令触发 Bake，它才会动。
                
                if(!EntityManager.HasComponent<BasicUnitTag>(newUnit))
                    EntityManager.AddComponent<BasicUnitTag>(newUnit);
            }
            else if (cmd.Type == RTSCommandType.Move)
            {
                if (SystemAPI.TryGetSingletonEntity<FlowFieldGlobalTarget>(out var gridEntity))
                {
                    SystemAPI.SetComponent(gridEntity, new FlowFieldGlobalTarget { TargetPosition = cmd.Position });

                    if (!SystemAPI.HasComponent<RecalculateFlowFieldTag>(gridEntity))
                        EntityManager.AddComponent<RecalculateFlowFieldTag>(gridEntity);
                    
                    Debug.Log($"[Replay] Move to: {cmd.Position}");
                }
            }
        }
    }

    // 【新增】用于暴力清空网格的简单 Job
    [BurstCompile]
    public struct ClearGridJob : IJobParallelFor
    {
        public NativeArray<FlowFieldCell> Grid;

        public void Execute(int index)
        {
            var cell = Grid[index];
            cell.IntegrationValue = ushort.MaxValue;
            cell.BestDirectionIndex = 0xFF; // 255 = 无效/待命
            // 注意：不要清空 Cost (障碍物数据)，否则回放时会穿墙
            // 除非你希望重置一切。通常 Cost 是静态的，保留即可。
            Grid[index] = cell;
        }
    }
}