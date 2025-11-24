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
                DisconnectServer();

                state.ValueRW.IsPlaying = true;
                state.ValueRW.IsRecording = false;
                state.ValueRW.PlaybackIndex = 0;
                state.ValueRW.ReplayStartTime = SystemAPI.Time.ElapsedTime;

                if (!EntityManager.HasComponent<LocalInstance>(stateEntity))
                    EntityManager.AddComponentData(stateEntity, new LocalInstance());

                // 1. 清场单位
                var unitQuery = EntityManager.CreateEntityQuery(typeof(BasicUnitTag));
                var entities = unitQuery.ToEntityArray(Allocator.Temp);
                EntityManager.DestroyEntity(entities);
                entities.Dispose();

                // 2. 【核心修复】彻底重置流场
                if (SystemAPI.TryGetSingletonRW<FlowFieldGrid>(out var grid))
                {
                    var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGrid>();

                    // A. 移除可能存在的烘焙 Tag，防止系统在后台偷偷 Bake 旧数据
                    if (SystemAPI.HasComponent<RecalculateFlowFieldTag>(gridEntity))
                    {
                        EntityManager.RemoveComponent<RecalculateFlowFieldTag>(gridEntity);
                    }

                    // B. 强制将目标归零
                    if (SystemAPI.TryGetSingletonEntity<FlowFieldGlobalTarget>(out var targetEntity))
                    {
                        SystemAPI.SetComponent(targetEntity,
                            new FlowFieldGlobalTarget { TargetPosition = float3.zero });
                    }

                    // C. 暴力刷写内存，让网格变成 "无效/待命"
                    // 使用 Run (主线程立即执行) 确保在任何单位生成前，网格是干净的
                    new ClearGridJob
                    {
                        Grid = grid.ValueRW.Grid
                    }.Run(grid.ValueRW.Grid.Length);

                    Debug.Log(">>> 流场已暴力重置");
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
                using var connectionQuery =
                    EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamConnection>());
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

            var ecb = new EntityCommandBuffer(Allocator.Temp);

            int idx = state.ValueRO.PlaybackIndex;
            if (idx >= buffer.Length)
            {
                Debug.Log("回放结束");
                state.ValueRW.IsPlaying = false;
                ecb.Dispose(); // 别忘了释放
                return;
            }

            double currentReplayTime = SystemAPI.Time.ElapsedTime - state.ValueRO.ReplayStartTime;

            // 2. 循环处理指令
            while (idx < buffer.Length)
            {
                var cmd = buffer[idx];
                if (currentReplayTime < cmd.TimeOffset) break;

                ExecuteReplayCommand(cmd, ref ecb);
                idx++;
            }

            // 3. 先更新索引 (状态变更)
            state.ValueRW.PlaybackIndex = idx;

            // 4. 最后统一执行所有结构化改变 (生成单位、加组件等)
            // 这时候 Buffer 即使失效了也没关系，因为我们已经读完了
            ecb.Playback(EntityManager);
            ecb.Dispose();
        }

        // 注意参数变化：传入 ref EntityCommandBuffer ecb
        private void ExecuteReplayCommand(ReplayCommandElement cmd, ref EntityCommandBuffer ecb)
        {
            if (cmd.Type == RTSCommandType.Spawn)
            {
                var prefabEntity = SystemAPI.GetSingleton<RtsLocalPrefabs>().Entity;

                // 使用 ecb.Instantiate
                var newUnit = ecb.Instantiate(prefabEntity);

                var transform = LocalTransform.FromPosition(cmd.Position);
                transform.Scale = 0.5f;
                // 使用 ecb.SetComponent
                ecb.SetComponent(newUnit, transform);

                // 使用 ecb.AddComponent
                ecb.AddComponent(newUnit, new LocalInstance() { Id = 0 });
                
                ecb.AddComponent(newUnit, new BasicUnitTag());
            }
            else if (cmd.Type == RTSCommandType.Move)
            {
                if (SystemAPI.TryGetSingletonEntity<FlowFieldGlobalTarget>(out var gridEntity))
                {
                    ecb.SetComponent(gridEntity, new FlowFieldGlobalTarget { TargetPosition = cmd.Position });
                    ecb.AddComponent<RecalculateFlowFieldTag>(gridEntity);
                }
            }

            Debug.Log($"[Replay] Command Queued: {cmd.Type}");
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
}
