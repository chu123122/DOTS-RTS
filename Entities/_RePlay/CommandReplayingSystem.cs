using _RePlaySystem.Base;
using DefaultNamespace;
using Entities._Common;
using QFramework;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using Utils;

namespace _RePlaySystem
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    public partial class CommandReplayingSystem : SystemBase, IServiceSystemLocator, ICanRegisterEvent
    {
        private NetworkTick _currentReplayTick;

        private World _initialWorld; // 初始世界的快照

        private bool _isReplaying = false;
        private int _replayCommandCount = 0;
        private EntityQuery _startSceneQuery;

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }

        protected override void OnCreate()
        {
            _currentReplayTick = new NetworkTick(0);
        }

        protected override void OnStartRunning()
        {
            _startSceneQuery = EntityManager.CreateEntityQuery(typeof(LocalTransform));
            CaptureInitialState();
        }

        private RtsReplayComponent _replayComponent;
        protected override void OnUpdate()
        {
            if (Input.GetKeyDown(KeyCode.R)) StartReplay();
            if (!_isReplaying) return;
            int tickValueAsInt = int.Parse(_currentReplayTick.TickValue.ToString());
            tickValueAsInt += 1;
            _currentReplayTick = new NetworkTick((uint)tickValueAsInt);
            

            if (HaveCommandAtTick(_replayComponent, _currentReplayTick, out var currentCommand))
            {
                RequestCommandRpcSystem commandRpcSystem = this.GetService<RequestCommandRpcSystem>();
                commandRpcSystem.ExecutedInputCommandInLocal(currentCommand);
                _replayCommandCount++;
                Debug.Log($"执行指令：{currentCommand.ToString()}");
            }


            if (_replayCommandCount == _replayComponent.CommandHistory.Length)
            {
                _isReplaying = false;
                _replayComponent.CommandHistory.Dispose();
                Debug.Log("Replay finished.");
            }
        }

        private bool HaveCommandAtTick(RtsReplayComponent replayComponent,
            NetworkTick tick, out PlayerInputCommand currentCommand)
        {
            foreach (var command in replayComponent.CommandHistory)
            {
                if (command.Tick == tick)
                {
                    currentCommand = command;
                    return true;
                }
            }

            currentCommand = default;
            return false;
        }

        private void StartReplay()
        {
            _replayCommandCount = 0;
            _currentReplayTick = new NetworkTick(0);
            
            var replayQuery = EntityManager.CreateEntityQuery(typeof(NetWorkDataContainer));
            var replayDataComponent = replayQuery.GetSingleton<NetWorkDataContainer>().ReplayDataComponent;
            _replayComponent = replayDataComponent.ToReplayComponent();
           
            DisconnectServer();
            RestoreInitialState();
            _isReplaying = true;
        }

        private void CaptureInitialState()
        {
            if (_initialWorld != null && _initialWorld.IsCreated) _initialWorld.Dispose();

            _initialWorld = new World("InitialStateSnapshot");

            var entities = _startSceneQuery.ToEntityArray(Allocator.Temp);
            if (entities.Length > 0)
            {
                // 克隆实体到新世界
                _initialWorld.EntityManager.CopyEntitiesFrom(EntityManager, entities);
            }
        }

        private void DisconnectServer()
        {
            //获取网络驱动
            using var networkDriverQuery =
                EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamDriver>());
             var driver = networkDriverQuery.GetSingletonRW<NetworkStreamDriver>();
            //获取网络连接
            using var connectionQuery =
                EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamConnection>());
            var connections = connectionQuery.ToComponentDataArray<NetworkStreamConnection>(Allocator.Temp);
            //断开所有连接
            foreach (var connection in connections)
            {
                driver.ValueRW.DriverStore.Disconnect(connection);
            }
        }

        private void RestoreInitialState()
        {
            //重置场景
            var serverQuery = EntityManager.CreateEntityQuery(typeof(LocalTransform));
            var currentEntities = serverQuery.ToEntityArray(Allocator.Temp);
            if (currentEntities.Length > 0)
            {
                EntityManager.DestroyEntity(currentEntities);
            }

            //TODO:待后续进一步修改
            var snapshotQuery = _initialWorld.EntityManager.CreateEntityQuery(typeof(LocalTransform));
            var snapshotEntities = snapshotQuery.ToEntityArray(Allocator.Temp);
            if (snapshotEntities.Length > 0)
            {
                EntityManager.CopyEntitiesFrom(_initialWorld.EntityManager, snapshotEntities);
            }
        }

        protected override void OnDestroy()
        {
            if (_initialWorld != null && _initialWorld.IsCreated)
            {
                _initialWorld.Dispose();
                _initialWorld = null;
            }
        }
    }
}