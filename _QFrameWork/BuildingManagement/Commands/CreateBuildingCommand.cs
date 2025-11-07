using System;
using System.Collections;
using System.Collections.Generic;
using _QFrameWork._CommonUtils;
using _RePlaySystem.Base;
using DefaultNamespace;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using Entities.Building.Authoring;
using QFramework.BuildingManagement.Utils;
using Test.BuildingSystem;
using UI.MapUI;
using UI.MapUI.Events;
using Unity.Entities;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using 简单战斗;
using 简单战斗.ServiceLocator;
using 通用;
using Object = UnityEngine.Object;

// 生成建筑
namespace QFramework.BuildingManagement.Commands
{
    public class CreateBuildingCommand : AbstractCommand, IGetService, ICanSendEvent,IServiceSystemLocator
    {
        private readonly GameObject _building;
        private readonly LayerMask _groundMask;

        private CoroutineManager _coroutineManager;
        private IGridManager _gridManager;
        private IBuildingUtility _buildUtility;
        private readonly Camera _camera;
        private EntityManager _entityManager;
        private Entity _buildingEntityPrefab;

        public CreateBuildingCommand(GameObject building, LayerMask groundMask)
        {
            _building = building;
            _groundMask = groundMask;
            _camera = Camera.main;
        }

        protected override void OnExecute()
        {
            _coroutineManager = this.GetServiceObject<CoroutineManager>();
            _gridManager = this.GetServiceObject<IGridManager>();
            _buildUtility = this.GetUtility<IBuildingUtility>();
            CreateBuilding(_building);
        }

        /// <summary>
        /// 创建建筑实体
        /// </summary>
        private void CreateBuildingEntity(Vector3 pos)
        {
            var entityQuery = World.DefaultGameObjectInjectionWorld.EntityManager.
                CreateEntityQuery(typeof(NetWorkDataContainer));
            int ghostId = entityQuery.GetSingleton<NetWorkDataContainer>().Id;
            var endPosition = _camera.ScreenToWorldPoint(Input.mousePosition);
            var clientHelpSystem = this.GetService<ClientHelpSystem>();
            var buildingAuthoring = _building.GetComponent<BuildingAuthoring>();
            clientHelpSystem.SendSpawnCreateEntityRpc(new CreateBaseBuildingRpc(pos, buildingAuthoring.buildingType));
            List<int> ghostIds = new List<int>() { ghostId };
            RequestCommandRpcSystem controller = this.GetService<RequestCommandRpcSystem>();
            controller.SendInputCommand(controller.CreateInputCommand(InputCommandType.Create, endPosition, ghostIds));
            // DelayedEntityQuery(ghostIds);
        }
        
        private void DelayedEntityQuery(List<int> ghostIds)
        {
            Debug.Log("DelayedEntityQuery");
            foreach (var id in ghostIds)
            {
                Entity entity = this.GetService<ClientHelpSystem>().GetEntityByIndexInClientWorld(id);
                this.SendEvent(new CreateEntityEvent()
                {
                    IsEnemy = true,
                    Type = MapUIEntityType.Building,
                    Entity = entity
                });
            }
        }
        /// <summary>
        /// 创建建筑 Mono
        /// </summary>
        /// <param name="building">建筑预制体</param>
        private void CreateBuilding(GameObject building)
        {
            Vector3 endPosition = _camera.ScreenToWorldPoint(Input.mousePosition);
            Debug.LogError(endPosition);
            GameObject previewGameObject = CreatePreviewGameObject(building, endPosition);
            // 启动协程，持续更新预览建筑物的位置，直到鼠标点击放置
            StartPlacementProcess(previewGameObject);
        }


        /// <summary>
        /// 创建预览建筑
        /// </summary>
        /// <param name="building">预览建筑的预制体</param>
        /// <param name="position">预览建筑的位置</param>
        /// <returns></returns>
        private GameObject CreatePreviewGameObject(GameObject building, Vector3 position)
        {
            GameObject previewGameObject = Object.Instantiate(building, position, Quaternion.identity);
            this.SendCommand(new SetBeCreatedBuildingCommand(previewGameObject));
            return previewGameObject;
        }

        /// <summary>
        /// 通过_coroutineManager 开启一个协程
        /// </summary>
        /// <param name="previewGameObject">预览建筑</param>
        private void StartPlacementProcess(GameObject previewGameObject)
        {
            _coroutineManager.StartCoroutine(ContinueFollowMouse(previewGameObject));
        }

        // 持续更新预览建筑物的位置，直到玩家点击鼠标放置建筑物
        private IEnumerator ContinueFollowMouse(GameObject previewGameObject)
        {
            BuildingBase buildingBase = previewGameObject.GetComponent<BuildingBase>();

            // 如果建筑物没有挂载 BuildingBase 组件，抛出异常
            if (buildingBase == null)
                throw new ArgumentException($"Building:{previewGameObject.name} Don't have BuildingBase Component");

            // 持续跟踪鼠标位置直到点击左键放置建筑
            while (!Input.GetMouseButtonDown((int)MouseButton.Left))
            {
                Ray ray = _camera.ScreenPointToRay(Input.mousePosition);
                Debug.DrawRay(ray.origin, ray.direction * 100f, Color.red, 0.5f);
                if (Physics.Raycast(ray, out var hit, Mathf.Infinity, _groundMask))
                {
                    var worldPosition = hit.point;
                    worldPosition.y = 1.75f;
                    var gridPo = _gridManager.SnapToGrid(worldPosition); // 对齐到网格
                    Debug.DrawLine(ray.origin, hit.point, Color.green, 1f);
                    if (_gridManager.CheckHaveBuildingAt(gridPo, buildingBase))
                        previewGameObject.transform.position = _gridManager.SnapToGrid(worldPosition);
                }
                else
                {
                    throw new Exception("放置建筑物射线检测出现问题");
                }

                yield return null;
            }

            var pos = previewGameObject.transform.position;
            GameObject.Destroy(previewGameObject);
            // this.SendCommand(new SetHasCreatedBuildingCommand(previewGameObject));
            // 将建筑物的位置转换为网格坐标
            // Vector3Int gridPosition = _gridManager.WorldToCell(previewGameObject.transform.position);
            // 将建筑物放置在网格上
            // _gridManager.PutObjectAt(buildingBase, gridPosition);
            CreateBuildingEntity(pos);
        }
        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}