using System;
using TMPro;
using Unity.Entities;
using Unity.NetCode;
using Unity.Networking.Transport;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using 中间值;
using 通用;

namespace 客户端
{
    public class ClientConnectManager : MonoBehaviour
    {
        [SerializeField] private TMP_Dropdown connectModeDropdown;
        [SerializeField] private TMP_Dropdown teamDropdown;
        [SerializeField] private TMP_InputField addressInputField;
        [SerializeField] private TMP_InputField portInputField;
        [SerializeField] private Button connectButton;

        private ushort Port => ushort.Parse(portInputField.text);
        private string Address => addressInputField.text;

        private void Awake()
        {
            connectButton.onClick.AddListener(OnPlayerConnect);
        }

        private void OnPlayerConnect()
        {
            DestroyLocalSimulationWorld();
            SceneManager.LoadScene(1);//加载游戏主场景

            switch (connectModeDropdown.value)
            {
                case 0:
                    StartServer();
                    StartClient();
                    break;
                case 1:
                    StartClient();
                    break;
                case 2:
                    StartServer();
                    break;
            }
            SceneManager.LoadSceneAsync("SubScene 1", LoadSceneMode.Additive);
           // SceneManager.LoadSceneAsync("Main", LoadSceneMode.Single);
        }

        private void StartServer()
        {
            var serverWorld = ClientServerBootstrap.CreateServerWorld("Rts Server World");

            var serverEndPoint = NetworkEndpoint.AnyIpv4.WithPort(Port);
   
            var networkDriverQuery =
                serverWorld.EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamDriver>());
            networkDriverQuery.GetSingletonRW<NetworkStreamDriver>().ValueRW.Listen(serverEndPoint);
        }

        private void StartClient()
        {
            var clientWorld = ClientServerBootstrap.CreateClientWorld("Rts Client World");

            var connectionEndPoint = NetworkEndpoint.Parse(Address, Port);
            {
                using var networkDriverQuery =
                    clientWorld.EntityManager.CreateEntityQuery(ComponentType.ReadWrite<NetworkStreamDriver>());
                networkDriverQuery.GetSingletonRW<NetworkStreamDriver>().ValueRW.Connect(clientWorld.EntityManager,connectionEndPoint);
            }

            var team = teamDropdown.value switch
            {
                0=> TeamType.AutoAssign,
                1=> TeamType.Red,
                2=> TeamType.Blue,
                _ => throw new ArgumentOutOfRangeException()
            };

            var teamRequestEntity = clientWorld.EntityManager.CreateEntity();
            clientWorld.EntityManager.AddComponentData(teamRequestEntity, new ClientTeamRequest()
            {
                Value = team,
            });
            
            World.DefaultGameObjectInjectionWorld = clientWorld;
           
        }

        private void DestroyLocalSimulationWorld()
        {
            foreach (var world in World.All)
            {
                if (world.Flags == WorldFlags.Game)
                {
                    world.Dispose();
                    break;
                }
            }
        }
    }
}