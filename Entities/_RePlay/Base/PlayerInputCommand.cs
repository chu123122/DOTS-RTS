using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using DefaultNamespace;
using Newtonsoft.Json;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using JsonSerializer = System.Text.Json.JsonSerializer;

namespace _RePlaySystem.Base
{
    public enum InputCommandType
    {
        Move,
        Create,
    }

    public struct PlayerInputCommand
    {
        public NetworkTick Tick;
        public InputCommandType Type;
        public int PlayerNetWorkId;
        public float3 Position;
        public BlobAssetReference<SelectedUnitsBlob> Units;

        public readonly PlayerInputCommandData ToCommandData()
        {
            var options = new JsonSerializerOptions 
            { 
                ReferenceHandler = ReferenceHandler.Preserve 
            };
            return new PlayerInputCommandData()
            {
                
                Tick = this.Tick,
                Type =  this.Type,
                PlayerNetWorkId = this.PlayerNetWorkId,
                Position =  this.Position,//JsonSerializer.Serialize(this.Position),
                Units = JsonSerializer.Serialize(Units.Value.UnitEntityIndex.ToArray()),
            };
        }
        public static PlayerInputCommandData[] ToCommandDatas (PlayerInputCommand[] commandDatas)
        {
            int i = 0;
            PlayerInputCommandData[] commands = new PlayerInputCommandData[commandDatas.Length];
            foreach (var commandData in commandDatas)
            {
                commands[i] = commandData.ToCommandData();
                i++;
            }
            return commands;
        }

        public override string ToString()
        {
            string entityName = "";
            foreach (var entity in this.Units.Value.UnitEntityIndex.ToArray())
            {
                entityName += entity.ToString();
            }

            if (this.Type == InputCommandType.Move)
                return $"时间：{this.Tick.TickValue}+" +
                       $"目标GhostIds：{entityName}+" +
                       $"目标移动位置：{this.Position}";
            else
                return $"时间：{this.Tick.TickValue}+" +
                       $"目标GhostIds：{entityName}+" +
                       $"玩家ID：{this.PlayerNetWorkId}+" +
                       $"单位创建位置：{this.Position}";
        }
    }
    
    public struct PlayerInputCommandData:IServiceSystemLocator
    {
        public NetworkTick Tick;
        public InputCommandType Type;
        public int PlayerNetWorkId;
        public float3 Position;
        public FixedString128Bytes Units;

        public readonly PlayerInputCommand ToCommand()
        {
            string unitsJson = Units.ToString()==""?"[]":Units.ToString();
            int[] unitEntityIndexes = JsonSerializer.Deserialize<int[]>(unitsJson);
            BlobAssetReference<SelectedUnitsBlob> units = this.GetService<RequestCommandRpcSystem>()
                .CreateBlobAssetReference(unitEntityIndexes);
            return new PlayerInputCommand()
            {
                Tick=Tick,
                Type=Type,
                PlayerNetWorkId=PlayerNetWorkId,
                Position=Position,
                Units=units
            };
        }
        public static PlayerInputCommand[] ToCommands (PlayerInputCommandData[] commandDatas)
        {
            int i = 0;
            PlayerInputCommand[] commands = new PlayerInputCommand[commandDatas.Length];
            foreach (var commandData in commandDatas)
            {
                commands[i] = commandData.ToCommand();
                i++;
            }
            return commands;
        }
    }

    public struct SelectedUnitsBlob
    {
        public BlobArray<int> UnitEntityIndex;
    }
    
    public struct RtsReplayComponent : IComponentData
    {
        public NativeList<PlayerInputCommand> CommandHistory;
        public NetworkTick CurrentReplayTick;
        
        public RtsReplayDataComponent ToRePlayDataComponent()
        {
            JsonConvert.DefaultSettings = () => new JsonSerializerSettings {
                Converters = { new Float3Converter(), new NetworkTickConverter(),new FixedString128BytesConverter() }
            };
            
            PlayerInputCommand[] commands = this.CommandHistory.AsArray().ToArray();
            PlayerInputCommandData[] commandDatas=PlayerInputCommand.ToCommandDatas(commands);
            FixedString4096Bytes json = JsonConvert.SerializeObject(commandDatas);
            
            RtsReplayDataComponent replayDataComponent = new RtsReplayDataComponent()
            {
                CommandDataHistory = json,
                CurrentReplayTick = this.CurrentReplayTick
            };
            return replayDataComponent;
        }
    }
    
    public struct RtsReplayDataComponent : IComponentData
    {
        public FixedString4096Bytes CommandDataHistory;  
        public NetworkTick CurrentReplayTick;
        
        public RtsReplayComponent ToReplayComponent()
        {
            JsonConvert.DefaultSettings = () => new JsonSerializerSettings {
                Converters = { new Float3Converter(), new NetworkTickConverter(),new FixedString128BytesConverter()}
            };
            string json = this.CommandDataHistory.ToString() == ""
                ? "[]" : this.CommandDataHistory.ToString();
            PlayerInputCommandData[] commandDatas =
                JsonConvert.DeserializeObject<PlayerInputCommandData[]>(json);
            PlayerInputCommand[] commands = PlayerInputCommandData.ToCommands(commandDatas);
            
            var nativeArray = new NativeArray<PlayerInputCommand>(commands, Allocator.Temp);
            var nativeList = new NativeList<PlayerInputCommand>(Allocator.Persistent);
            nativeList.AddRange(nativeArray);
            
            return new RtsReplayComponent()
            {
                CommandHistory = nativeList,
                CurrentReplayTick = this.CurrentReplayTick
            };
        }
    }
}