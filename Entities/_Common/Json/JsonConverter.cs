using System;
using Newtonsoft.Json;
using Unity.Collections;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;

public class Float3Converter : JsonConverter<float3>
{
    public override void WriteJson(JsonWriter writer, float3 value, JsonSerializer serializer)
    {
        writer.WriteValue($"{value.x},{value.y},{value.z}");
    }

    public override float3 ReadJson(JsonReader reader, Type objectType, float3 existingValue, bool hasExistingValue,
        JsonSerializer serializer)
    {
        string[] parts = ((string)reader.Value).Split(',');
        return new float3(float.Parse(parts[0]), float.Parse(parts[1]), float.Parse(parts[2]));
    }
}

public class NetworkTickConverter : JsonConverter<NetworkTick>
{
    public override void WriteJson(JsonWriter writer, NetworkTick value, JsonSerializer serializer)
    {
        writer.WriteValue(value.TickValue.ToString());
    }

    public override NetworkTick ReadJson(JsonReader reader, Type objectType, NetworkTick existingValue,
        bool hasExistingValue, JsonSerializer serializer)
    {
        int value = int.Parse((string)reader.Value);
        // 从 JSON 读取数字并转换为 NetworkTick
        return new NetworkTick(Convert.ToUInt32(value));
    }
}

public class FixedString128BytesConverter : JsonConverter<FixedString128Bytes>
{
    public override void WriteJson(JsonWriter writer, FixedString128Bytes value, JsonSerializer serializer)
    {
        writer.WriteValue(value.ToString());
    }

    public override FixedString128Bytes ReadJson(JsonReader reader, Type objectType, FixedString128Bytes existingValue,
        bool hasExistingValue, JsonSerializer serializer)
    {
        string value = reader.Value as string;
        return (FixedString128Bytes)value;
    }
}