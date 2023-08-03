using System;
using System.Collections.Generic;
using System.Reflection;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using RazorTerm.Logging;

namespace RazorTerm.Utils
{
    public static class StaticSerializer
    {
        public static string Serialize(Type type)
        {
            return JsonConvert.SerializeObject(SerializeType(type), Formatting.Indented);
        }

        private static Dictionary<string, object> SerializeType(Type type)
        {
            var result = new Dictionary<string, object>();

            var properties = type.GetProperties(BindingFlags.Static | BindingFlags.Public);
            var nestedTypes = type.GetNestedTypes(BindingFlags.Static | BindingFlags.Public);

            foreach (var propertyInfo in properties)
            {
                result.Add(propertyInfo.Name, propertyInfo.GetGetMethod().Invoke(null, null));
            }

            foreach (var nestedType in nestedTypes)
            {
                result.Add(nestedType.Name, SerializeType(nestedType));
            }

            return result;
        }

        public static void Deserialize(Type type, string json)
        {
            var dict = JsonConvert.DeserializeObject<Dictionary<string, object>>(json, new NestedDictionaryConverter());

            DeserializeType(type, dict);
        }

        private static void DeserializeType(Type type, Dictionary<string, object> dict)
        {
            foreach (var pair in dict)
            {
                if (pair.Value is Dictionary<string, object> nestedDict && type.GetNestedType(pair.Key) is Type nestedType)
                {
                    DeserializeType(nestedType, nestedDict);
                }
                else if (type.GetProperty(pair.Key) is PropertyInfo propertyInfo)
                {
                    propertyInfo.SetMethod.Invoke(null, new[] { pair.Value });
                }
                else
                {
                    Logger.Log($"Invalid property for {type.Name}: {pair.Key}", LogLevel.Warning);
                }
            }
        }
    }

    internal class NestedDictionaryConverter : CustomCreationConverter<IDictionary<string, object>>
    {
        public override IDictionary<string, object> Create(Type objectType)
        {
            return new Dictionary<string, object>();
        }

        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(object) || base.CanConvert(objectType);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            if (reader.TokenType == JsonToken.StartObject
                || reader.TokenType == JsonToken.Null)
                return base.ReadJson(reader, objectType, existingValue, serializer);

            return serializer.Deserialize(reader);
        }
    }
}