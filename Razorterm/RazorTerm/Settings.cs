using System;
using System.IO;
using System.Linq;
using Newtonsoft.Json;
using RazorTerm.Utils;

namespace RazorTerm
{
    public static class Settings
    {
        public static string PortName { get; set; }
        public static string DeviceName { get; set; } = "mower";
        public static string DisplayName { get; set; }

        public static class Mqtt
        {
            public static bool Enabled { get; set; }
            public static string Host { get; set; }
            public static string Username { get; set; }
            public static string Password { get; set; }
            public static bool RebootOnConnectionFailure { get; set; }
        }

        public static class TcpClient
        {
            public static bool Enabled { get; set; }
            public static string Host { get; set; }
            public static string OnConnect { get; set; }
        }

        public static class TcpServer
        {
            public static bool Enabled { get; set; }
        }

        public static class HttpServer
        {
            public static bool Enabled { get; set; }
            public static long Port { get; set; } = 80;
        }

        public static class Service
        {
            public static bool Enabled { get; set; }
        }

        public static class Multi
        {
            public static bool Enabled { get; set; }

            // Comma separated list of other mowers
            public static string Peers { get; set; }
        }

        public static void Parse(string[] args)
        {
            if (!args.Any())
            {
                ReadDefault();
                return;
            }

            for (var i = 0; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "--port-name":
                        PortName = Next(i++);
                        break;
                    case "--device-name":
                        DeviceName = Next(i++);
                        break;
                    case "--mqtt-host":
                        Mqtt.Enabled = true;
                        Mqtt.Host = Next(i++);
                        break;
                    case "--mqtt-username":
                        Mqtt.Username = Next(i++);
                        break;
                    case "--mqtt-password":
                        Mqtt.Password = Next(i++);
                        break;
                    case "--tcp-host":
                        TcpClient.Enabled = true;
                        TcpClient.Host = Next(i++);
                        break;
                    case "--tcp-on-connect":
                        TcpClient.OnConnect = Next(i++);
                        break;
                    case "--tcp-server":
                        TcpServer.Enabled = true;
                        break;
                    case "--service":
                        Service.Enabled = true;
                        break;
                    case "--http-server":
                        HttpServer.Enabled = true;
                        break;
                    case "--http-port":
                        HttpServer.Port = int.Parse(Next(i++));
                        break;
                    case "--save-default":
                        break;
                    default:
                        throw new ArgumentException($"Invalid parameter {args[i]}");
                }
            }

            if (args.Any(a => a == "--save-default"))
            {
                SaveDefault();
            }

            string Next(int i)
            {
                if (args.Length <= i + 1)
                {
                    throw new ArgumentException($"Expected value for parameter {args[i]}");
                }

                if (args[i + 1].StartsWith("--"))
                {
                    throw new ArgumentException($"Unexpected value for parameter {args[i]} ({args[i + 1]} looks like a parameter)");
                }

                return args[i + 1];
            }
        }

        private static void SaveDefault()
        {
            File.WriteAllText("settings.json", StaticSerializer.Serialize(typeof(Settings)));
        }

        private static void ReadDefault()
        {
            if (!File.Exists("settings.json"))
            {
                return;
            }

            var settings = File.ReadAllText("settings.json");
            StaticSerializer.Deserialize(typeof(Settings), settings);
        }
    }
}