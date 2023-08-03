using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;
using RazorTerm.Connection;
using RazorTerm.Data;
using RazorTerm.Http;
using RazorTerm.Logging;
using RazorTerm.Modules;
using RazorTerm.Mqtt;
using RazorTerm.Tcp;

namespace RazorTerm
{
    public static class TermFactory
    {
        public static void Start(string[] args)
        {
            try
            {
                Setup(args);
            }
            catch (Exception e)
            {
                Logger.Log(e);
                throw;
            }
        }

        private static void Setup(string[] args)
        {
            Settings.Parse(args);

            IConnection connection;
            var services = new List<ServiceBase>
            {
                new Logger()
            };

            if (Settings.TcpClient.Enabled)
            {
                connection = new TcpConnection();
                services.Add((TcpConnection)connection);

            }
            else
            {
                connection = new SerialConnection();
                services.Add((SerialConnection)connection);
            }

            var dataCollector = Settings.Mqtt.Enabled || Settings.HttpServer.Enabled
                ? new DataCollector(connection)
                : null;

            MqttServer mqtt = null;
            if (Settings.Mqtt.Enabled)
            {
                Logger.Log("Mqtt enabled");
                mqtt = new MqttServer(dataCollector, connection);
                services.Add(mqtt);
            }

            if (Settings.TcpServer.Enabled)
            {
                var serviceModules = new List<ITermModule>
                {
                    new UpdateModule()
                };

                if (Settings.Multi.Enabled)
                {
                    serviceModules.Add(new MultiModule(dataCollector, mqtt));
                }

                Logger.Log("TcpServer enabled");
                var tcpServer = new TcpServer(connection, new ModuleCollection(serviceModules.ToArray()));
                services.Add(tcpServer);
            }

            if (Settings.HttpServer.Enabled)
            {
                Logger.Log("Http enabled");
                var http = new HttpServer(connection, dataCollector);
                services.Add(http);
            }

            if (dataCollector != null)
            {
                Logger.Log("DataCollector enabled");
                services.Add(dataCollector);
            }

            foreach (var service in services)
            {
                service.Start();
            }

            if (Settings.Service.Enabled)
            {
                Console.WriteLine("Press Q to quit");

                while (Console.ReadKey().Key != ConsoleKey.Q)
                {
                    Thread.Sleep(1000);
                    if (RazorApplication.Stopped)
                    {
                        break;
                    }
                }

                connection.Disconnect();
                foreach (var service in services)
                {
                    _ = service.Stop();
                }

                Logger.Log("Quitting...");
                Environment.Exit(1);
            }
            else
            {
                var modules = Settings.TcpClient.Enabled
                    ? new ModuleCollection(new RazorBoardModule(), new HightlightModule())
                    : new ModuleCollection(new RazorBoardModule(), new HightlightModule(), new PortSettingsModule());

                var term = new Term(connection, modules);
                term.StartTermInput();
            }
        }


        // private static void Init(IConnection connection, IEnumerable<Type> serviceTypes)
        // {
        //     var instances = new List<object>();
        //     while (true)
        //     {
        //         foreach (var serviceType in serviceTypes)
        //         {
        //             var instance = Resolve(serviceType, connection);
        //         }
        //     }
        //
        // }
        //
        // private static object Resolve(Type type, params object[] objects)
        // {
        //     var constructors = type.GetConstructors();
        //     foreach (var constructor in constructors)
        //     {
        //         if (constructor.GetParameters().All(c => objects.Any(o => o.GetType().IsAssignableFrom(c.ParameterType))))
        //         {
        //             Console.WriteLine(type.Name + " [ok]");
        //         }
        //     }
        // }
    }
}