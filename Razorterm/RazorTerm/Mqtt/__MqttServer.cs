// using System;
// using System.Threading.Tasks;
// using RazorTerm.Data;
// using RazorTerm.Logging;
//
// namespace RazorTerm.Mqtt
// {
//     public class MqttServer : ServiceBase
//     {
//         private readonly MqttClient _client;
//
//         public MqttServer(DataCollector collector)
//         {
//             _collector = collector;
//             _client = new MqttClient();
//         }
//
//         protected override async Task OnStart()
//         {
//             await _client.Connect();
//             _collector.DataReady += CollectorOnDataReady;
//         }
//
//         protected override async Task Run()
//         {
//             while (Running)
//             {
//                 await Task.Delay(100);
//             }
//         }
//
//         protected override async Task OnStop()
//         {
//             _collector.DataReady -= CollectorOnDataReady;
//             await _client.Disconnect();
//         }
//
//         private void CollectorOnDataReady(IRazorBoardData data)
//         {
//             try
//             {
//                 _client.SendData(data).Wait();
//             }
//             catch (Exception e)
//             {
//                 Logger.Log(e, LogLevel.Debug);
//             }
//         }
//     }
// }