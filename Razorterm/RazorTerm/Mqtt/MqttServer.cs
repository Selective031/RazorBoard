using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using MQTTnet.Exceptions;
using MQTTnet.Formatter;
using Newtonsoft.Json;
using RazorTerm.Connection;
using RazorTerm.Data;
using RazorTerm.Logging;
using RazorTerm.Modules;

namespace RazorTerm.Mqtt
{
    public class MqttServer : ServiceBase
    {
        private readonly SemaphoreSlim _sendLock = new SemaphoreSlim(1);
        private IMqttClient _client;
        private readonly IConnection _connection;
        private DateTime _lastSuccessfulMessageSent = DateTime.Now;
        private DateTime _startedAt = DateTime.Now;

        public bool IsConnected => _client?.IsConnected == true
                                 && _lastSuccessfulMessageSent - DateTime.Now < TimeSpan.FromSeconds(6);

        private readonly DataCollector _collector;
        public MqttServer(DataCollector collector, IConnection connection)
        {
            _collector = collector;
            _connection = connection;
        }

        protected override async Task OnStart()
        {
            while (Running && !await Connect())
            {
                Logger.Log($"Failed to connect to {Settings.Mqtt.Host}. Reconnecting in {5} seconds");
                CheckIfRebootIsRequired();
                await Task.Delay(TimeSpan.FromSeconds(5));
            }

            _collector.DataReady += SendData;
        }

        protected override async Task Run()
        {
            while (Running)
            {
                await Task.Delay(100);
            }
        }

        protected override async Task OnStop()
        {
            _collector.DataReady -= SendData;
            await Disconnect();
        }

        private async Task<bool> Connect()
        {
            try
            {
                var factory = new MqttFactory();
                _client = factory.CreateMqttClient();

                Logger.Log($"Connecting to mqtt server {Settings.Mqtt.Host} as {Settings.Mqtt.Username}...");

                var options = new MqttClientOptionsBuilder()
                    .WithClientId(Settings.DeviceName)
                    .WithTcpServer(Settings.Mqtt.Host)
                    .WithCredentials(Settings.Mqtt.Username, Settings.Mqtt.Password)
                    .WithCleanSession()
                    .WithProtocolVersion(MqttProtocolVersion.V311)
                    .Build();

                await _client.ConnectAsync(options, CancellationToken.None);

                Logger.Log($"Mqtt connected to {Settings.Mqtt.Host}");

                await Listen();

                return true;
            }
            catch (Exception e)
            {
                Logger.Log(e.Message);
                return false;
            }
        }

        private async Task<bool> Reconnect()
        {
            try
            {
                if (_client.IsConnected)
                {
                    return true;
                }

                Logger.Log($"Trying to reconnect mqtt... ", LogLevel.Debug);
                await _client.ReconnectAsync();
                await Listen();
                Logger.Log("Reconnected mqtt", LogLevel.Debug);
                return true;
            }
            catch (ObjectDisposedException)
            {
                Logger.Log("Mqtt client disposed. Creating new...", LogLevel.Debug);
                return await Connect();
            }
            catch (Exception e)
            {
                Logger.Log(e.Message);
                return false;
            }
        }

        private void CheckIfRebootIsRequired()
        {
            if (!Settings.Mqtt.RebootOnConnectionFailure)
            {
                return;
            }

            if (DateTime.Now - _lastSuccessfulMessageSent > TimeSpan.FromMinutes(10) && DateTime.Now - _startedAt > TimeSpan.FromMinutes(4))
            {
                Logger.Log($"Mqtt connection failed for 10 minutes (Last successful send at: {_lastSuccessfulMessageSent}, started at: {_startedAt}, now: {DateTime.Now}). Exiting...");
                RazorApplication.Stop();
                //Logger.Log("Mqtt connection failed for 10 minutes. Trying to reboot...");
                //UpdateModule.ExecuteCommand("sudo reboot");
            }
        }

        private async Task Listen()
        {
            Logger.Log($"Subscribed to razorboard/{Settings.DeviceName.ToLower()}/command", LogLevel.Debug);

            await _client.SubscribeAsync($"razorboard/{Settings.DeviceName.ToLower()}/command");

            _client.UseApplicationMessageReceivedHandler(e =>
            {
                var payload = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
                if (e.ApplicationMessage.Retain)
                {
                    Logger.Log($"Ignoring retained command: {payload}", LogLevel.Debug);
                    return;
                }

                var command = RazorBoardCommand.Parse(payload);
                if (command != null)
                {
                    Logger.Log($"Sending command: {command}", LogLevel.Debug);
                    _connection.SendMessage(command);
                }
                else
                {
                    Logger.Log($"Sending unknown payload: {payload}", LogLevel.Debug);
                    _connection.SendMessage(payload);
                }
            });
        }

        private async Task Disconnect()
        {
            Logger.Log("Disconnecting mqtt...");

            await Task.Delay(1000);

            if (!_client.IsConnected)
            {
                Logger.Log("Not connected when disconnecting mqtt", LogLevel.Info);
                return;
            }

            await _client.UnsubscribeAsync();
            await _client.DisconnectAsync();

            Logger.Log("Disconnected mqtt");
        }

        private async Task SendData(IRazorBoardData data)
        {
            if (await _sendLock.WaitAsync(1000))
            {
                try
                {
                    CheckIfRebootIsRequired();
                    var timeoutTask = Task.Delay(TimeSpan.FromSeconds(30));
                    var sendTask = SendMessage(data);
                    await Task.WhenAny(sendTask, timeoutTask);
                }
                catch (Exception e) when (e is MqttCommunicationTimedOutException or SocketException)
                {
                    try
                    {
                        await _client.DisconnectAsync();
                    }
                    catch
                    {
                        // Ignore
                    }
                }
                catch (Exception e)
                {
                    Logger.Log(e);
                }
                finally
                {
                    _sendLock.Release();
                }
            }
            else
            {
                Logger.Log("Mqtt send lock sem busy", LogLevel.Debug);
            }
        }

        private async Task<bool> SendMessage(IRazorBoardData data)
        {
            if (!_client.IsConnected)
            {
                Logger.Log("Not connected when sending data to mqtt", LogLevel.Debug);
                if (Running && !await Reconnect())
                {
                    Logger.Log("Failed to reconnect mqtt. Dropping message.");
                    return true;
                }
            }

            var message = new MqttApplicationMessageBuilder()
                .WithTopic($"razorboard/{Settings.DeviceName.ToLower()}/status")
                .WithPayload(Payload(data))
                .WithExactlyOnceQoS()
                .WithRetainFlag(false)
                .Build();

            _client.PublishAsync(message, CancellationToken.None).Wait(3000);

            _lastSuccessfulMessageSent = DateTime.Now;
            Logger.Log("Mqtt message sent....", LogLevel.Debug);
            return true;
        }

        private string Payload(IRazorBoardData data)
        {
            return JsonConvert.SerializeObject(data, Formatting.Indented);
        }
    }
}