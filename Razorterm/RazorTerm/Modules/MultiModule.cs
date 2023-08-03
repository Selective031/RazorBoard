using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using RazorTerm.Connection;
using RazorTerm.Data;
using RazorTerm.Logging;
using RazorTerm.Mqtt;

namespace RazorTerm.Modules
{
    public class MultiModule : ITermModule
    {
        private DateTime? _requestedDockAt;
        private bool HasRequestedToDock => _requestedDockAt != null;
        private DateTime? _lastRequestSentAt;
        private readonly DataCollector _dataCollector;
        private IConnection _connection;
        private readonly MqttServer _mqtt;

        public IDictionary<string, Action> Commands => new Dictionary<string, Action>
        {
            { "[REQUEST TO DOCK]", ReceivedDockRequest },
            { "[DOCK REQUEST OK]", DockRequestOk },
            { "[DOCK REQUEST DENIED - SEARCHING]", DockRequestDeniedSearching },
            { "[DOCK REQUEST DENIED - DOCKED]", DockRequestDeniedDocked },
            { "[REPEAT]", RepeatRequest },
        };

        private static string[] Peers => Settings.Multi.Peers.Split(',').ToArray();

        public MultiModule(DataCollector dataCollector, MqttServer mqtt)
        {
            _dataCollector = dataCollector;
            _mqtt = mqtt;
        }

        private async void RepeatRequest()
        {
            if (HasRequestedToDock)
            {
                Logger.Log("[MULTI] Repeat dock request");
                await RequestToDock();
            }
        }

        private void DockRequestOk()
        {
            if (HasRequestedToDock)
            {
                Logger.Log("[MULTI] Dock request ok");
                _connection.SendMessage("ENABLE");
                _connection.SendMessage("TRACK GUIDE");
                _requestedDockAt = null;
            }
        }

        private void DockRequestDeniedDocked()
        {
            if (HasRequestedToDock)
            {
                Logger.Log("[MULTI] Dock request denied - docked");
                _connection.SendMessage("DISABLE");
            }
        }

        private void DockRequestDeniedSearching()
        {
            if (HasRequestedToDock)
            {
                Logger.Log("[MULTI] Dock request denied - searching");
            }
        }

        private async Task CheckState()
        {
            if (_requestedDockAt == null)
            {
                return;
            }

            if (DateTime.Now - _lastRequestSentAt > TimeSpan.FromMinutes(1))
            {
                await RequestToDock();
                if (_mqtt.IsConnected && _dataCollector.Data.Voltage <= 22)
                {
                    await _connection.SendMessage("DISABLE");
                }
            }
        }

        private async void ReceivedDockRequest()
        {
            if (_dataCollector.Data.Docked == true
                && _dataCollector.Data.BatteryFullyCharged == true
                && _dataCollector.Data.DockingLocked == false)
            {
                await _connection.SendMessage("UNDOCK AND STOP");
            }

            if (_dataCollector.Data.Docked == true)
            {
                await SendToAll("[DOCK REQUEST DENIED - DOCKED]");
            }
            else if (_dataCollector.Data.PerimeterTracking == true)
            {
                await SendToAll("[DOCK REQUEST DENIED - SEARCHING]");
            }
        }

        public Task Init(IConnection connection)
        {
            _connection = connection;
            _connection.MessageReceived += ConnectionOnMessageReceived;
            return Task.CompletedTask;
        }

        public bool ParseCommand(string command)
        {
            return false;
        }

        public bool ParseReceived(string command)
        {
            return false;
        }

        public bool Mute(string command)
        {
            return false;
        }

        private async void ConnectionOnMessageReceived(string message, MessageType _)
        {
            if (message.Contains("[REQUEST TO DOCK]"))
            {
                await RequestToDock();
            }
            else if (message.Contains("[DOCK FREE]"))
            {
                await SendToAll("[REPEAT]");
            }
            else if (message.Contains("[FULLY CHARGED]"))
            {
                await SendToAll("[REPEAT]");
            }

            await CheckState();
        }

        private async Task RequestToDock()
        {
            if (_lastRequestSentAt == null || DateTime.Now - _lastRequestSentAt > TimeSpan.FromSeconds(5))
            {
                Logger.Log("[MULTI] Requesting to dock");
                _requestedDockAt ??= DateTime.Now;
                if (await SendToAll("[REQUEST TO DOCK]"))
                {
                    _lastRequestSentAt = DateTime.Now;
                }
            }
        }

        private async Task<bool> SendToAll(string message)
        {
            Logger.Log($"[MULTI] Sending to all peers: {message}");
            var success = true;
            foreach (var peer in Peers)
            {
                if (!await SendToPeer(peer, message))
                {
                    success = false;
                }
            }

            return success;
        }

        private async Task<bool> SendToPeer(string peer, string message)
        {
            try
            {
                var tcpClient = new TcpClient();
                var connected = tcpClient.ConnectAsync(Settings.TcpClient.Host, 5042).Wait(2500);
                if (connected)
                {
                    var stream = tcpClient.GetStream();
                    var bytes = Encoding.UTF8.GetBytes(message);
                    await stream.WriteAsync(bytes, 0, bytes.Length);
                    Logger.Log($"[MULTI] Successfully sent to {peer}: {message}");
                }

            }
            catch (Exception e)
            {
                Logger.Log(e);
            }

            Logger.Log($"[MULTI] Failed to send to {peer}: {message}");
            return false;
        }
    }
}