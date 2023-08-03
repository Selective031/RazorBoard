using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using RazorTerm.Connection;
using RazorTerm.Logging;

namespace RazorTerm.Tcp
{
    public class TcpConnection : ServiceBase, IConnection
    {
        public event MessageReceivedDelegate MessageReceived;
        public event Func<Task> Disconnected;
        private TcpClient _tcpClient;
        private NetworkStream _stream;

        public async Task SendMessage(string message)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(message) || _stream == null)
                {
                    return;
                }

                var bytes = Encoding.UTF8.GetBytes(message);
                await _stream.WriteAsync(bytes, 0, bytes.Length);
            }
            catch (IOException)
            {
                MessageReceived?.Invoke($"Connected to {Settings.TcpClient.Host} was lost", MessageType.Exception);
                Disconnected?.Invoke();
            }
        }

        protected override async Task OnStart()
        {
            Task.Delay(500).Wait();
            MessageReceived?.Invoke($"Connecting to {Settings.TcpClient.Host}...", MessageType.System);

            var connectionStarted = DateTime.Now;
            var connected = false;
            for (var i = 0; i < 100; i++)
            {
                _tcpClient = new TcpClient();
                connected = _tcpClient.ConnectAsync(Settings.TcpClient.Host, 5042).Wait(1500);
                if (connected)
                {
                    break;
                }

                MessageReceived?.Invoke($"Failed to connect. Reconnecting ({i + 1} / 100)...", MessageType.Warning);
                Task.Delay(1000 + i * 10).Wait();
            }

            if (!connected)
            {
                MessageReceived?.Invoke($"Failed to connect", MessageType.Exception);
                throw new ApplicationException("Failed to connect");
            }
            _stream = _tcpClient.GetStream();

            var connectionTime = DateTime.Now - connectionStarted;
            MessageReceived?.Invoke($"Connected to {Settings.TcpClient.Host} (took {connectionTime.TotalMilliseconds} ms)", MessageType.Success);
            if (!string.IsNullOrEmpty(Settings.TcpClient.OnConnect))
            {
                MessageReceived?.Invoke($"Sending connect command: {Settings.TcpClient.OnConnect}", MessageType.System);
                await SendMessage(Settings.TcpClient.OnConnect);
                Settings.TcpClient.OnConnect = null;
            }
        }

        protected override async Task Run()
        {
            var buffer = new byte[1024];
            while (Running)
            {
                try
                {
                    var bytesRead = await _stream.ReadAsync(buffer, 0, buffer.Length);
                    var messages = Encoding.UTF8.GetString(buffer, 0, bytesRead).Split(new [] {"\r\n"}, StringSplitOptions.RemoveEmptyEntries);
                    foreach (var message in messages)
                    {
                        Logger.Log($"Received tcp message: {message}", LogLevel.Debug);
                        MessageReceived?.Invoke(message, MessageType.ReceiveMessage);
                    }
                }
                catch (Exception e)
                {
                    Logger.Log(e);
                    await Task.Delay(1000);
                }
            }
        }

        protected override async Task OnStop()
        {
            await Disconnect();
        }

        public Task Disconnect()
        {
            Logger.Log("Disconnecting tcp connection");
            _tcpClient?.Close();
            _stream?.Close();
            Disconnected?.Invoke();
            return Task.CompletedTask;
        }

        public async Task<bool> Connect()
        {
            await Task.Delay(1000);
            return _stream != null;
        }
    }
}