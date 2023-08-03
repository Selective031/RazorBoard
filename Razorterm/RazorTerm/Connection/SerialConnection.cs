using System;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RazorTerm.Logging;

namespace RazorTerm.Connection
{
    public sealed class SerialConnection : ServiceBase, IConnection
    {
        public SerialPort SerialPort;

        public event MessageReceivedDelegate MessageReceived;
        public event Func<Task> Disconnected;
        private bool _hasConnected;

        private void ApplyDefaults()
        {
            SerialPort = new SerialPort
            {
                NewLine = "\r\n",
                BaudRate = 115200,
                DataBits = 8,
                StopBits = StopBits.One,
                Parity = Parity.None,
                Handshake = Handshake.None
            };
        }

        public SerialConnection()
        {
            Logger.EventLogged += (message, type) => MessageReceived?.Invoke(message, type);

            ApplyDefaults();

            if (string.IsNullOrEmpty(Settings.PortName))
            {
                foreach (var port in SerialPort.GetPortNames())
                {
                    SerialPort.PortName = port;
                    if (Connect().Result)
                    {
                        break;
                    }
                }
            }
            else
            {
                var portName = SerialPort.GetPortNames()
                    .FirstOrDefault(p =>
                        string.Equals(p, Settings.PortName, StringComparison.CurrentCultureIgnoreCase));

                if (!string.IsNullOrEmpty(portName))
                {
                    SerialPort.PortName = portName;
                    Connect();
                }
            }
        }
        protected override async Task OnStart()
        {
            while (!await Connect())
            {
                await Task.Delay(1000);
                if (!Running)
                {
                    return;
                }
            }
        }

        public Task<bool> Connect()
        {
            try
            {
                if (!SerialPort.GetPortNames().Any())
                {
                    Logger.Log("No ports available");
                    return Task.FromResult(false);
                }

                if (SerialPort.IsOpen)
                {
                    Disconnect();
                }

                if (!SerialPort.GetPortNames().Contains(SerialPort.PortName))
                {
                    SerialPort.PortName = SerialPort.GetPortNames().FirstOrDefault();
                }

                Logger.Log($"Connecting to to {SerialPort.PortName}...");

                SerialPort.ReadTimeout = 500;
                SerialPort.WriteTimeout = 500;

                SerialPort.Open();
                _hasConnected = true;

                Logger.Log($"Connected to {SerialPort.PortName}");
                MessageReceived?.Invoke($"Connected to {SerialPort.PortName}", MessageType.Success);
            }
            catch (Exception e)
            {
                Logger.Log(e.Message);
                return Task.FromResult(false);
            }

            return Task.FromResult(true);
        }

        protected override async Task Run()
        {
            var emptyCount = 0;
            while (Running)
            {
                if (!_hasConnected)
                {
                    await Task.Delay(1000);
                }

                try
                {
                    var message = SerialPort.ReadLine();
                    if (string.IsNullOrWhiteSpace(message))
                    {
                        emptyCount++;
                    }
                    else
                    {
                        emptyCount = 0;
                    }

                    if (emptyCount > 10)
                    {
                        throw new OperationCanceledException("Serial ports appears disconnected.");
                    }

                    MessageReceived?.Invoke(message, MessageType.ReceiveMessage);
                }
                catch (TimeoutException)
                {
                    // Ignore
                }
                catch (Exception e) when (e is OperationCanceledException || e is InvalidOperationException)
                {
                    if (!_hasConnected)
                    {
                        continue;
                    }

                    Logger.Log(e.Message);
                    Logger.Log("Trying to reconnect...");

                    while (Running && !Reconnect())
                    {
                        MessageReceived?.Invoke($"Failed to reconnect to reconnect. Retrying...", MessageType.Warning);
                        await Task.Delay(5000);
                    }

                    MessageReceived?.Invoke($"Reconnected to {SerialPort.PortName}", MessageType.Success);
                    emptyCount = 0;

                }
                catch (Exception e)
                {
                    LogException(e);
                }
            }
        }

        protected override Task OnStop()
        {
            Disconnect();
            SerialPort?.Dispose();
            return Task.CompletedTask;
        }

        private bool Reconnect()
        {
            try
            {
                SerialPort.Close();
                SerialPort.Open();
                return SerialPort.IsOpen;
            }
            catch (Exception)
            {
                MessageReceived?.Invoke($"Failed to reopen port {SerialPort.PortName}", MessageType.Warning);
                return false;
            }
        }

        public Task SendMessage(string message)
        {
            try
            {
                if (SerialPort.IsOpen)
                {
                    if (message == "RECONNECT")
                    {
                        Disconnect();
                        Task.Delay(500).Wait();
                        Connect();
                        return Task.CompletedTask;
                    }

                    SerialPort.WriteLine(message + "\r\n");
                }
                else
                {
                    MessageReceived?.Invoke("Not connected", MessageType.Warning);
                }

            }
            catch (Exception e)
            {
                Logger.Log(e, LogLevel.Debug);
            }

            return Task.CompletedTask;
        }

        public Task Disconnect()
        {
            if (!SerialPort.IsOpen)
            {
                Logger.Log("Port already closed");
                return Task.CompletedTask;
            }

            _hasConnected = false;

            Logger.Log("Closing port");
            SerialPort.Close();

            Logger.Log("Port closed");

            Disconnected?.Invoke();
            return Task.CompletedTask;
        }

        public void SetEncoding(Encoding encoding)
        {
            SerialPort.Encoding = encoding;
        }

        private void LogException(Exception e)
        {
            Logger.Log(e);
            MessageReceived?.Invoke($"{e.GetType()}: {e.Message}", MessageType.Exception);
        }
    }
}