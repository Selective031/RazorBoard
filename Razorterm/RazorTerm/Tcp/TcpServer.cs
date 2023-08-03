using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using RazorTerm.Connection;
using RazorTerm.Logging;
using RazorTerm.Modules;

namespace RazorTerm.Tcp
{
    public class TcpServer : ServiceBase, ITerm
    {
        private TcpListener _tcpListener;
        private readonly IConnection _connection;
        private CancellationTokenSource _cancellationToken;
        private ModuleCollection _modules;

        private readonly List<(TcpClient client, NetworkStream stream)> _tcpClients = new();
        private object _clientLock = new ();

        public TcpServer(IConnection connection, ModuleCollection modules)
        {
            _connection = connection;
            _modules = modules;
            _modules.Init(_connection);
            _connection.MessageReceived += ReceiveMessage;
            //_connection.Disconnected += Stop;
        }

        protected override Task OnStart()
        {
            _cancellationToken = new CancellationTokenSource();
            _tcpListener = new TcpListener(new IPEndPoint(IPAddress.Any, 5042));
            _tcpListener.Start();
            return Task.CompletedTask;
        }

        protected override async Task Run()
        {
            Logger.Log("Tcp server started and listening for connections");

            while (Running)
            {
                try
                {
                    var tcpClient = await _tcpListener.AcceptTcpClientAsync();
                    _ = StartTcpClientAsync(tcpClient);
                }
                catch (Exception e)
                {
                    if (Running)
                    {
                        Logger.Log(e);
                        await Task.Delay(500);
                    }
                }
            }
        }

        protected override Task OnStop()
        {
            _cancellationToken.Cancel();
            _tcpListener.Server.Close();
            _tcpListener.Stop();
            return Task.CompletedTask;
        }

        private async Task StartTcpClientAsync(TcpClient tcpClient)
        {
            try
            {
                var stream = tcpClient.GetStream();
                Lock(() =>
                {
                    _tcpClients.Add((tcpClient, stream));
                });

                Logger.Log($"Serial connection from: {tcpClient.Client.RemoteEndPoint}");
                ReceiveMessage($"Hello razorterm, my name is {Settings.DisplayName ?? Settings.DeviceName}", MessageType.Undefined);
                {
                    var buffer = new byte[1024];
                    while (Running)
                    {
                        var bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length, _cancellationToken.Token);
                        var message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                        if (string.IsNullOrEmpty(message))
                        {
                            continue;
                        }

                        Logger.Log($"Received tcp message: {message}", LogLevel.Debug);

                        if (message == "KILL RAZORTERM")
                        {
                            Logger.Log("KILL COMMAND RECEIVED");
                            RazorApplication.Stop();
                            return;
                        }

                        if (!_modules.TryInvoke(message))
                        {
                            await _connection.SendMessage(message);
                        }
                    }

                    Logger.Log("Closing end of client loop");
                    stream.Close();
                }
            }
            catch (Exception)
            {
                Logger.Log("Tcp client disconnected");
            }
            finally
            {
                Lock(() => _tcpClients.RemoveAll(tc => tc.client == tcpClient));
            }
        }

        private void Lock(Action action)
        {
            lock (_clientLock)
            {
                try
                {
                    action();
                }
                catch (Exception e)
                {
                    Logger.Log(e);
                }
            }
        }

        public async void ReceiveMessage(string message, MessageType messageType)
        {
            try
            {
                message = message.Trim('\r', '\n');
                message += "\r\n";

                if (string.IsNullOrWhiteSpace(message))
                {
                    return;
                }

                var bytes = Encoding.UTF8.GetBytes(message);

                foreach (var client in _tcpClients.ToList())
                {
                    try
                    {
                        await client.stream.WriteAsync(bytes, 0, bytes.Length);
                    }
                    catch (IOException)
                    {
                        DisconnectClient(client);
                    }
                    catch (SocketException)
                    {
                        DisconnectClient(client);
                    }
                    catch (ObjectDisposedException)
                    {
                        ClearConnections();
                    }
                }
            }
            catch (Exception e)
            {
                Logger.Log(e);
            }
        }

        private void ClearConnections()
        {
            Lock(() => _tcpClients.Clear());
        }

        private void DisconnectClient((TcpClient client, NetworkStream stream) client)
        {
            try
            {
                Lock(() => _tcpClients.Remove(client));
                client.client.Dispose();
            }
            catch (Exception)
            {
                // Ignore
            }
        }
    }
}