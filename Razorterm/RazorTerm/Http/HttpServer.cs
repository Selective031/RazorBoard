using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Threading.Tasks;
using Newtonsoft.Json;
using RazorTerm.Connection;
using RazorTerm.Data;
using RazorTerm.Logging;
using RazorTerm.Utils;

namespace RazorTerm.Http
{
    public class HttpServer : ServiceBase
    {
        private const int MaxLogCount = 200;

        private readonly IConnection _connection;
        private readonly DataCollector _collector;
        private readonly IList<string> _log = new List<string>();
        private HttpListener _http;
        public HttpServer(IConnection connection, DataCollector collector)
        {
            _connection = connection;
            _collector = collector;
        }

        protected override Task OnStart()
        {
            _connection.MessageReceived += OnMessageReceived;
            _http = new HttpListener();
            _http.Prefixes.Add($"http://*:{Settings.HttpServer.Port}/");
            _http.Start();
            Logger.Log($"Http server listening on port {Settings.HttpServer.Port}");
            return Task.CompletedTask;
        }

        private void OnMessageReceived(string message, MessageType _)
        {
            if (message.IsDebugMessage() || message.IsFlashProgressMessage())
            {
                return;
            }

            while (_log.Count > MaxLogCount)
            {
                _log.RemoveAt(0);
            }

            _log.Add(message);
        }

        protected override async Task Run()
        {
            while (Running)
            {
                try
                {
                    var context = await _http.GetContextAsync();
                    switch (context.Request.RawUrl.Trim('/').ToLower())
                    {
                        case "log":
                            WriteTextResponse(context.Response, string.Join("<br>", _log.Reverse()), "text/html");
                            break;
                        case "status":
                            var text = JsonConvert.SerializeObject(_collector.Data, Formatting.Indented);
                            WriteTextResponse(context.Response, text, "application/json");
                            break;
                        default:
                            WriteTextResponse(context.Response, "available: /log /status", "text/html");
                            break;
                    }
                }
                catch (Exception e)
                {
                    Logger.Log(e, LogLevel.Debug);
                }
            }
        }

        private void WriteTextResponse(HttpListenerResponse response, string text, string contentType)
        {
            var templateStart = @"
                <html><head>
                    <style type=""text/css"">
                    body {
                        background-color:#333;
                        color:#eee;
                        font-size:13px;
                        font-family:verdana;
                        }
                    </style>
                </head><body>";
            var templateEnd = "</body></html>";
            var output = System.Text.Encoding.UTF8.GetBytes(templateStart + text + templateEnd);
            response.ContentType = contentType;
            response.ContentLength64 = output.Length;
            var outputStream = response.OutputStream;
            outputStream.Write(output, 0, output.Length);
            outputStream.Close();
        }

        protected override Task OnStop()
        {
            _http.Stop();
            return Task.CompletedTask;
        }
    }
}