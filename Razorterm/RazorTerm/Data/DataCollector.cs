using System;
using System.Threading.Tasks;
using RazorTerm.Connection;

namespace RazorTerm.Data
{
    public class DataCollector : ServiceBase, ITerm
    {
        private static TimeSpan UpdateInterval => TimeSpan.FromSeconds(5);

        public event Func<IRazorBoardData, Task> DataReady;
        public IRazorBoardData Data { get; set; }

        private readonly IConnection _connection;
        private readonly DataParser _dataParser;
        public DataCollector(IConnection connection)
        {
            _connection = connection;
            _dataParser = new DataParser();
        }

        public void ReceiveMessage(string message, MessageType messageType)
        {
            _dataParser.Parse(message);
        }

        protected override Task OnStart()
        {
            _connection.MessageReceived += ReceiveMessage;
            return Task.CompletedTask;
        }

        protected override async Task Run()
        {
            DateTime lastSend = DateTime.Now;
            DateTime? lastDebugCommand = null;
            await _connection.SendMessage("DEBUG ON");

            while (Running)
            {
                if (_dataParser.IsComplete || (_dataParser.AnyData && DateTime.Now - lastSend > TimeSpan.FromSeconds(20)))
                {
                    Data = _dataParser.ReadAndReset();
                    DataReady?.Invoke(Data);

                    for (var i = 0; i < UpdateInterval.TotalSeconds && Running; i++)
                    {
                        await Task.Delay(1000);
                    }

                    lastSend = DateTime.Now;
                    lastDebugCommand = DateTime.Now;
                }
                else if (lastDebugCommand == null || DateTime.Now - lastDebugCommand > TimeSpan.FromMinutes(4))
                {
                    // Maybe the "DEBUG ON" command was not received by the connection
                    await _connection.SendMessage("DEBUG ON");
                    lastDebugCommand = DateTime.Now;
                }

                await Task.Delay(100);
            }
        }

        protected override async Task OnStop()
        {
            _connection.MessageReceived -= ReceiveMessage;
            await _connection.SendMessage("DEBUG OFF");
        }
    }
}