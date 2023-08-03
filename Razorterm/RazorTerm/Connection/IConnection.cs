using System;
using System.Threading.Tasks;

namespace RazorTerm.Connection
{
    public interface IConnection
    {
        event MessageReceivedDelegate MessageReceived;
        event Func<Task> Disconnected;
        Task SendMessage(string message);
        Task Disconnect();
        Task<bool> Connect();
    }

    public delegate void MessageReceivedDelegate(string message, MessageType messageType);
}