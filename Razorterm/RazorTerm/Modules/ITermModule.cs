using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RazorTerm.Connection;

namespace RazorTerm.Modules
{
    public interface ITermModule
    {
        IDictionary<string, Action> Commands { get; }
        Task Init(IConnection connection);
        bool ParseCommand(string command);
        bool ParseReceived(string command);
        bool Mute(string command);
    }
}