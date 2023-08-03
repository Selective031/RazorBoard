using System.Collections.Generic;
using RazorTerm.Modules;

namespace RazorTerm
{
    public interface ITerm
    {
        void ReceiveMessage(string message, MessageType messageType);
    }
}