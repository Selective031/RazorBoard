using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using RazorTerm.Connection;
using RazorTerm.Utils;

namespace RazorTerm.Modules
{
    public class RazorBoardModule : ITermModule
    {
        private readonly Regex _commandRegex = new Regex(@"^(?<cmd>[\w ]+?) {2,}- .*$");
        private readonly Regex _nameRegex = new Regex(@"^Hello razorterm, my name is (?<name>.*)$");
        private readonly HashSet<string> _commands = new HashSet<string>();
        private bool _hideDebug = true;
        private bool _hideErrors = false;

        public Task Init(IConnection connection)
        {
            connection.MessageReceived += ConnectionOnMessageReceived;
            _commands.Add("HELP");
            _commands.Add("DISABLE");
            _commands.Add("!DEBUG");
            _commands.Add("!ERRORS");
            return Task.CompletedTask;
        }

        public bool ParseCommand(string command)
        {
            if (command == "!DEBUG")
            {
                _hideDebug = !_hideDebug;
                return true;
            }

            if (command == "!ERRORS")
            {
                _hideErrors = !_hideErrors;
                return true;
            }

            if (command.Equals("DEBUG ON", StringComparison.InvariantCultureIgnoreCase))
            {
                _hideDebug = false;
            }

            return false;
        }

        public bool ParseReceived(string command)
        {
            return false;
        }

        public bool Mute(string command)
        {
            return (_hideDebug && command.IsDebugMessage())
                   || (_hideErrors && command.IsErrorMessage());
        }

        private void ConnectionOnMessageReceived(string message, MessageType _)
        {
            if (_commands.Count <= 6)
            {
                var nameMatch = _nameRegex.Match(message);
                if (nameMatch.Success)
                {
                    Settings.DisplayName = nameMatch.Groups["name"].Value;
                }
            }

            var match = _commandRegex.Match(message);
            if (match.Success)
            {
                var cmd = match.Groups["cmd"].Value;
                if (!_commands.Contains(cmd))
                {
                    _commands.Add(cmd);
                }
            }
        }

        public IDictionary<string, Action> Commands => _commands.OrderBy(c => c).ToDictionary(x => x, _ => (Action)null);
    }
}