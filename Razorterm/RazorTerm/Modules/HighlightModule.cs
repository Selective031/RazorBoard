using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using RazorTerm.Connection;

namespace RazorTerm.Modules
{
    public class HightlightModule : ITermModule
    {
        private List<string> _highlights = new List<string>();
        private List<string> _breaks = new List<string>();
        private bool _highlightsOnly = false;

        public IDictionary<string, Action> Commands
        {
            get
            {
                var dict = new Dictionary<string, Action>();

                dict.Add("!guide", null); //temp guide debug
                dict.Add("!pid", null); //temp guide debug
                dict.Add("!setpoint", null); //temp guide debug
                dict.Add("!highlight", null);
                dict.Add("!break", null);
                dict.Add("!highlights-only", () => _highlightsOnly = !_highlightsOnly);
                dict.Add("!clear highlights", () => {
                    _highlights.Clear();
                    _highlightsOnly = false;
                });
                dict.Add("!clear breaks", () => _breaks.Clear());

                return dict;
            }
        }

        public Task Init(IConnection connection)
        {
            return Task.CompletedTask;
        }

        public bool ParseCommand(string command)
        {
            if (command.StartsWith("!highlight "))
            {
                _highlights.Add(command.Substring("!highlight ".Length));
                return true;
            }

            if (command.StartsWith("!break "))
            {
                _breaks.Add(command.Substring("!break ".Length));
                return true;
            }

            if (command == "!guide") //temp guide debug
            {
                _highlights.Add("->");
                _highlightsOnly = true;
                _breaks.Add("V1:");
            }

            if (command == "!setpoint") //temp guide debug
            {
                _highlights.Add("setPoint");
                _highlights.Add("error");
                _highlightsOnly = true;
            }

            if (command == "!pid") //temp guide debug
            {
                _highlights.Add("kp:");
                _highlights.Add("kd:");
                _highlights.Add("ki:");
                _breaks.Add("kd:");
                _highlightsOnly = true;
            }

            return false;
        }

        public bool ParseReceived(string command)
        {
            var result = _highlights.Any() && _highlightsOnly;

            if (_highlights.Any(h => command.ToLower().Contains(h.ToLower())))
            {
                Console.ForegroundColor = ConsoleColor.Green;
                result = false;
            }

            if (_breaks.Any(h => command.ToLower().Contains(h.ToLower())))
            {
                if (result == false)
                {
                    Console.WriteLine(command);
                }

                Console.WriteLine();
                result = true;
            }

            return result;
        }

        public bool Mute(string command)
        {
            return false;
        }
    }
}