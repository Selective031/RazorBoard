using System;
using System.Collections.Generic;
using System.Linq;
using RazorTerm.Connection;

namespace RazorTerm.Modules
{
    public class ModuleCollection
    {
        public IDictionary<string, Action> Commands => ParseModuleCommands();
        public IEnumerable<string> Options => Commands.Select(cmd => cmd.Key);
        private readonly IList<ITermModule> _modules;

        public ModuleCollection(params ITermModule[] modules)
        {
            _modules = modules;
        }

        public void Init(IConnection connection)
        {
            foreach (var module in _modules)
            {
                module.Init(connection);
            }
        }

        public bool TryInvokeMessageReceived(string message)
        {
            return _modules.Any(module => module.ParseReceived(message));
        }

        public bool Mute(string message)
        {
            return _modules.Any(module => module.Mute(message));
        }

        public bool TryInvoke(string command)
        {
            if (Commands.ContainsKey(command) && Commands[command] != null)
            {
                Commands[command].Invoke();
                return true;
            }

            return _modules.Any(module => module.ParseCommand(command));
        }

        private IDictionary<string, Action> ParseModuleCommands()
        {
            var commands = _modules
                .SelectMany(m => m.Commands)
                .ToDictionary(pair => pair.Key, pair => pair.Value);

            commands.Add("!quit", null);

            foreach (var command in commands.Keys.ToList())
            {
                var parts = command.Split(' ');
                var path = string.Empty;
                foreach (var part in parts)
                {
                    path = $"{path} {part}".Trim();
                    if (!commands.ContainsKey(path))
                    {
                        commands.Add(path, null);
                    }
                }
            }

            return commands;
        }
    }
}