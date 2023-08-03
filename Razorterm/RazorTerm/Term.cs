using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using RazorTerm.Connection;
using RazorTerm.Modules;
using RazorTerm.Utils;

namespace RazorTerm
{
    public class Term : ITerm
    {
        private const string QuitCommand = "!quit";

        private readonly StringBuilder _builder = new StringBuilder();
        private List<string> _history = new List<string>();
        private readonly ModuleCollection _modules;
        private readonly IConnection _connection;
        private readonly object _consoleLock = new object();
        private string _lastMessageReceived = string.Empty;
        // private string _highlight = null;
        // private bool _highlightOnly = false;
        // private string _break = null;

        public Term(IConnection connection, ModuleCollection modules)
        {
            _connection = connection;
            _modules = modules;
            _modules.Init(_connection);
        }

        public void StartTermInput()
        {
            _connection.MessageReceived += ReceiveMessage;

            while (true)
            {
                var input = ReadInput(out var exit);

                if (exit || input == QuitCommand)
                {
                    _connection.Disconnect();
                    break;
                }

                if (string.IsNullOrWhiteSpace(input))
                {
                    continue;
                }

                // if (input.StartsWith("!h "))
                // {
                //     _highlight = input.Substring(3);
                //     continue;
                // }
                //
                // if (input.StartsWith("!b "))
                // {
                //     _break = input.Substring(3);
                //     continue;
                // }
                //
                // if (input == "!hh")
                // {
                //     _highlightOnly = !_highlightOnly;
                //     continue;
                // }

                if (!_modules.TryInvoke(input))
                {
                    _connection.SendMessage(input);
                }
            }
        }
        private void ResetCursor()
        {
            Console.SetCursorPosition(0, Console.CursorTop);
        }

        private void WriteCaret()
        {
            ResetCursor();
            Console.ForegroundColor = ConsoleColor.Magenta;
            Console.Write($"{Settings.DisplayName} ");
            Console.ForegroundColor = ConsoleColor.DarkMagenta;
            Console.Write("# ");
            Console.ResetColor();
        }

        private string ReadInput(out bool exit)
        {
            exit = false;

            var matchingOn = string.Empty;
            var matchIndex = -1;
            var historyIndex = _history.Count;
            _builder.Clear();

            WriteCaret();
            var input = Console.ReadKey(intercept: true);

            while (input.Key != ConsoleKey.Enter)
            {
                if (input.Key == ConsoleKey.Q && input.Modifiers == ConsoleModifiers.Control)
                {
                    exit = true;
                    return null;
                }

                var currentInput = _builder.ToString();
                if (input.Key == ConsoleKey.Tab)
                {
                    if (string.IsNullOrWhiteSpace(matchingOn) || !IsMatch(currentInput, matchingOn))
                    {
                        matchingOn = currentInput;
                        matchIndex = -1;
                    }

                    var matches = _modules.Options.Where(item => IsMatch(item, matchingOn)).ToList();

                    var match = string.Empty;
                    if (matches.Any())
                    {
                        if (input.Modifiers.HasFlag(ConsoleModifiers.Shift))
                        {
                            if (matchIndex <= 0)
                            {
                                matchIndex = matches.Count - 2;
                            }
                            else
                            {
                                matchIndex -= 2;
                            }
                        }

                        match = matches.FirstOrDefault(item => matches.IndexOf(item) > matchIndex) ?? matches.First();
                    }

                    if (!string.IsNullOrWhiteSpace(currentInput) && !string.IsNullOrEmpty(match))
                    {
                        matchIndex = matches.IndexOf(match);

                        _builder.Clear();
                        _builder.Append(match);

                        WriteSuggestion(match, matchingOn);
                    }
                }
                else if (input.Key == ConsoleKey.UpArrow || input.Key == ConsoleKey.DownArrow)
                {
                    switch (input.Key)
                    {
                        case ConsoleKey.UpArrow:
                            historyIndex--;
                            break;
                        case ConsoleKey.DownArrow:
                            historyIndex++;
                            break;
                    }

                    if (historyIndex > _history.Count - 1)
                    {
                        historyIndex = _history.Count - 1;
                    }

                    if (historyIndex < 0)
                    {
                        historyIndex = 0;
                    }

                    if (_history.Any())
                    {
                        _builder.Clear();
                        _builder.Append(_history[historyIndex]);
                        WriteSuggestion(_history[historyIndex], matchingOn);
                    }
                }
                else
                {
                    if (input.Key == ConsoleKey.Backspace && currentInput.Length > 0)
                    {
                        var removeLength = 1;
                        if (input.Modifiers == ConsoleModifiers.Alt)
                        {
                            if (currentInput.EndsWith(" "))
                            {
                                removeLength = 1;
                            }
                            else
                            {
                                removeLength = currentInput.Length -
                                               currentInput.LastIndexOf(" ",
                                                   StringComparison.InvariantCultureIgnoreCase) - 1;
                            }
                        }

                        _builder.Remove(_builder.Length - removeLength, removeLength);
                        currentInput = currentInput.Remove(currentInput.Length - removeLength);

                        matchIndex = -1;
                        matchingOn = currentInput;
                        WriteSuggestion(currentInput, matchingOn);
                    }
                    else if (input.Key != ConsoleKey.Backspace)
                    {
                        var key = input.KeyChar;
                        _builder.Append(key);
                        currentInput = _builder.ToString();

                        matchIndex = -1;
                        matchingOn = currentInput;
                        WriteSuggestion(currentInput, _builder.ToString());
                    }
                }

                input = Console.ReadKey(intercept: true);
            }

            return Commit();
        }

        private void WriteSuggestion(string currentInput, string matchingOn)
        {
            lock (_consoleLock)
            {
                ClearCurrentLine();
                var suggestionText = GetSuggestionText(currentInput, matchingOn);

                if (!string.IsNullOrEmpty(suggestionText))
                {
                    var currentLine = Console.CursorTop;
                    Console.SetCursorPosition(Math.Max(10, Console.WindowWidth - suggestionText.Length - 1), currentLine);
                    Console.ForegroundColor = ConsoleColor.DarkGreen;
                    Console.Write(suggestionText);

                    Console.ResetColor();

                    Console.SetCursorPosition(0, currentLine);
                }

                WriteCaret();
                Console.Write(currentInput);
            }
        }

        private string GetSuggestionText(string currentInput, string matchingOn)
        {
            var charLimit = Math.Min(Console.WindowWidth - matchingOn.Length - 5, Console.WindowWidth / 2);

            var suggestions = _modules.Options.Where(d => IsMatch(d, matchingOn))
                .Select(LastOptionPart)
                .ToList();

            if (suggestions.Count == 1 && LastOptionPart(currentInput) == suggestions.First())
            {
                return string.Empty;
            }

            var suggestionText = string.Empty;

            foreach (var s in suggestions)
            {
                if ((suggestionText + s).Length <= charLimit)
                {
                    suggestionText += s.Trim() + " | ";
                }
                else
                {
                    suggestionText += "...";
                    break;
                }
            }

            suggestionText = suggestionText.Trim(' ', '|');

            return suggestionText;
        }

        private string LastOptionPart(string option)
        {
            return option.Substring(option.LastIndexOf(" ", StringComparison.Ordinal) > -1
                ? option.LastIndexOf(" ", StringComparison.Ordinal)
                : 0);
        }

        private void ClearCurrentLine()
        {
            var currentLine = Console.CursorTop;
            Console.SetCursorPosition(0, Console.CursorTop);
            Console.Write(new string(' ', Console.WindowWidth - 1));
            Console.SetCursorPosition(0, currentLine);
        }

        private void ReplaceLastInput()
        {
            var currentLine = Console.CursorTop - 1;
            Console.SetCursorPosition(0, Console.CursorTop - 1);
            Console.Write(new string(' ', Console.WindowWidth - 1));
            Console.SetCursorPosition(0, currentLine);
        }

        private bool IsMatch(string item, string matchingOn)
        {
            if (string.IsNullOrEmpty(matchingOn) || string.Equals(item, matchingOn, StringComparison.CurrentCultureIgnoreCase))
            {
                return false;
            }

            if (matchingOn.Split(' ').Length != item.Split(' ').Length)
            {
                return false;
            }

            return item != matchingOn && item.StartsWith(matchingOn, true, CultureInfo.InvariantCulture);
        }

        private string Commit()
        {
            var message = _builder.ToString();
            _history.Add(message);
            _builder.Clear();
            ResetCursor();
            ClearCurrentLine();
            Console.WriteLine(message);
            Console.ResetColor();
            return message;
        }

        public void ReceiveMessage(string message, MessageType messageType)
        {
            if (_modules.Mute(message))
            {
                return;
            }

            ClearCurrentLine();

            if (message.IsFlashProgressMessage() && _lastMessageReceived.IsFlashProgressMessage())
            {
                ReplaceLastInput();
            }

            _lastMessageReceived = message;

            if (messageType == MessageType.ReceiveMessage)
            {
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.Write($"{DateTime.Now:T}");

                Console.ForegroundColor = ConsoleColor.DarkCyan;
                Console.Write(": ");

                if (message.StartsWith("["))
                {
                    Console.ForegroundColor = ConsoleColor.DarkCyan;
                }
                else
                {
                    Console.ForegroundColor = ConsoleColor.White;
                }
                if (!_modules.TryInvokeMessageReceived(message))
                {
                    Console.WriteLine(message);
                }
            }
            else
            {
                ConsoleColor color;
                switch (messageType)
                {
                    case MessageType.Exception:
                        color = ConsoleColor.Red;
                        break;
                    case MessageType.Success:
                        color = ConsoleColor.Green;
                        break;
                    case MessageType.Warning:
                        color = ConsoleColor.DarkYellow;
                        break;
                    case MessageType.System:
                        color = ConsoleColor.DarkCyan;
                        break;
                    default:
                        color = ConsoleColor.White;
                        break;
                }

                Console.ForegroundColor = color;
                Console.WriteLine(message);
            }

            Console.ResetColor();
            WriteSuggestion(_builder.ToString(), _builder.ToString());
            Console.ResetColor();
        }
    }
}