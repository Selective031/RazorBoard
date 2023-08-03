using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using RazorTerm.Connection;

namespace RazorTerm.Logging
{
    public class Logger : ServiceBase
    {
        private const string LogFile = "razorterm.log";
        public static bool CreateLogFileIfNotExists => Settings.Service.Enabled;
        private const long MaxFileSize = 2 * 1024 * 1024; //2 MB
        private const bool LogToConsole = true;
        private static readonly object Lock = new object();
        private static long _loggedRowsSinceTrim = 0;
        public static event MessageReceivedDelegate EventLogged;
        private static Queue<string> Buffer = new Queue<string>();
        private static DateTime LastWriteTime = DateTime.Now;
        private static bool Enabled = true;

        public static void Log(string message, LogLevel logLevel = LogLevel.Info)
        {
            var text = $"{DateTime.Now:s} [{logLevel}] {message}".Trim();

            lock (Lock)
            {
                if (LogToConsole && (int)logLevel > (int)LogLevel.Debug)
                {
                    EventLogged?.Invoke($"[{logLevel}] {message}", MessageType.System);
                }

                if (Enabled)
                {
                    Buffer.Enqueue(text);
                }
            }
        }

        public static void Log(Exception e, LogLevel logLevel = LogLevel.Critical)
        {
            Log($"{e.GetType().Name} {e.Message}\n{e.StackTrace}", logLevel);
        }

        private static void CheckFileSizeAndTrim()
        {
            if (new FileInfo(LogFile).Length > MaxFileSize)
            {
                var logLines = File.ReadAllLines(LogFile);
                File.WriteAllText(LogFile, string.Join("\n", logLines.Skip(logLines.Length / 4)));
            }
        }

        protected override async Task OnStart()
        {
            if (File.Exists(LogFile))
            {
                return;
            }

            if (CreateLogFileIfNotExists)
            {
                using (var writer = File.CreateText(LogFile))
                {
                    await writer.WriteLineAsync("Log file created");
                }
            }
            else
            {
                Enabled = false;
            }
        }

        protected override async Task Run()
        {
            while (Running)
            {
                await Task.Delay(100);
                WriteBuffer();
            }
        }

        private static void WriteBuffer(bool force = false)
        {
            lock (Lock)
            {
                var write = force
                            || DateTime.Now - LastWriteTime > TimeSpan.FromSeconds(20)
                            || Buffer.Count > 15;
                if (write)
                {
                    var lines = new List<string>();
                    while (Buffer.Count > 0)
                    {
                        lines.Add(Buffer.Dequeue());
                    }

                    foreach (var line in lines.Distinct().ToList())
                    {
                        var occurrences = lines.Count(l => l == line);
                        if (occurrences > 1)
                        {
                            var index = lines.IndexOf(line);
                            lines.RemoveAll(l => l == line);
                            lines.Insert(index, $"{line} ({occurrences} times)");
                        }
                    }

                    File.AppendAllLines(LogFile, lines);
                    LastWriteTime = DateTime.Now;

                    _loggedRowsSinceTrim += lines.Count;
                    if (_loggedRowsSinceTrim > 200)
                    {
                        _loggedRowsSinceTrim = 0;
                        CheckFileSizeAndTrim();
                    }
                }
            }
        }

        protected override Task OnStop()
        {
            WriteBuffer(true);
            return Task.CompletedTask;
        }
    }
}