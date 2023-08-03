using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using RazorTerm.Connection;
using RazorTerm.Logging;

namespace RazorTerm.Modules
{
    public class UpdateModule : ITermModule
    {
        private bool _upgradeCommandSent;
        public IDictionary<string, Action> Commands => new Dictionary<string, Action>
        {
            { "UPGRADE AND FLASH", BeginUpgrade },
            { "PERFORM UPGRADE", PerformUpgrade },
            { "FIX BOOT", FixBoot },
            { "ENTER BOOTLOADER", EnterBootloader },
            { "UPGRADE", UpgradeDisabledNotice }
        };

        private void UpgradeDisabledNotice()
        {
            Logger.Log("Upgrade command disabled - use ENTER BOOTLOADER or UPGRADE AND FLASH");
        }

        private void EnterBootloader()
        {
            _connection.SendMessage("UPGRADE");
        }

        private IConnection _connection;
        public Task Init(IConnection connection)
        {
            _connection = connection;
            _connection.MessageReceived += ConnectionOnMessageReceived;
            return Task.CompletedTask;
        }

        public bool ParseCommand(string command)
        {
            return false;
        }

        public bool ParseReceived(string command)
        {
            return false;
        }

        public bool Mute(string command)
        {
            return false;
        }

        private void ConnectionOnMessageReceived(string message, MessageType _)
        {
            if (message.Contains("Entering Bootloader") && _upgradeCommandSent)
            {
                _upgradeCommandSent = false;
                PerformUpgrade();
            }
        }

        private async void FixBoot()
        {
            await Task.Delay(1000);
            await _connection.Disconnect();
            await Task.Delay(1000);
            ExecuteCommand("stm32flash -g 0x8000000 /dev/ttyUSB0");
            Logger.Log("Boot command finished");
            await Task.Delay(1000);
            await _connection.Connect();
        }

        private async void BeginUpgrade()
        {
            _upgradeCommandSent = true;
            await _connection.SendMessage("DEBUG OFF");
            await Task.Delay(500);
            await _connection.SendMessage("DISABLE");
            await Task.Delay(2000);
            await _connection.SendMessage("UPGRADE");

            // Wait for Entering Bootloader command before performing upgrade
        }

        private async void PerformUpgrade()
        {
            await Task.Delay(1000);
            await _connection.Disconnect();
            await Task.Delay(1000);
            ExecuteCommand("stm32flash -w /home/pi/RazorBoard.bin -v -g 0x8000000 /dev/ttyUSB0");
            Logger.Log("Flush command finished");
            await Task.Delay(2000);
            await _connection.Connect();
        }

        public static void ExecuteCommand(string command)
        {
            Logger.Log(command);

            var proc = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = "-c \""+ command + "\"",
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    CreateNoWindow = true
                }
            };

            proc.Start();
            Logger.Log("Waiting for command to finish...");

            while (!proc.StandardOutput.EndOfStream)
            {
                var line = proc.StandardOutput.ReadLine();
                Logger.Log(line);
            }

            proc.WaitForExit();
            Logger.Log("Command finished");
            //
            // var errorOutput = proc.StandardError.ReadToEnd();
            // if (!string.IsNullOrEmpty(errorOutput))
            // {
            //     Logger.Log(errorOutput, LogLevel.Warning);
            // }
            //
            // var output = proc.StandardOutput.ReadToEnd();
            // Logger.Log(output);
        }
    }
}