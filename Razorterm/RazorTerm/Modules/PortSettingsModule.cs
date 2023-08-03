using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RazorTerm.Connection;

namespace RazorTerm.Modules
{
    public class PortSettingsModule : ITermModule
    {
        private SerialConnection _connection;
        public IDictionary<string, Action> Commands
        {
            get
            {
                var dict = new Dictionary<string, Action>();

                dict.Add("!connect", () => _connection.Connect());
                dict.Add("!disconnect", () => _connection.Disconnect());

                foreach (var port in SerialPort.GetPortNames())
                {
                    dict.Add("!connect " + port, () =>
                    {
                        _connection.Disconnect();
                        _connection.SerialPort.PortName = port;
                        _connection.Connect();
                    });

                    dict.Add("!term port " + port, () =>
                    {
                        var wasConnected = _connection.SerialPort.IsOpen;
                        _connection.Disconnect();
                        _connection.SerialPort.PortName = port;
                        if (wasConnected)
                        {
                            _connection.Connect();
                        }
                    });
                }

                foreach (var baudrate in new[] { 115200, 57600, 38400, 28800, 19200, 14400, 9600, 4800, 2400, 1200 })
                {
                    dict.Add("!term baudrate " + baudrate, () => _connection.SerialPort.BaudRate = baudrate );
                }

                foreach (var databits in new[] {5, 6, 7, 8})
                {
                    dict.Add("!term databits " + databits, () => _connection.SerialPort.DataBits = databits );
                }

                foreach (var stopbits in Enum.GetValues(typeof(StopBits)).Cast<StopBits>())
                {
                    dict.Add("!term stopbits " + stopbits, () => _connection.SerialPort.StopBits = stopbits );
                }

                foreach (var handshake in Enum.GetValues(typeof(Handshake)).Cast<Handshake>())
                {
                    dict.Add("!term handshake " + handshake, () => _connection.SerialPort.Handshake = handshake );
                }

                foreach (var parity in Enum.GetValues(typeof(Parity)).Cast<Parity>())
                {
                    dict.Add("!term parity " + parity, () => _connection.SerialPort.Parity = parity );
                }

                foreach (var encoding in new[] { Encoding.UTF8, Encoding.ASCII, Encoding.Unicode, Encoding.BigEndianUnicode, Encoding.UTF32 })
                {
                    dict.Add("!term encoding " + encoding.WebName, () => _connection.SetEncoding(encoding) );
                }

                return dict;
            }
        }

        public Task Init(IConnection connection)
        {
            _connection = connection as SerialConnection;
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
    }
}