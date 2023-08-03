using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace RazorTerm.Data
{
    public class RazorBoardCommand
    {
        public static string Stop => "DISABLE";
        public static string Start => "ENABLE";
        public static string Dock => $"TRACK PERIMETER";
        public static string Undock => $"UNDOCK";

        public static string Parse(string command)
        {
            switch (command)
            {
                case "Stop":
                    return RazorBoardCommand.Stop;
                case "Start":
                    return RazorBoardCommand.Start;
                case "Dock":
                    return RazorBoardCommand.Dock;
                case "Undock":
                    return RazorBoardCommand.Undock;
            }

            return null;
        }
    }
}