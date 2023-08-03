namespace RazorTerm
{
    public static class RazorApplication
    {
        private static bool _stopped;
        public static bool Stopped => _stopped;
        public static bool Stop() => _stopped = true;
    }
}