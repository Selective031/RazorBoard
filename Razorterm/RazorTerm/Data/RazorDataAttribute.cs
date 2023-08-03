using System;

namespace RazorTerm.Data
{
    public class RazorDataAttribute : Attribute
    {
        public string Pattern { get; set; }
        public Type ParseType { get; set; }
        public int References { get; set; } = 1;
        public float Peak { get; set; }
    }
}