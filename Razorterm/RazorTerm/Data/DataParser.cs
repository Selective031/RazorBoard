using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using RazorTerm.Logging;

namespace RazorTerm.Data
{
    public class DataParser
    {
        private class DataProp
        {
            public PropertyInfo PropertyInfo { get; set; }
            public Regex Regex { get; set; }
            public Type ParseType { get; set; }
            public int References { get; set; }
            public decimal? Peak { get; set; }
        }

        private RazorBoardData _data;
        private List<DataProp> _props = new List<DataProp>();
        private readonly object _lock = new object();
        public DataParser()
        {
            _data = new RazorBoardData();

            var props = typeof(RazorBoardData)
                .GetProperties();

            foreach (var prop in props)
            {
                var attr = prop.GetCustomAttributes(typeof(RazorDataAttribute), true).FirstOrDefault() as RazorDataAttribute;
                if (attr == null)
                {
                    continue;
                }

                _props.Add(new DataProp
                {
                    PropertyInfo = prop,
                    Regex = new Regex(attr.Pattern),
                    ParseType = attr.ParseType,
                    References = attr.References,
                    Peak = attr.Peak > 0 ? (decimal)attr.Peak : null,
                });
            }

        }

        public void Parse(string str)
        {
            lock (_lock)
            {
                try
                {
                    var matches = 0;
                    foreach (var prop in _props)
                    {
                        if (TryParse(str, prop.Regex, out var matchedValue))
                        {
                            var parseValue = prop.ParseType != null
                                ? Convert.ChangeType(matchedValue, prop.ParseType)
                                : matchedValue;

                            var value = Convert.ChangeType(parseValue, Nullable.GetUnderlyingType(prop.PropertyInfo.PropertyType) ?? prop.PropertyInfo.PropertyType);
                            if (prop.Peak == null || value is not decimal decimalValue || decimalValue < prop.Peak)
                            {
                                prop.PropertyInfo.GetSetMethod().Invoke(_data, new [] { value });
                                matches++;
                            }
                            else
                            {
                                Logger.Log($"Ignored peak value {value} for {str}");
                            }

                            if (matches >= prop.References)
                            {
                                return;
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    Logger.Log(e, LogLevel.Debug);
                }
            }
        }

        public bool IsComplete
        {
            get
            {
                lock (_lock)
                {
                    return _data.IsComplete;
                }
            }
        }

        public bool AnyData
        {
            get
            {
                lock (_lock)
                {
                    return _data.AnyData;
                }
            }
        }

        public IRazorBoardData ReadAndReset()
        {
            lock (_lock)
            {
                var data = _data;
                _data = new RazorBoardData();
                return data;
            }
        }

        private bool TryParse(string str, Regex regex, out string matchedValue)
        {
            var match = regex.Match(str);
            if (match.Success)
            {
                matchedValue = match.Groups[1].Value;
                return true;
            }
            else
            {
                matchedValue = default;
                return false;
            }
        }
    }
}