using System.Linq;
using Newtonsoft.Json;

namespace RazorTerm.Data
{
    public class RazorBoardData : IRazorBoardData
    {
        [RazorData(Pattern = @"^\(D\) V1: (\d+\.?\d*)")]
        public decimal? Voltage { get; set; }

        [RazorData(Pattern = @"^\(D\) Movement: (\d+\.?\d*)")]
        public decimal? MovementSensor { get; set; }

        [RazorData(Pattern = @"^\(D\) Movement Verdict: (.*)")]
        public string Movement { get; set; }

        [RazorData(Pattern = @"^\(D\) Charger Connected: (\d+)", ParseType = typeof(int))]
        public bool? ChargerConnected { get; set; }

        [RazorData(Pattern = @"^\(D\) Battery Fully Charged: (\d+)", ParseType = typeof(int))]
        public bool? BatteryFullyCharged { get; set; }

        [RazorData(Pattern = @"^\(D\) Docked: (\d+)", ParseType = typeof(int))]
        public bool? Docked { get; set; }

        [RazorData(Pattern = @"^\(D\) M1: (\d+\.?\d*)", Peak = 5)]
        public decimal? M1Current { get; set; }

        [RazorData(Pattern = @"^\(D\) M2: (\d+\.?\d*)", Peak = 5)]
        public decimal? M2Current { get; set; }

        [RazorData(Pattern = @"^\(D\) C1: (\d+\.?\d*)", Peak = 5)]
        public decimal? C1Current { get; set; }

        [RazorData(Pattern = @"^\(D\) Security (.*)")]
        public string Security { get; set; }

        [RazorData(Pattern = @"^\(D\) State (.*)")]
        public string State { get; set; }

        [RazorData(Pattern = @"^\(D\) Guide tracking: (\d+)", ParseType = typeof(int))]
        public bool? PerimeterTracking { get; set; }

        [RazorData(Pattern = @"^\(D\) Guide tracking active: (\d+)", ParseType = typeof(int))]
        public bool? PerimeterTrackingActive { get; set; }

        [RazorData(Pattern = @"^\(D\) Docking Station Locked: (\d+)", ParseType = typeof(int))]
        public bool? DockingLocked { get; set; }

        [RazorData(Pattern = @"^\(D\) Enabled: (\d+)", ParseType = typeof(int))]
        public bool? Enabled { get; set; }

        // Roll/Pitch/Yaw
        [RazorData(Pattern = @"^\(D\) Roll:\s*(\d+\.?\d*)")]
        public decimal? Roll { get; set; }

        [RazorData(Pattern = @"^\(D\) Roll:\s*\d+\.?\d*\s?Pitch:\s*(\d+\.?\d*)")]
        public decimal? Pitch { get; set; }

        [RazorData(Pattern = @"^\(D\) Roll:(?:.*?) Yaw: +(\d+)")]
        public int? Yaw { get; set; }

        // Boundary
        [RazorData(Pattern = @"^\(D\) IN-> BWF1: (\d+) BWF2: \d+ BWF3: \d+", References = 3)]
        public int? BWF1BoundaryIn { get; set; }

        [RazorData(Pattern = @"^\(D\) IN-> BWF1: \d+ BWF2: (\d+) BWF3: \d+", References = 3)]
        public int? BWF2BoundaryIn { get; set; }

        [RazorData(Pattern = @"^\(D\) IN-> BWF1: \d+ BWF2: \d+ BWF3: (\d+)", References = 3)]
        public int? BWF3BoundaryIn { get; set; }

        [RazorData(Pattern = @"^\(D\) OUT-> BWF1: (\d+) BWF2: \d+ BWF3: \d+", References = 3)]
        public int? BWF1BoundaryOut { get; set; }

        [RazorData(Pattern = @"^\(D\) OUT-> BWF1: \d+ BWF2: (\d+) BWF3: \d+", References = 3)]
        public int? BWF2BoundaryOut { get; set; }

        [RazorData(Pattern = @"^\(D\) OUT-> BWF1: \d+ BWF2: \d+ BWF3: (\d+)", References = 3)]
        public int? BWF3BoundaryOut { get; set; }

        // Guide
        [RazorData(Pattern = @"^\(D\) IN_GUIDE-> BWF1: (\d+) BWF2: \d+", References = 2)]
        public int? BWFGuide1In { get; set; }

        [RazorData(Pattern = @"^\(D\) IN_GUIDE-> BWF1: \d+ BWF2: (\d+)", References = 2)]
        public int? BWFGuide2In { get; set; }

        [RazorData(Pattern = @"^\(D\) OUT_GUIDE-> BWF1: (\d+) BWF2: \d+", References = 2)]
        public int? BWF1GuideOut { get; set; }

        [RazorData(Pattern = @"^\(D\) OUT_GUIDE-> BWF1: \d+ BWF2: (\d+)", References = 2)]
        public int? BWF2GuideOut { get; set; }

        // Magnitude
        [RazorData(Pattern = @"^\(D\) Magnitude -> BWF1: (\d+\.?\d*) BWF2: (?:\d+\.?\d*)", References = 2)]
        public decimal? BWF1BoundaryMag { get; set; }

        [RazorData(Pattern = @"^\(D\) Magnitude -> BWF1: (?:\d+\.?\d*) BWF2: (\d+\.?\d*)", References = 2)]
        public decimal? BWF2BoundaryMag { get; set; }

        [RazorData(Pattern = @"^\(D\) GuideMgmt -> BWF1: (\d+\.?\d*) BWF2: (?:\d+\.?\d*)", References = 2)]
        public decimal? BWF1GuideMag { get; set; }

        [RazorData(Pattern = @"^\(D\) GuideMgmt -> BWF1: (?:\d+\.?\d*) BWF2: (\d+\.?\d*)", References = 2)]
        public decimal? BWF2GuideMag { get; set; }

        [JsonIgnore]
        public bool IsComplete =>
            typeof(IRazorBoardData)
                .GetProperties()
                .All(p => p
                    .GetGetMethod()
                    .Invoke(this, null) != null);

        [JsonIgnore]
        public bool AnyData =>
            typeof(IRazorBoardData)
                .GetProperties()
                .Any(p => p
                    .GetGetMethod()
                    .Invoke(this, null) != null);
    }
}