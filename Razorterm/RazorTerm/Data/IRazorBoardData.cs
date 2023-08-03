namespace RazorTerm.Data
{
    public interface IRazorBoardData
    {
        decimal? Voltage { get; }
        decimal? MovementSensor { get; }
        string Movement { get; }
        bool? ChargerConnected { get; }
        bool? BatteryFullyCharged { get; }
        bool? Docked { get; }
        decimal? M1Current { get; }
        decimal? M2Current { get; }
        decimal? C1Current { get; }
        string Security { get; }
        string State { get; }
        bool? PerimeterTracking { get; }
        bool? PerimeterTrackingActive { get; }
        bool? DockingLocked { get; }
        bool? Enabled { get; }

        // Roll/Pitch/Yaw
        decimal? Roll { get; }
        decimal? Pitch { get; }
        int? Yaw { get; }

        // Boundary
        int? BWF1BoundaryIn { get; }
        int? BWF2BoundaryIn { get; }
        int? BWF3BoundaryIn { get; }
        int? BWF1BoundaryOut { get; }
        int? BWF2BoundaryOut { get; }
        int? BWF3BoundaryOut { get; }

        // Guide
        int? BWFGuide1In { get; }
        int? BWFGuide2In { get; }
        int? BWF1GuideOut { get; }
        int? BWF2GuideOut { get; }

        // Magnitude
        decimal? BWF1BoundaryMag { get; }
        decimal? BWF2BoundaryMag { get; }
        decimal? BWF1GuideMag { get; }
        decimal? BWF2GuideMag { get; }
    }
}