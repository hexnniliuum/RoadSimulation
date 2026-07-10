using UnityEngine;

/// <summary>
/// Opt-in console diagnostics for abrupt car stops; filter the console for "[RBDIAG]".
/// Stops below MinReportedSpeed are routine queue shuffling, not regressions, and stay silent.
/// </summary>
public static class BrakeDiagnostics
{
    public static bool Enabled = false;

    private const float MinReportedSpeed = 5f;

    /// <summary>
    /// A car zeroed its speed via the last-resort emergency brake while moving.
    /// </summary>
    public static void LogEmergencyStop(CarController car, CarController leader, float centerGap)
    {
        if (!Enabled || car.CurrentSpeed <= MinReportedSpeed)
            return;

        Debug.LogWarning(
            $"[RBDIAG] emergency stop car{car.CarID} v={car.CurrentSpeed:F1} path={car.CurrentPathType} " +
            $"curve={CurveKind(car)} state={car.CurrentCarState} gap={centerGap:F1} " +
            $"leader=car{leader.CarID} leaderPath={leader.CurrentPathType} leaderCurve={CurveKind(leader)} " +
            $"leaderV={leader.CurrentSpeed:F1} leaderState={leader.CurrentCarState}");
    }

    /// <summary>
    /// A car hit the stop/yield line still carrying speed and was halted there.
    /// </summary>
    public static void LogLineStop(CarController car, Node intersection)
    {
        if (!Enabled || car.CurrentSpeed <= MinReportedSpeed)
            return;

        Debug.LogWarning(
            $"[RBDIAG] line stop car{car.CarID} v={car.CurrentSpeed:F1} node={intersection?.ID} " +
            $"roundabout={intersection?.IsRoundabout}");
    }

    private static string CurveKind(CarController car)
    {
        return car.CurrentTurnCurve is RoundaboutTurnCurve ? "ring" : "plain";
    }
}
