using UnityEngine;

[CreateAssetMenu(fileName = "RoadGenerationConfig", menuName = "Traffic Simulation/Road Generation Config")]
public class RoadGenerationConfig : ScriptableObject
{
    [Header("Grid Settings")]
    [Tooltip("Number of intersections horizontally (increase for high jitter values)")]
    public int GridWidth = 4;
    
    [Tooltip("Number of intersections vertically (increase for high jitter values)")]
    public int GridHeight = 3;

    [Header("Segment Length (Random Range)")]
    [Tooltip("Minimum length of a road segment")]
    public float MinSegmentLength = 30f;
    
    [Tooltip("Maximum length of a road segment")]
    public float MaxSegmentLength = 50f;

    [Header("Intersection Jitter")]
    [Tooltip("Maximum random offset applied to intersection positions (set to 0 for perfect grid)")]
    public float IntersectionJitter = 3f;

    [Header("Lane Settings")]
    [Tooltip("Width of each lane")]
    public float LaneWidth = 3.5f;
    
    [Tooltip("Distance from intersection center where lanes stop (stop line)")]
    public float StopLineDistance = 4f;
    
    [Tooltip("Radius for turn curves at intersections")]
    public float TurnRadius = 5f;

    [Header("Network Randomization")]
    [Tooltip("Percentage of non-border nodes to randomly remove (0-50%)")]
    [Range(0f, 50f)]
    public float NodeRemovalPercentage = 15f;
    
    [Tooltip("Percentage of non-essential roads to randomly remove (0-30%)")]
    [Range(0f, 30f)]
    public float RoadRemovalPercentage = 10f;

    [Header("Bridge Settings")]
    [Tooltip("Enable bridges for crossing roads")]
    public bool EnableBridges = true;
    
    [Tooltip("Height of bridge at crossing point")]
    public float BridgeHeight = 5f;
    
    [Tooltip("Minimum ramp length for smooth bridge approach (each side)")]
    public float MinBridgeRampLength = 8f;
    
    [Tooltip("Minimum distance from intersection to allow bridge")]
    public float MinDistanceFromIntersection = 12f;
    
    [Tooltip("Minimum angle between crossing roads to allow bridge (degrees)")]
    [Range(30f, 90f)]
    public float MinBridgeCrossingAngle = 35f;
    
    [Tooltip("Percentage chance to add diagonal/shortcut connections (0-30%)")]
    [Range(0f, 30f)]
    public float DiagonalConnectionPercentage = 15f;

    [Header("Random Seed")]
    [Tooltip("Use fixed seed for reproducible generation (0 = random)")]
    public int RandomSeed = 0;

    [Header("Y-Axis Position (Floor Height)")]
    [Tooltip("Constant Y position for all road elements")]
    public float FloorHeight = 0f;

    [Header("Intersection Logic")]
    [Tooltip("Percentage of eligible intersections (3+ roads) to have traffic lights (0-100%)")]
    [Range(0f, 100f)]
    public float TrafficLightPercentage = 100f;

    [Tooltip("Distance from lane end to start checking for intersection")]
    public float IntersectionDetectionDistance = 25f;
    
    [Tooltip("Buffer distance before stop line (prevents overshooting)")]
    public float StopLineBuffer = 0.5f;
    
    [Tooltip("Time to wait before forcing entry to break deadlock (seconds)")]
    public float DeadlockTimeout = 5f;

    private void OnValidate()
    {
        // Ensure at least 4 intersections (minimum 2x2 grid)
        GridWidth = Mathf.Max(2, GridWidth);
        GridHeight = Mathf.Max(2, GridHeight);
        MinSegmentLength = Mathf.Max(5f, MinSegmentLength);
        MaxSegmentLength = Mathf.Max(MinSegmentLength, MaxSegmentLength);
        LaneWidth = Mathf.Max(1f, LaneWidth);
        TurnRadius = Mathf.Max(1f, TurnRadius);
    }
}

