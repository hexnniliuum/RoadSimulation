using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Represents an intersection or connection point in the road network.
/// </summary>
[System.Serializable]
public class Node
{
    public int ID;
    public Vector3 Position; // Y will be constant (floor height)
    public List<Road> ConnectedRoads = new List<Road>();

    public Node(int id, Vector3 position)
    {
        ID = id;
        Position = position;
    }
}

/// <summary>
/// Temporary structure for road topology before geometry creation.
/// </summary>
public struct RoadConnection
{
    public int ID;
    public Node StartNode;
    public Node EndNode;
    public float Length;
    public List<BridgeInfo> Bridges; // List of bridges if road crosses over other roads

    public RoadConnection(int id, Node start, Node end)
    {
        ID = id;
        StartNode = start;
        EndNode = end;
        Length = Vector3.Distance(start.Position, end.Position);
        Bridges = new List<BridgeInfo>();
    }
}

/// <summary>
/// Information about a bridge segment where a road crosses over another.
/// </summary>
[System.Serializable]
public class BridgeInfo
{
    public Vector3 CrossingPoint; // XZ position where roads cross
    public float StartDistance; // Distance from road start where bridge begins
    public float EndDistance; // Distance from road start where bridge ends
    public float PeakHeight; // Maximum elevation at crossing point
    public float RampLength; // Length of the approach ramp on each side
    
    public BridgeInfo(Vector3 crossingPoint, float startDist, float endDist, float peakHeight, float rampLength)
    {
        CrossingPoint = crossingPoint;
        StartDistance = startDist;
        EndDistance = endDist;
        PeakHeight = peakHeight;
        RampLength = rampLength;
    }
    
    // Calculate vertical offset at a given distance along the road
    public float GetElevationAt(float distance)
    {
        if (distance < StartDistance || distance > EndDistance)
            return 0f; // Flat ground level
        
        // Use smooth parabolic curve
        float bridgeProgress = (distance - StartDistance) / (EndDistance - StartDistance);
        
        // Parabolic curve: peaks at 0.5, returns to 0 at edges
        float curveValue = 4f * bridgeProgress * (1f - bridgeProgress);
        
        return curveValue * PeakHeight;
    }
}

/// <summary>
/// Represents a road connecting two nodes with two lanes (one in each direction).
/// </summary>
[System.Serializable]
public class Road
{
    public int ID;
    public Node StartNode;
    public Node EndNode;
    public Lane ForwardLane;  // StartNode -> EndNode
    public Lane BackwardLane; // EndNode -> StartNode
    public float Length;
    public List<BridgeInfo> Bridges; // List of bridges if road crosses over other roads

    public Road(int id, Node start, Node end, float laneWidth, float stopLineDistanceStart, float stopLineDistanceEnd, List<BridgeInfo> bridges = null)
    {
        ID = id;
        StartNode = start;
        EndNode = end;
        Length = Vector3.Distance(start.Position, end.Position);
        
        // Create lanes offset from centerline (on XZ plane)
        Vector3 direction = (end.Position - start.Position).normalized;
        Vector3 perpendicular = new Vector3(direction.z, 0, -direction.x); // Left perpendicular for right-side driving
        float offset = laneWidth * 0.5f;

        // Calculate lane start/end positions with variable stop line distances
        // (adjusted based on angle between roads at each intersection)
        Vector3 forwardStart = start.Position + direction * stopLineDistanceStart + perpendicular * offset;
        Vector3 forwardEnd = end.Position - direction * stopLineDistanceEnd + perpendicular * offset;
        Vector3 backwardStart = end.Position - direction * stopLineDistanceEnd - perpendicular * offset;
        Vector3 backwardEnd = start.Position + direction * stopLineDistanceStart - perpendicular * offset;

        Bridges = bridges ?? new List<BridgeInfo>();

        ForwardLane = new Lane(id * 2, start, end, forwardStart, forwardEnd, Bridges, isReversed: false, roadLength: Length, stopLineDistanceAtStart: stopLineDistanceStart);
        BackwardLane = new Lane(id * 2 + 1, end, start, backwardStart, backwardEnd, Bridges, isReversed: true, roadLength: Length, stopLineDistanceAtStart: stopLineDistanceEnd);
    }
}

/// <summary>
/// Represents a single lane where cars travel in one direction.
/// </summary>
[System.Serializable]
public class Lane
{
    public int ID;
    public Node StartNode;
    public Node EndNode;
    public Vector3 StartPosition;
    public Vector3 EndPosition;
    public float Length;
    public List<BridgeInfo> Bridges;
    public List<TurnCurve> AvailableTurns = new List<TurnCurve>();

    public Lane(int id, Node start, Node end, Vector3 startPos, Vector3 endPos, List<BridgeInfo> bridges = null, bool isReversed = false, float roadLength = 0f, float stopLineDistanceAtStart = 0f)
    {
        ID = id;
        StartNode = start;
        EndNode = end;
        StartPosition = startPos;
        EndPosition = endPos;
        Bridges = bridges ?? new List<BridgeInfo>();
        IsReversed = isReversed;
        RoadLength = roadLength;
        StopLineDistanceAtStart = stopLineDistanceAtStart;
        
        // Calculate actual 3D arc length (accounts for bridge elevations)
        Length = Calculate3DArcLength();
    }
    
    /// <summary>
    /// Calculate the actual 3D arc length of this lane path, including bridge elevations.
    /// Uses numerical integration for accuracy.
    /// </summary>
    private float Calculate3DArcLength()
    {
        // Start with horizontal distance as base
        float horizontalLength = Vector3.Distance(StartPosition, EndPosition);
        
        // If no bridges, horizontal length IS the 3D length
        if (Bridges == null || Bridges.Count == 0)
        {
            return horizontalLength;
        }
        
        // Numerical integration: sample the path and sum segment lengths
        int samples = 100; // Good balance between accuracy and performance
        float totalLength = 0f;
        Vector3 previousPos = StartPosition; // Start at Y=0
        
        for (int i = 1; i <= samples; i++)
        {
            float t = i / (float)samples;
            float distance = t * horizontalLength;
            
            // Get position at this distance (includes bridge elevation)
            Vector3 basePos = Vector3.Lerp(StartPosition, EndPosition, t);
            
            // Apply bridge elevations
            float distanceFromRoadStart = IsReversed 
                ? (RoadLength - StopLineDistanceAtStart - distance)
                : (StopLineDistanceAtStart + distance);
            
            float totalElevation = 0f;
            foreach (var bridge in Bridges)
            {
                totalElevation += bridge.GetElevationAt(distanceFromRoadStart);
            }
            basePos.y += totalElevation;
            
            // Add segment length
            totalLength += Vector3.Distance(previousPos, basePos);
            previousPos = basePos;
        }
        
        return totalLength;
    }
    
    public bool IsReversed; // True if this lane runs opposite to the road direction
    public float RoadLength; // Total road length (needed for reversed lane bridge calculations)
    public float StopLineDistanceAtStart; // Distance from road start where this lane begins

    public Vector3 GetPositionAtDistance(float distance)
    {
        // Clamp distance to valid range
        distance = Mathf.Clamp(distance, 0f, Length);
        
        // If no bridges, simple linear interpolation works fine
        if (Bridges == null || Bridges.Count == 0)
        {
            float tParam = distance / Length;
            return Vector3.Lerp(StartPosition, EndPosition, tParam);
        }
        
        // With bridges, we need to find the correct position along the 3D arc
        // Use binary search to find t such that 3D arc length equals desired distance
        float horizontalLength = Vector3.Distance(StartPosition, EndPosition);
        float t = FindTForArcDistance(distance, horizontalLength);
        
        Vector3 basePosition = Vector3.Lerp(StartPosition, EndPosition, t);
        
        // Apply bridge elevations
        float horizontalDistance = t * horizontalLength;
        float distanceFromRoadStart = IsReversed 
            ? (RoadLength - StopLineDistanceAtStart - horizontalDistance)
            : (StopLineDistanceAtStart + horizontalDistance);
        
        float totalElevation = 0f;
        foreach (var bridge in Bridges)
        {
            totalElevation += bridge.GetElevationAt(distanceFromRoadStart);
        }
        basePosition.y += totalElevation;
        
        return basePosition;
    }
    
    /// <summary>
    /// Find the parametric t value that corresponds to a given 3D arc distance along the lane.
    /// Uses binary search for efficiency and accuracy.
    /// </summary>
    private float FindTForArcDistance(float targetDistance, float horizontalLength)
    {
        if (targetDistance <= 0f)
            return 0f;
        if (targetDistance >= Length)
            return 1f;
        
        // Binary search to find correct t
        float tMin = 0f;
        float tMax = 1f;
        int maxIterations = 20; // Sufficient for high accuracy
        
        for (int i = 0; i < maxIterations; i++)
        {
            float tMid = (tMin + tMax) * 0.5f;
            float arcLengthAtMid = Calculate3DArcLengthUpToT(tMid, horizontalLength);
            
            if (Mathf.Abs(arcLengthAtMid - targetDistance) < 0.01f) // Tolerance: 1cm
            {
                return tMid;
            }
            
            if (arcLengthAtMid < targetDistance)
            {
                tMin = tMid;
            }
            else
            {
                tMax = tMid;
            }
        }
        
        return (tMin + tMax) * 0.5f;
    }
    
    /// <summary>
    /// Calculate the 3D arc length from the start of the lane up to parameter t.
    /// </summary>
    private float Calculate3DArcLengthUpToT(float t, float horizontalLength)
    {
        int samples = 20; // Fewer samples for real-time calculation
        float totalLength = 0f;
        Vector3 previousPos = StartPosition;
        
        for (int i = 1; i <= samples; i++)
        {
            float currentT = (i / (float)samples) * t;
            float distance = currentT * horizontalLength;
            
            Vector3 basePos = Vector3.Lerp(StartPosition, EndPosition, currentT);
            
            float distanceFromRoadStart = IsReversed 
                ? (RoadLength - StopLineDistanceAtStart - distance)
                : (StopLineDistanceAtStart + distance);
            
            float totalElevation = 0f;
            foreach (var bridge in Bridges)
            {
                totalElevation += bridge.GetElevationAt(distanceFromRoadStart);
            }
            basePos.y += totalElevation;
            
            totalLength += Vector3.Distance(previousPos, basePos);
            previousPos = basePos;
        }
        
        return totalLength;
    }

    public Vector3 GetDirection()
    {
        return (EndPosition - StartPosition).normalized;
    }
}

/// <summary>
/// Represents a curved path for turning at intersections.
/// </summary>
[System.Serializable]
public class TurnCurve
{
    public Lane FromLane;
    public Lane ToLane;
    public Vector3 StartPoint;
    public Vector3 ControlPoint; // For quadratic Bezier
    public Vector3 EndPoint;
    public float ArcLength;
    public TurnType Type;

    public enum TurnType { Straight, Left, Right }

    public TurnCurve(Lane from, Lane to, float radius)
    {
        FromLane = from;
        ToLane = to;
        StartPoint = from.EndPosition;
        EndPoint = to.StartPosition;

        // Determine turn type (using Y component of cross product for XZ plane)
        Vector3 incomingDir = from.GetDirection();
        Vector3 outgoingDir = to.GetDirection();
        float crossProduct = Vector3.Cross(incomingDir, outgoingDir).y;
        
        if (Mathf.Abs(crossProduct) < 0.1f)
        {
            Type = TurnType.Straight;
            ControlPoint = Vector3.Lerp(StartPoint, EndPoint, 0.5f);
        }
        else if (crossProduct > 0)
        {
            Type = TurnType.Left;
            ControlPoint = CalculateControlPoint(StartPoint, EndPoint, incomingDir, outgoingDir, radius);
        }
        else
        {
            Type = TurnType.Right;
            ControlPoint = CalculateControlPoint(StartPoint, EndPoint, incomingDir, outgoingDir, radius);
        }

        ArcLength = EstimateArcLength();
    }

    private Vector3 CalculateControlPoint(Vector3 start, Vector3 end, Vector3 inDir, Vector3 outDir, float radius)
    {
        // For straight or nearly straight connections
        float dot = Vector3.Dot(inDir, outDir);
        if (dot > 0.95f) // Nearly parallel (less than ~18 degree turn)
        {
            return Vector3.Lerp(start, end, 0.5f);
        }
        
        // For very sharp angles (>140 degrees turn), use simple offset approach
        if (dot < -0.766f) // cos(140°) ≈ -0.766
        {
            // Sharp turn - use midpoint with small perpendicular offset
            Vector3 midpoint = (start + end) * 0.5f;
            Vector3 toEnd = (end - start).normalized;
            Vector3 perpendicular = new Vector3(toEnd.z, 0, -toEnd.x); // Right-side driving
            
            // Determine turn direction from cross product
            float cross = Vector3.Cross(inDir, outDir).y;
            float offsetAmount = Mathf.Min(radius, Vector3.Distance(start, end) * 0.3f);
            return midpoint + perpendicular * Mathf.Sign(cross) * offsetAmount;
        }
        
        // Calculate intersection point of the two lane directions
        Vector3 d1 = inDir;
        Vector3 d2 = -outDir; // Reverse to point backward from end
        
        // Find intersection of two rays on XZ plane
        // start + t1*d1 = end + t2*d2
        float denominator = d1.x * d2.z - d1.z * d2.x;
        
        if (Mathf.Abs(denominator) < 0.001f)
        {
            // Parallel or nearly parallel
            return Vector3.Lerp(start, end, 0.5f);
        }
        
        float t1 = ((end.x - start.x) * d2.z - (end.z - start.z) * d2.x) / denominator;
        float t2 = ((end.x - start.x) * d1.z - (end.z - start.z) * d1.x) / denominator;
        
        // Calculate intersection point
        Vector3 intersection = start + d1 * t1;
        
        // Clamp control point distance to prevent wild curves
        float maxDistFromStart = Vector3.Distance(start, end) * 2.0f;
        float distFromStart = Vector3.Distance(start, intersection);
        
        if (t1 < 0 || t2 < 0 || distFromStart > maxDistFromStart)
        {
            // Intersection too far or behind - use fallback
            Vector3 midpoint = (start + end) * 0.5f;
            Vector3 avgDir = (inDir + outDir).normalized;
            Vector3 perpendicular = new Vector3(avgDir.z, 0, -avgDir.x); // Right-side driving
            return midpoint + perpendicular * radius;
        }
        
        intersection.y = start.y; // Maintain floor height
        return intersection;
    }

    private float EstimateArcLength()
    {
        // Approximate arc length by sampling the curve
        int samples = 10;
        float length = 0f;
        Vector3 prevPoint = GetPointOnCurve(0f);
        
        for (int i = 1; i <= samples; i++)
        {
            float t = i / (float)samples;
            Vector3 point = GetPointOnCurve(t);
            length += Vector3.Distance(prevPoint, point);
            prevPoint = point;
        }
        
        return length;
    }

    public Vector3 GetPointOnCurve(float t)
    {
        // Quadratic Bezier curve
        t = Mathf.Clamp01(t);
        float u = 1f - t;
        return u * u * StartPoint + 2f * u * t * ControlPoint + t * t * EndPoint;
    }

    public Vector3 GetTangentOnCurve(float t)
    {
        // Derivative of quadratic Bezier
        t = Mathf.Clamp01(t);
        return (2f * (1f - t) * (ControlPoint - StartPoint) + 2f * t * (EndPoint - ControlPoint)).normalized;
    }
    
    /// <summary>
    /// Get position at normalized time along curve (alias for GetPointOnCurve).
    /// </summary>
    public Vector3 GetPositionAtTime(float t)
    {
        return GetPointOnCurve(t);
    }
    
    /// <summary>
    /// Get direction (tangent) at normalized time along curve (alias for GetTangentOnCurve).
    /// </summary>
    public Vector3 GetDirectionAtTime(float t)
    {
        return GetTangentOnCurve(t);
    }
}

/// <summary>
/// Container for the entire road network.
/// </summary>
public class RoadMap
{
    public List<Node> Nodes = new List<Node>();
    public List<Road> Roads = new List<Road>();
    public int RandomSeed;

    public void Clear()
    {
        Nodes.Clear();
        Roads.Clear();
    }

    public List<Lane> GetAllLanes()
    {
        List<Lane> lanes = new List<Lane>();
        foreach (var road in Roads)
        {
            lanes.Add(road.ForwardLane);
            lanes.Add(road.BackwardLane);
        }
        return lanes;
    }
}

// ============================================================================
// EXPORT DATA STRUCTURES FOR JSON SERIALIZATION
// ============================================================================

/// <summary>
/// Root export structure for JSON serialization.
/// Contains complete road network data for Python visualization.
/// </summary>
[System.Serializable]
public class RoadNetworkExport
{
    public int RandomSeed;
    public List<NodeExport> Nodes;
    public List<RoadExport> Roads;
    public List<LaneExport> Lanes;
    public List<BridgeExport> Bridges;
    public List<TurnCurveExport> TurnCurves;
}

[System.Serializable]
public class NodeExport
{
    public int ID;
    public float X, Y, Z;
    public List<int> ConnectedRoadIDs;
}

[System.Serializable]
public class RoadExport
{
    public int ID;
    public int StartNodeID;
    public int EndNodeID;
    public float Length;
    public int ForwardLaneID;
    public int BackwardLaneID;
}

[System.Serializable]
public class LaneExport
{
    public int ID;
    public int StartNodeID;
    public int EndNodeID;
    public float StartX, StartY, StartZ;
    public float EndX, EndY, EndZ;
    public float Length;
    public bool IsReversed;
    public float RoadLength;
    public float StopLineDistanceAtStart;
    public List<int> BridgeIndices;
    public List<int> AvailableTurnIDs;
}

[System.Serializable]
public class BridgeExport
{
    public float CrossingX, CrossingZ;
    public float StartDistance;
    public float EndDistance;
    public float PeakHeight;
    public float RampLength;
}

[System.Serializable]
public class TurnCurveExport
{
    public int ID;
    public int FromLaneID;
    public int ToLaneID;
    public float StartX, StartY, StartZ;
    public float ControlX, ControlY, ControlZ;
    public float EndX, EndY, EndZ;
    public float ArcLength;
    public string TurnType;
}

