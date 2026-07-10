using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class Node
{
    public int ID;
    public Vector3 Position; // Y is constant (floor height)
    public List<Road> ConnectedRoads = new List<Road>();

    // Marks this node as a roundabout: turns become RoundaboutTurnCurve ring paths,
    // entry is arbitrated by ring occupancy instead of the turn-conflict map, and the
    // node is never signalized. Decided during generation, before lane geometry exists.
    public bool IsRoundabout;

    public Node(int id, Vector3 position)
    {
        ID = id;
        Position = position;
    }
    
    /// <summary>
    /// True when only two roads meet here - a straight-through continuation
    /// with no possible collision, not a real intersection.
    /// </summary>
    public bool IsTrivialIntersection()
    {
        return ConnectedRoads.Count == 2;
    }
}

/// <summary>
/// Transient topology data used before Road/Lane geometry is generated.
/// </summary>
public struct RoadConnection
{
    public int ID;
    public Node StartNode;
    public Node EndNode;
    public float Length;
    public List<BridgeInfo> Bridges;

    public RoadConnection(int id, Node start, Node end)
    {
        ID = id;
        StartNode = start;
        EndNode = end;
        Length = Vector3.Distance(start.Position, end.Position);
        Bridges = new List<BridgeInfo>();
    }
}

[System.Serializable]
public class BridgeInfo
{
    public Vector3 CrossingPoint; // XZ position where roads cross
    public float StartDistance;
    public float EndDistance;
    public float PeakHeight;
    public float RampLength; // Ramp length on each side of the crossing point
    
    public BridgeInfo(Vector3 crossingPoint, float startDist, float endDist, float peakHeight, float rampLength)
    {
        CrossingPoint = crossingPoint;
        StartDistance = startDist;
        EndDistance = endDist;
        PeakHeight = peakHeight;
        RampLength = rampLength;
    }
    
    public float GetElevationAt(float distance)
    {
        if (distance < StartDistance || distance > EndDistance)
            return 0f;
        
        float bridgeProgress = (distance - StartDistance) / (EndDistance - StartDistance);

        // Zero slope at both ends so the ramp tangent matches the flat road,
        // avoiding a kink where the ramp meets level ground.
        float curveValue = 0.5f * (1f - Mathf.Cos(2f * Mathf.PI * bridgeProgress));

        return curveValue * PeakHeight;
    }
}

/// <summary>
/// A road between two nodes, with one lane in each direction.
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
    public List<BridgeInfo> Bridges;

    public Road(int id, Node start, Node end, float laneWidth, float stopLineDistanceStart, float stopLineDistanceEnd, List<BridgeInfo> bridges = null)
    {
        ID = id;
        StartNode = start;
        EndNode = end;
        Length = Vector3.Distance(start.Position, end.Position);
        
        Vector3 direction = (end.Position - start.Position).normalized;
        Vector3 perpendicular = new Vector3(direction.z, 0, -direction.x); // Left perpendicular for right-side driving
        float offset = laneWidth * 0.5f;

        // Stop line distance varies with the angle between roads at the intersection
        Vector3 forwardStart = start.Position + direction * stopLineDistanceStart + perpendicular * offset;
        Vector3 forwardEnd = end.Position - direction * stopLineDistanceEnd + perpendicular * offset;
        Vector3 backwardStart = end.Position - direction * stopLineDistanceEnd - perpendicular * offset;
        Vector3 backwardEnd = start.Position + direction * stopLineDistanceStart - perpendicular * offset;

        Bridges = bridges ?? new List<BridgeInfo>();

        ForwardLane = new Lane(id * 2, start, end, forwardStart, forwardEnd, Bridges, isReversed: false, roadLength: Length, stopLineDistanceAtStart: stopLineDistanceStart);
        BackwardLane = new Lane(id * 2 + 1, end, start, backwardStart, backwardEnd, Bridges, isReversed: true, roadLength: Length, stopLineDistanceAtStart: stopLineDistanceEnd);
    }
}

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
        
        Length = Calculate3DArcLength();
    }
    
    /// <summary>
    /// 3D arc length of the lane, accounting for bridge elevation via numerical integration.
    /// </summary>
    private float Calculate3DArcLength()
    {
        float horizontalLength = Vector3.Distance(StartPosition, EndPosition);
        
        if (Bridges == null || Bridges.Count == 0)
        {
            return horizontalLength;
        }
        
        int samples = 100; // Good balance between accuracy and performance
        float totalLength = 0f;
        Vector3 previousPos = StartPosition;
        
        for (int i = 1; i <= samples; i++)
        {
            float t = i / (float)samples;
            float distance = t * horizontalLength;
            
            Vector3 basePos = Vector3.Lerp(StartPosition, EndPosition, t);
            
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
    
    public bool IsReversed;
    public float RoadLength; // Needed for reversed-lane bridge distance calculations
    public float StopLineDistanceAtStart;

    public Vector3 GetPositionAtDistance(float distance)
    {
        distance = Mathf.Clamp(distance, 0f, Length);
        
        if (Bridges == null || Bridges.Count == 0)
        {
            float tParam = distance / Length;
            return Vector3.Lerp(StartPosition, EndPosition, tParam);
        }
        
        // Bridges make position nonlinear in t, so binary-search for the t matching this arc distance
        float horizontalLength = Vector3.Distance(StartPosition, EndPosition);
        float t = FindTForArcDistance(distance, horizontalLength);
        
        Vector3 basePosition = Vector3.Lerp(StartPosition, EndPosition, t);
        
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
    
    private float FindTForArcDistance(float targetDistance, float horizontalLength)
    {
        if (targetDistance <= 0f)
            return 0f;
        if (targetDistance >= Length)
            return 1f;
        
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

    // Arc-length lookup table: a quadratic Bezier is not uniform in t, so without
    // it, cars would visibly speed up and slow down through turns.
    private float[] cumulativeLengths;

    public enum TurnType { Straight, Left, Right, Roundabout }

    /// <summary>
    /// For subclasses with their own path geometry: sets endpoints and type but
    /// builds no arc table. The subclass must call BuildArcLengthTable() once its
    /// geometry fields are ready, since the table samples the overridden curve.
    /// </summary>
    protected TurnCurve(Lane from, Lane to, TurnType type)
    {
        FromLane = from;
        ToLane = to;
        StartPoint = from.EndPosition;
        EndPoint = to.StartPosition;
        Type = type;
        ControlPoint = Vector3.Lerp(StartPoint, EndPoint, 0.5f); // placeholder; subclass path math ignores it
    }

    public TurnCurve(Lane from, Lane to, float radius)
    {
        FromLane = from;
        ToLane = to;
        StartPoint = from.EndPosition;
        EndPoint = to.StartPosition;

        // Unity is left-handed with +Y up, so on the XZ plane a right turn (clockwise,
        // e.g. +Z rotating toward +X) yields cross.y > 0; a left turn yields < 0.
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
            Type = TurnType.Right;
            ControlPoint = CalculateControlPoint(StartPoint, EndPoint, incomingDir, outgoingDir, radius);
        }
        else
        {
            Type = TurnType.Left;
            ControlPoint = CalculateControlPoint(StartPoint, EndPoint, incomingDir, outgoingDir, radius);
        }

        BuildArcLengthTable();
    }

    private Vector3 CalculateControlPoint(Vector3 start, Vector3 end, Vector3 inDir, Vector3 outDir, float radius)
    {
        float dot = Vector3.Dot(inDir, outDir);
        if (dot > 0.95f) // Nearly parallel (less than ~18 degree turn)
        {
            return Vector3.Lerp(start, end, 0.5f);
        }
        
        if (dot < -0.766f) // cos(140°) ≈ -0.766
        {
            // Ray intersection from both ends handles inner and outer curve shapes for sharp turns
            Vector3 sharpD1 = inDir;
            Vector3 sharpD2 = -outDir;
            
            float sharpDenom = sharpD1.x * sharpD2.z - sharpD1.z * sharpD2.x;
            
            if (Mathf.Abs(sharpDenom) > 0.001f)
            {
                float sharpT = ((end.x - start.x) * sharpD2.z - (end.z - start.z) * sharpD2.x) / sharpDenom;
                
                // Sharp-angle intersections can land further out, so allow more slack than usual
                float maxDist = Vector3.Distance(start, end) * 3.0f;
                
                if (sharpT > 0 && sharpT < maxDist)
                {
                    Vector3 sharpIntersection = start + sharpD1 * sharpT;
                    sharpIntersection.y = start.y;
                    return sharpIntersection;
                }
            }
            
            float extensionAmount = Vector3.Distance(start, end) * 0.4f;
            Vector3 sharpControl = start + inDir * extensionAmount;
            sharpControl.y = start.y;
            return sharpControl;
        }
        
        Vector3 d1 = inDir;
        Vector3 d2 = -outDir; // Reverse to point backward from end
        
        // Solve start + t1*d1 = end + t2*d2 for the two rays' intersection on XZ
        float denominator = d1.x * d2.z - d1.z * d2.x;
        
        if (Mathf.Abs(denominator) < 0.001f)
        {
            // Parallel or nearly parallel
            return Vector3.Lerp(start, end, 0.5f);
        }
        
        float t1 = ((end.x - start.x) * d2.z - (end.z - start.z) * d2.x) / denominator;
        float t2 = ((end.x - start.x) * d1.z - (end.z - start.z) * d1.x) / denominator;
        
        Vector3 intersection = start + d1 * t1;
        
        // Clamp control point distance to prevent wild curves
        float maxDistFromStart = Vector3.Distance(start, end) * 2.0f;
        float distFromStart = Vector3.Distance(start, intersection);
        
        if (t1 < 0 || t2 < 0 || distFromStart > maxDistFromStart)
        {
            // No valid intersection (behind a lane or too far out) - fall back to a perpendicular offset
            Vector3 midpoint = (start + end) * 0.5f;
            Vector3 avgDir = (inDir + outDir).normalized;
            Vector3 perpendicular = new Vector3(avgDir.z, 0, -avgDir.x); // Right-side driving
            return midpoint + perpendicular * radius;
        }
        
        intersection.y = start.y; // Maintain floor height
        return intersection;
    }

    /// <summary>
    /// Rebuilds the arc-length table so distance-based traversal stays uniform after reshaping.
    /// </summary>
    public void SetControlPoint(Vector3 controlPoint)
    {
        ControlPoint = controlPoint;
        BuildArcLengthTable();
    }

    protected void BuildArcLengthTable()
    {
        // Also sets ArcLength as a side effect (the final cumulative length)
        int samples = 24;
        cumulativeLengths = new float[samples + 1];
        float length = 0f;
        Vector3 prevPoint = GetPointOnCurve(0f);

        for (int i = 1; i <= samples; i++)
        {
            float t = i / (float)samples;
            Vector3 point = GetPointOnCurve(t);
            length += Vector3.Distance(prevPoint, point);
            cumulativeLengths[i] = length;
            prevPoint = point;
        }

        ArcLength = length;
    }

    /// <summary>
    /// Maps arc distance to the Bezier parameter t via interpolation in the cumulative-length table.
    /// </summary>
    public float GetTForArcDistance(float distance)
    {
        if (cumulativeLengths == null)
        {
            BuildArcLengthTable();
        }

        if (distance <= 0f)
            return 0f;
        if (distance >= ArcLength)
            return 1f;

        int samples = cumulativeLengths.Length - 1;
        for (int i = 1; i <= samples; i++)
        {
            if (cumulativeLengths[i] >= distance)
            {
                float segmentStart = cumulativeLengths[i - 1];
                float segmentLength = cumulativeLengths[i] - segmentStart;
                float fraction = segmentLength > 0.0001f ? (distance - segmentStart) / segmentLength : 0f;
                return (i - 1 + fraction) / samples;
            }
        }

        return 1f;
    }

    public virtual Vector3 GetPointOnCurve(float t)
    {
        // Quadratic Bezier curve
        t = Mathf.Clamp01(t);
        float u = 1f - t;
        return u * u * StartPoint + 2f * u * t * ControlPoint + t * t * EndPoint;
    }

    public virtual Vector3 GetTangentOnCurve(float t)
    {
        // Derivative of quadratic Bezier
        t = Mathf.Clamp01(t);
        return (2f * (1f - t) * (ControlPoint - StartPoint) + 2f * t * (EndPoint - ControlPoint)).normalized;
    }
    
    /// <summary>
    /// Alias for GetPointOnCurve.
    /// </summary>
    public Vector3 GetPositionAtTime(float t)
    {
        return GetPointOnCurve(t);
    }
    
    /// <summary>
    /// Alias for GetTangentOnCurve.
    /// </summary>
    public Vector3 GetDirectionAtTime(float t)
    {
        return GetTangentOnCurve(t);
    }
}

/// <summary>
/// Composite turn path through a roundabout: an entry connector (quadratic Bezier)
/// from the incoming lane onto a circular ring around the node, a ring arc travelled
/// with the island on the driver's left (right-hand traffic), and an exit connector
/// back onto the outgoing lane. The ring-coordinate helpers expose where a car is on
/// the shared ring so entry arbitration and car-following can reason about
/// circulating traffic across different turn instances of the same roundabout.
/// </summary>
[System.Serializable]
public class RoundaboutTurnCurve : TurnCurve
{
    public Vector3 Center;
    public float Radius;
    public float EntryAngle; // radians, position angle where the path merges onto the ring
    public float SweepAngle; // radians travelled along the ring, always > 0

    private Vector3 entryRingPoint;
    private Vector3 exitRingPoint;
    private Vector3 entryControl;
    private Vector3 exitControl;
    private float entryLength;
    private float ringLength;
    private float exitLength;

    // Merge/diverge points are pulled this far along the ring from the arm's radial
    // direction so connectors join the ring at a shallow tangent instead of head-on.
    private const float MaxLeadAngle = 20f * Mathf.Deg2Rad;

    public float RingCircumference => 2f * Mathf.PI * Radius;
    public float ExitAngle => EntryAngle + SweepAngle;
    public float EntryRingDistance => RingMod(EntryAngle * Radius, RingCircumference);
    public float ExitRingDistance => RingMod(ExitAngle * Radius, RingCircumference);

    public RoundaboutTurnCurve(Lane from, Lane to, Vector3 center, float radius)
        : base(from, to, TurnType.Roundabout)
    {
        Center = center;
        Radius = radius;

        float entryArmAngle = PositionAngle(StartPoint);
        float exitArmAngle = PositionAngle(EndPoint);

        // Shrink the leads on narrow arm gaps so the sweep can never go negative,
        // which the modulo would otherwise turn into an almost-full extra lap.
        float armGap = Mod2Pi(exitArmAngle - entryArmAngle);
        float lead = Mathf.Min(MaxLeadAngle, armGap * 0.25f);
        EntryAngle = entryArmAngle + lead;
        SweepAngle = armGap - 2f * lead;

        entryRingPoint = RingPoint(EntryAngle);
        exitRingPoint = RingPoint(ExitAngle);
        entryControl = ConnectorControl(StartPoint, from.GetDirection(), entryRingPoint, RingTangent(EntryAngle));
        exitControl = ConnectorControl(exitRingPoint, RingTangent(ExitAngle), EndPoint, to.GetDirection());

        entryLength = EstimateBezierLength(StartPoint, entryControl, entryRingPoint);
        ringLength = SweepAngle * Radius;
        exitLength = EstimateBezierLength(exitRingPoint, exitControl, EndPoint);

        BuildArcLengthTable();
    }

    private float TotalEstimatedLength => entryLength + ringLength + exitLength;

    public override Vector3 GetPointOnCurve(float t)
    {
        float total = TotalEstimatedLength;
        if (total <= 0.001f)
            return StartPoint;

        float d = Mathf.Clamp01(t) * total;
        if (d <= entryLength)
        {
            float u = entryLength > 0.001f ? d / entryLength : 1f;
            return BezierPoint(StartPoint, entryControl, entryRingPoint, u);
        }

        d -= entryLength;
        if (d <= ringLength)
        {
            return RingPoint(EntryAngle + d / Radius);
        }

        d -= ringLength;
        float v = exitLength > 0.001f ? d / exitLength : 1f;
        return BezierPoint(exitRingPoint, exitControl, EndPoint, v);
    }

    public override Vector3 GetTangentOnCurve(float t)
    {
        float total = TotalEstimatedLength;
        if (total <= 0.001f)
            return FromLane.GetDirection();

        float d = Mathf.Clamp01(t) * total;
        if (d <= entryLength)
        {
            float u = entryLength > 0.001f ? d / entryLength : 1f;
            return BezierTangent(StartPoint, entryControl, entryRingPoint, u);
        }

        d -= entryLength;
        if (d <= ringLength)
        {
            return RingTangent(EntryAngle + d / Radius);
        }

        d -= ringLength;
        float v = exitLength > 0.001f ? d / exitLength : 1f;
        return BezierTangent(exitRingPoint, exitControl, EndPoint, v);
    }

    /// <summary>
    /// Ring coordinate (arc distance in [0, circumference)) the car occupies at the
    /// given path distance, clamped to the entry point before merging and to the
    /// exit point after leaving the ring.
    /// </summary>
    public float RingPositionOf(float pathDistance)
    {
        float ringTravel = Mathf.Clamp(ToEstimateDomain(pathDistance) - entryLength, 0f, ringLength);
        return RingMod(EntryAngle * Radius + ringTravel, RingCircumference);
    }

    /// <summary>
    /// Ring arc distance still to be covered before this path leaves the ring; never negative.
    /// </summary>
    public float RemainingRingTravel(float pathDistance)
    {
        return ringLength - Mathf.Clamp(ToEstimateDomain(pathDistance) - entryLength, 0f, ringLength);
    }

    /// <summary>
    /// Entry-connector distance still to be covered before the path merges onto the ring.
    /// </summary>
    public float EntryConnectorRemaining(float pathDistance)
    {
        return Mathf.Max(0f, entryLength - ToEstimateDomain(pathDistance));
    }

    /// <summary>
    /// Distance travelled past the ring on the exit connector; 0 while still on it.
    /// </summary>
    public float ExitConnectorProgress(float pathDistance)
    {
        return Mathf.Max(0f, ToEstimateDomain(pathDistance) - entryLength - ringLength);
    }

    public bool SharesRingWith(RoundaboutTurnCurve other)
    {
        return other != null && (other.Center - Center).sqrMagnitude < 0.01f;
    }

    public float EntryConnectorLength => entryLength;
    public float RingArcLength => ringLength;

    /// <summary>
    /// Progress along the ring relative to this path's merge point: negative while
    /// still on the entry connector, greater than RingArcLength once on the exit
    /// connector. Lets callers project a car's future ring position linearly.
    /// </summary>
    public float SignedRingProgress(float pathDistance)
    {
        return ToEstimateDomain(pathDistance) - entryLength;
    }

    public static float RingMod(float value, float circumference)
    {
        float r = value % circumference;
        return r < 0f ? r + circumference : r;
    }

    // A car's DistanceAlongPath is true arc length (corrected by the arc-length
    // table), while segment boundaries live in the estimated-length domain the
    // composite parameterization uses; rescale before comparing against them.
    private float ToEstimateDomain(float pathDistance)
    {
        if (ArcLength <= 0.001f)
            return 0f;
        return Mathf.Clamp01(pathDistance / ArcLength) * TotalEstimatedLength;
    }

    private float PositionAngle(Vector3 p)
    {
        return Mathf.Atan2(p.z - Center.z, p.x - Center.x);
    }

    private Vector3 RingPoint(float angle)
    {
        return Center + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * Radius;
    }

    // Tangent of increasing angle: on Unity's XZ plane this circulation keeps the
    // island on the driver's left, which is the correct sense for right-hand traffic.
    private Vector3 RingTangent(float angle)
    {
        return new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));
    }

    private static float Mod2Pi(float angle)
    {
        float r = angle % (2f * Mathf.PI);
        return r < 0f ? r + 2f * Mathf.PI : r;
    }

    /// <summary>
    /// Control point for a connector Bezier: where the start heading meets the
    /// reversed end heading (same scheme as TurnCurve.CalculateControlPoint),
    /// with the chord midpoint as fallback for degenerate headings.
    /// </summary>
    private static Vector3 ConnectorControl(Vector3 start, Vector3 startDir, Vector3 end, Vector3 endDir)
    {
        Vector3 d1 = startDir;
        Vector3 d2 = -endDir;

        float denom = d1.x * d2.z - d1.z * d2.x;
        if (Mathf.Abs(denom) > 0.001f)
        {
            float t1 = ((end.x - start.x) * d2.z - (end.z - start.z) * d2.x) / denom;
            float t2 = ((end.x - start.x) * d1.z - (end.z - start.z) * d1.x) / denom;
            float maxDist = Vector3.Distance(start, end) * 2f;

            if (t1 > 0f && t2 > 0f && t1 < maxDist)
            {
                Vector3 control = start + d1 * t1;
                control.y = start.y;
                return control;
            }
        }

        return Vector3.Lerp(start, end, 0.5f);
    }

    private static Vector3 BezierPoint(Vector3 a, Vector3 control, Vector3 b, float u)
    {
        u = Mathf.Clamp01(u);
        float w = 1f - u;
        return w * w * a + 2f * w * u * control + u * u * b;
    }

    private static Vector3 BezierTangent(Vector3 a, Vector3 control, Vector3 b, float u)
    {
        u = Mathf.Clamp01(u);
        return (2f * (1f - u) * (control - a) + 2f * u * (b - control)).normalized;
    }

    private static float EstimateBezierLength(Vector3 a, Vector3 control, Vector3 b)
    {
        const int samples = 12;
        float length = 0f;
        Vector3 prev = a;

        for (int i = 1; i <= samples; i++)
        {
            Vector3 p = BezierPoint(a, control, b, i / (float)samples);
            length += Vector3.Distance(prev, p);
            prev = p;
        }

        return length;
    }
}

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

/// <summary>
/// Root export structure for JSON serialization, consumed by the Python visualization scripts.
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

