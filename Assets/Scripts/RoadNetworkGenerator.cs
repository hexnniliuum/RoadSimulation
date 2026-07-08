using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Generates a procedural road network with randomized spacing and intersections.
/// </summary>
public class RoadNetworkGenerator : MonoBehaviour
{
    [SerializeField] private RoadGenerationConfig config;
    
    private RoadMap roadMap;
    private System.Random random;

    public RoadMap GenerateRoadNetwork()
    {
        if (config == null)
        {
            Debug.LogError("RoadGenerationConfig is not assigned!");
            return null;
        }

        roadMap = new RoadMap();
        
        int seed = config.RandomSeed == 0 ? System.Environment.TickCount : config.RandomSeed;
        roadMap.RandomSeed = seed;
        random = new System.Random(seed);
        
        Debug.Log($"Generating road network with seed: {seed}");

        CreateGridNodes();

        List<RoadConnection> connections = CreateRoadConnections();

        RemoveRandomNodes();
        connections = RemoveInvalidConnections(connections);
        connections = RemoveRandomRoads(connections);
        connections = RemoveDeadEndNodes(connections);

        connections = RemoveSharpAngleRoads(connections);

        connections = ValidateAndRepairTopology(connections);

        CreateBridges(connections);

        Dictionary<(int, int), float> stopDistances = CalculateAllStopDistances(connections);

        CreateRoadsWithLanes(connections, stopDistances);

        PrecomputeTurnCurves();

        FlattenOverlappingOppositeLeftTurns();

        Debug.Log($"Generated road network: {roadMap.Nodes.Count} nodes, {roadMap.Roads.Count} roads");
        
        return roadMap;
    }

    private void CreateGridNodes()
    {
        int nodeId = 0;
        
        // Small grids (3x3) with high jitter (>5) can overlap lanes at intersections via
        // wrap-around; use 4x4+ grids for jitter values above 5.
        float[] xPositions = new float[config.GridWidth];
        float[] zPositions = new float[config.GridHeight];
        
        float avgSpacingX = (config.MinSegmentLength + config.MaxSegmentLength) * 0.5f;
        float avgSpacingZ = (config.MinSegmentLength + config.MaxSegmentLength) * 0.5f;
        
        float currentX = 0;
        for (int x = 0; x < config.GridWidth; x++)
        {
            xPositions[x] = currentX;
            float variation = (config.MaxSegmentLength - config.MinSegmentLength) * 0.2f;
            currentX += avgSpacingX + RandomRange(-variation, variation);
        }
        
        float currentZ = 0;
        for (int z = 0; z < config.GridHeight; z++)
        {
            zPositions[z] = currentZ;
            float variation = (config.MaxSegmentLength - config.MinSegmentLength) * 0.2f;
            currentZ += avgSpacingZ + RandomRange(-variation, variation);
        }
        
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                // Half the configured jitter, for a more regular-looking grid.
                float jitterX = RandomRange(-config.IntersectionJitter, config.IntersectionJitter) * 0.5f;
                float jitterZ = RandomRange(-config.IntersectionJitter, config.IntersectionJitter) * 0.5f;
                
                Vector3 position = new Vector3(
                    xPositions[x] + jitterX, 
                    config.FloorHeight, 
                    zPositions[z] + jitterZ
                );
                
                Node node = new Node(nodeId++, position);
                roadMap.Nodes.Add(node);
            }
        }
    }

    private List<RoadConnection> CreateRoadConnections()
    {
        // Pass 1: Build road topology without creating geometry
        List<RoadConnection> connections = new List<RoadConnection>();
        List<RoadConnection> tempConnections = new List<RoadConnection>(); // For intersection checking
        
        int roadId = 0;
        Node[,] grid = new Node[config.GridWidth, config.GridHeight];
        
        // Rebuild grid reference for easy lookup
        int index = 0;
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                grid[x, z] = roadMap.Nodes[index++];
            }
        }
        
        // Create horizontal road connections
        for (int z = 0; z < config.GridHeight; z++)
        {
            for (int x = 0; x < config.GridWidth; x++)
            {
                Node current = grid[x, z];
                int nextX = (x + 1) % config.GridWidth;
                Node next = grid[nextX, z];

                bool isWrapAround = (nextX == 0 && x == config.GridWidth - 1);

                // Wrap-around roads span the whole grid, so (like diagonals) they must
                // not pass through unrelated intersections.
                bool wrapAcceptable = !isWrapAround ||
                    (IsReasonableWrapDistance(current, next) && !PassesTooCloseToNodes(current, next, grid));

                if (!ConnectionExists(connections, current, next) && wrapAcceptable)
                {
                    RoadConnection conn = new RoadConnection(roadId++, current, next);

                    bool canAdd = TryHandleRoadCrossing(ref conn, tempConnections);

                    if (canAdd)
                    {
                        connections.Add(conn);
                        tempConnections.Add(conn);
                    }
                    else
                    {
                        Debug.LogWarning($"Skipping road from Node {current.ID} to {next.ID}: crossing not feasible for bridge");
                    }
                }
                else if (!ConnectionExists(connections, current, next) && isWrapAround)
                {
                    Debug.LogWarning($"Skipping wrap-around road from Node {current.ID} to {next.ID}: too long or passes through another intersection ({Vector3.Distance(current.Position, next.Position):F1})");
                }
            }
        }
        
        // Create vertical road connections
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                Node current = grid[x, z];
                int nextZ = (z + 1) % config.GridHeight;
                Node next = grid[x, nextZ];

                bool isWrapAround = (nextZ == 0 && z == config.GridHeight - 1);

                bool wrapAcceptable = !isWrapAround ||
                    (IsReasonableWrapDistance(current, next) && !PassesTooCloseToNodes(current, next, grid));

                if (!ConnectionExists(connections, current, next) && wrapAcceptable)
                {
                    RoadConnection conn = new RoadConnection(roadId++, current, next);

                    bool canAdd = TryHandleRoadCrossing(ref conn, tempConnections);

                    if (canAdd)
                    {
                        connections.Add(conn);
                        tempConnections.Add(conn);
                    }
                    else
                    {
                        Debug.LogWarning($"Skipping road from Node {current.ID} to {next.ID}: crossing not feasible for bridge");
                    }
                }
                else if (!ConnectionExists(connections, current, next) && isWrapAround)
                {
                    Debug.LogWarning($"Skipping wrap-around road from Node {current.ID} to {next.ID}: too long or passes through another intersection ({Vector3.Distance(current.Position, next.Position):F1})");
                }
            }
        }
        
        // Add diagonal/shortcut connections to create potential bridge crossings
        if (config.DiagonalConnectionPercentage > 0)
        {
            connections = AddDiagonalConnections(connections, tempConnections, ref roadId, grid);
        }
        
        return connections;
    }
    
    private List<RoadConnection> AddDiagonalConnections(List<RoadConnection> connections, List<RoadConnection> tempConnections, ref int roadId, Node[,] grid)
    {
        const float MIN_ANGLE_DEGREES = 20f;
        
        int potentialDiagonals = config.GridWidth * config.GridHeight;
        int diagonalsToAdd = Mathf.RoundToInt(potentialDiagonals * config.DiagonalConnectionPercentage / 100f);
        
        List<RoadConnection> newConnections = new List<RoadConnection>(connections);
        int added = 0;
        
        for (int attempt = 0; attempt < diagonalsToAdd * 3 && added < diagonalsToAdd; attempt++)
        {
            int x1 = random.Next(config.GridWidth);
            int z1 = random.Next(config.GridHeight);
            int x2 = random.Next(config.GridWidth);
            int z2 = random.Next(config.GridHeight);
            
            Node node1 = grid[x1, z1];
            Node node2 = grid[x2, z2];
            
            if (node1 == node2 || ConnectionExists(newConnections, node1, node2))
                continue;
            
            float distance = Vector3.Distance(node1.Position, node2.Position);
            if (distance > config.MaxSegmentLength * 2f)
                continue;
            
            if (PassesTooCloseToNodes(node1, node2, grid))
                continue;
            
            if (WouldCreateSharpAngle(node1, node2, newConnections, MIN_ANGLE_DEGREES) ||
                WouldCreateSharpAngle(node2, node1, newConnections, MIN_ANGLE_DEGREES))
                continue;
            
            RoadConnection diagonalConn = new RoadConnection(roadId++, node1, node2);
            
            bool canAdd = TryHandleRoadCrossing(ref diagonalConn, tempConnections);
            
            if (canAdd)
            {
                newConnections.Add(diagonalConn);
                tempConnections.Add(diagonalConn);
                added++;
            }
        }
        
        if (added > 0)
        {
            Debug.Log($"Added {added} diagonal/shortcut connections");
        }
        
        return newConnections;
    }
    
    private bool WouldCreateSharpAngle(Node atNode, Node toNode, List<RoadConnection> connections, float minAngleDegrees)
    {
        Vector3 newDir = (toNode.Position - atNode.Position).normalized;
        
        foreach (var conn in connections)
        {
            Vector3 existingDir;
            if (conn.StartNode == atNode)
                existingDir = (conn.EndNode.Position - atNode.Position).normalized;
            else if (conn.EndNode == atNode)
                existingDir = (conn.StartNode.Position - atNode.Position).normalized;
            else
                continue;
            
            float dot = Vector3.Dot(newDir, existingDir);
            float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;
            
            if (angleDegrees < minAngleDegrees)
                return true;
        }
        
        return false;
    }
    
    private bool PassesTooCloseToNodes(Node start, Node end, Node[,] grid)
    {
        float minDistance = config.MinDistanceFromIntersection;
        
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                Node node = grid[x, z];
                
                if (node == start || node == end)
                    continue;

                float distToSegment = DistanceFromPointToLineSegment(node.Position, start.Position, end.Position);

                if (distToSegment < minDistance)
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    private float DistanceFromPointToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        Vector3 lineDir = lineEnd - lineStart;
        float lineLength = lineDir.magnitude;

        if (lineLength < 0.001f)
            return Vector3.Distance(point, lineStart);

        lineDir /= lineLength;

        float t = Vector3.Dot(point - lineStart, lineDir);
        t = Mathf.Clamp(t, 0f, lineLength);

        Vector3 closestPoint = lineStart + lineDir * t;

        // XZ-only distance - height differences (bridges, slopes) shouldn't affect proximity checks.
        Vector3 diff = point - closestPoint;
        diff.y = 0;
        return diff.magnitude;
    }
    
    private bool ConnectionExists(List<RoadConnection> connections, Node from, Node to)
    {
        foreach (var conn in connections)
        {
            if ((conn.StartNode == from && conn.EndNode == to) ||
                (conn.StartNode == to && conn.EndNode == from))
            {
                return true;
            }
        }
        return false;
    }
    
    private bool IsReasonableWrapDistance(Node from, Node to)
    {
        // Jitter can push wrap-around nodes far enough apart to create absurdly long roads.
        float distance = Vector3.Distance(from.Position, to.Position);
        float maxReasonableDistance = config.MaxSegmentLength * 1.5f;
        
        return distance <= maxReasonableDistance;
    }
    
    private bool TryHandleRoadCrossing(ref RoadConnection newConn, List<RoadConnection> existingConnections)
    {
        // Bridges aren't built here; this only validates that a bridge could be built
        // later, once all removal passes are done.
        Vector3 p1 = newConn.StartNode.Position;
        Vector3 p2 = newConn.EndNode.Position;

        foreach (var existingConn in existingConnections)
        {
            if (existingConn.StartNode == newConn.StartNode || existingConn.StartNode == newConn.EndNode ||
                existingConn.EndNode == newConn.StartNode || existingConn.EndNode == newConn.EndNode)
            {
                continue;
            }

            Vector3 p3 = existingConn.StartNode.Position;
            Vector3 p4 = existingConn.EndNode.Position;

            if (LineSegmentsIntersect(p1, p2, p3, p4, out Vector3 crossingPoint))
            {
                if (!config.EnableBridges || !CanCreateBridge(newConn, existingConn, crossingPoint))
                {
                    return false;
                }
            }
        }

        return true;
    }
    
    private bool CanCreateBridge(RoadConnection newConn, RoadConnection existingConn, Vector3 crossingPoint)
    {
        float distToStart = Vector3.Distance(newConn.StartNode.Position, crossingPoint);
        float distToEnd = Vector3.Distance(newConn.EndNode.Position, crossingPoint);

        if (distToStart < config.MinDistanceFromIntersection || distToEnd < config.MinDistanceFromIntersection)
        {
            return false;
        }

        Vector3 dir1 = (newConn.EndNode.Position - newConn.StartNode.Position).normalized;
        Vector3 dir2 = (existingConn.EndNode.Position - existingConn.StartNode.Position).normalized;

        float dot = Mathf.Abs(Vector3.Dot(dir1, dir2));
        float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, 0f, 1f)) * Mathf.Rad2Deg;

        if (angleDegrees < config.MinBridgeCrossingAngle)
        {
            return false;
        }

        float requiredLength = config.MinBridgeRampLength * 2f;

        if (distToStart < config.MinBridgeRampLength || distToEnd < config.MinBridgeRampLength)
        {
            return false;
        }

        return true;
    }
    
    private void CreateBridges(List<RoadConnection> connections)
    {
        if (!config.EnableBridges)
            return;
        
        int bridgeCount = 0;
        
        for (int i = 0; i < connections.Count; i++)
        {
            RoadConnection conn = connections[i];
            Vector3 p1 = conn.StartNode.Position;
            Vector3 p2 = conn.EndNode.Position;
            
            // j > i avoids checking (and possibly bridging) each pair twice.
            for (int j = i + 1; j < connections.Count; j++)
            {
                RoadConnection other = connections[j];

                if (other.StartNode == conn.StartNode || other.StartNode == conn.EndNode ||
                    other.EndNode == conn.StartNode || other.EndNode == conn.EndNode)
                {
                    continue;
                }

                Vector3 p3 = other.StartNode.Position;
                Vector3 p4 = other.EndNode.Position;

                if (LineSegmentsIntersect(p1, p2, p3, p4, out Vector3 crossingPoint))
                {
                    // Bridge whichever side has room for ramps; validating only one side
                    // let some crossings through with no bridge at all (flat lane overlap).
                    if (CanCreateBridge(conn, other, crossingPoint))
                    {
                        AddBridge(ref conn, crossingPoint);
                        connections[i] = conn; // Update the struct in the list
                        bridgeCount++;
                    }
                    else if (CanCreateBridge(other, conn, crossingPoint))
                    {
                        AddBridge(ref other, crossingPoint);
                        connections[j] = other;
                        bridgeCount++;
                    }
                    else
                    {
                        Debug.LogWarning($"Roads {conn.ID} and {other.ID} cross at {crossingPoint} but neither can carry a bridge - lanes overlap at grade");
                    }
                }
            }
        }
        
        if (bridgeCount > 0)
        {
            Debug.Log($"Created {bridgeCount} bridge(s) after all removals");
        }
    }
    
    private void AddBridge(ref RoadConnection conn, Vector3 crossingPoint)
    {
        float distFromStart = Vector3.Distance(conn.StartNode.Position, crossingPoint);
        float distFromEnd = Vector3.Distance(conn.EndNode.Position, crossingPoint);

        float rampLength = config.MinBridgeRampLength;

        float startDist = distFromStart - rampLength;
        float endDist = distFromStart + rampLength;

        startDist = Mathf.Max(0f, startDist);
        endDist = Mathf.Min(conn.Length, endDist);

        BridgeInfo bridge = new BridgeInfo(
            crossingPoint,
            startDist,
            endDist,
            config.BridgeHeight,
            rampLength
        );

        // Merge overlapping bridge intervals - Lane sums elevation across all its
        // bridges, so overlapping intervals would double-stack height.
        conn.Bridges.Add(bridge);
        ConsolidateBridges(conn.Bridges);
    }

    private void ConsolidateBridges(List<BridgeInfo> bridges)
    {
        if (bridges.Count < 2)
            return;

        bridges.Sort((a, b) => a.StartDistance.CompareTo(b.StartDistance));

        for (int i = bridges.Count - 2; i >= 0; i--)
        {
            BridgeInfo current = bridges[i];
            BridgeInfo next = bridges[i + 1];

            if (next.StartDistance <= current.EndDistance)
            {
                current.EndDistance = Mathf.Max(current.EndDistance, next.EndDistance);
                current.PeakHeight = Mathf.Max(current.PeakHeight, next.PeakHeight);
                bridges.RemoveAt(i + 1);
            }
        }
    }
    
    private Dictionary<(int, int), float> CalculateAllStopDistances(List<RoadConnection> connections)
    {
        // Pass 2: Analyze all intersections with full network knowledge
        Dictionary<(int, int), float> stopDistances = new Dictionary<(int, int), float>();
        
        foreach (var conn in connections)
        {
            float stopDistStart = CalculateStopDistanceAtNode(conn.StartNode, conn.EndNode, connections);
            stopDistances[(conn.ID, conn.StartNode.ID)] = stopDistStart;

            float stopDistEnd = CalculateStopDistanceAtNode(conn.EndNode, conn.StartNode, connections);
            stopDistances[(conn.ID, conn.EndNode.ID)] = stopDistEnd;
        }
        
        return stopDistances;
    }
    
    private float CalculateStopDistanceAtNode(Node atNode, Node towardNode, List<RoadConnection> connections)
    {
        // Widens the setback for sharp angles at this node; using the full connection list
        // (not just this road) keeps both ends of an intersection symmetric.
        float baseDistance = config.StopLineDistance;

        Vector3 thisRoadDir = (towardNode.Position - atNode.Position).normalized;

        float minAngleDegrees = 180f; // Start with straight (180°)

        foreach (var conn in connections)
        {
            Vector3 otherDir;
            
            if (conn.StartNode == atNode)
            {
                // Don't compare with itself
                if (conn.EndNode == towardNode)
                    continue;
                otherDir = (conn.EndNode.Position - atNode.Position).normalized;
            }
            else if (conn.EndNode == atNode)
            {
                // Don't compare with itself (reverse direction)
                if (conn.StartNode == towardNode)
                    continue;
                otherDir = (conn.StartNode.Position - atNode.Position).normalized;
            }
            else
            {
                continue; // Connection doesn't involve this node
            }

            float dot = Vector3.Dot(thisRoadDir, otherDir);
            float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;

            minAngleDegrees = Mathf.Min(minAngleDegrees, angleDegrees);
        }

        // Geometric setback so lanes don't overlap at sharp angles: distance = (roadWidth / 2) / tan(angle / 2).
        if (minAngleDegrees < 90f)
        {
            float roadWidth = config.LaneWidth * 2f;

            float angleRadians = minAngleDegrees * Mathf.Deg2Rad;

            // 1.15x safety margin accounts for turn curves.
            float halfAngle = angleRadians / 2f;
            float geometricDistance = (roadWidth / 2f) / Mathf.Tan(halfAngle);
            float requiredDistance = geometricDistance * 1.15f;

            // Cap at the requirement for the sharpest angle the pipeline allows (20°,
            // matching RemoveSharpAngleRoads); a lower fixed cap left 20-37° intersections
            // with overlapping lanes.
            float maxCutoff = (roadWidth / 2f) / Mathf.Tan(20f * 0.5f * Mathf.Deg2Rad) * 1.15f;
            requiredDistance = Mathf.Min(requiredDistance, maxCutoff);

            return Mathf.Max(baseDistance, requiredDistance);
        }

        return baseDistance;
    }
    
    private void CreateRoadsWithLanes(List<RoadConnection> connections, Dictionary<(int, int), float> stopDistances)
    {
        // Pass 3: Create Road objects with pre-computed stop distances and bridges
        foreach (var conn in connections)
        {
            float stopDistStart = stopDistances[(conn.ID, conn.StartNode.ID)];
            float stopDistEnd = stopDistances[(conn.ID, conn.EndNode.ID)];

            // Never let the two setbacks consume the whole road: lane start
            // would land past lane end, inverting the lane direction
            float totalSetback = stopDistStart + stopDistEnd;
            float maxTotal = conn.Length - 2f;
            if (totalSetback > maxTotal)
            {
                float scale = Mathf.Max(0f, maxTotal) / totalSetback;
                stopDistStart *= scale;
                stopDistEnd *= scale;
                Debug.LogWarning($"Road {conn.ID}: stop distances scaled by {scale:F2} to fit road length {conn.Length:F1}");
            }

            Road road = new Road(conn.ID, conn.StartNode, conn.EndNode, config.LaneWidth, stopDistStart, stopDistEnd, conn.Bridges);
            roadMap.Roads.Add(road);
            conn.StartNode.ConnectedRoads.Add(road);
            conn.EndNode.ConnectedRoads.Add(road);
        }
    }
    
    private bool LineSegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
    {
        return LineSegmentsIntersect(p1, p2, p3, p4, out _);
    }
    
    private bool LineSegmentsIntersect(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, out Vector3 crossingPoint)
    {
        crossingPoint = Vector3.zero;

        // Parametric segment intersection on the XZ plane; height is ignored (bridges
        // cross at different Y).
        float x1 = p1.x, z1 = p1.z;
        float x2 = p2.x, z2 = p2.z;
        float x3 = p3.x, z3 = p3.z;
        float x4 = p4.x, z4 = p4.z;
        
        float denominator = (x1 - x2) * (z3 - z4) - (z1 - z2) * (x3 - x4);
        
        // Lines are parallel or coincident
        if (Mathf.Abs(denominator) < 0.0001f)
            return false;
        
        float t = ((x1 - x3) * (z3 - z4) - (z1 - z3) * (x3 - x4)) / denominator;
        float u = -((x1 - x2) * (z1 - z3) - (z1 - z2) * (x1 - x3)) / denominator;
        
        // Small epsilon avoids rejecting roads that barely touch at endpoints.
        const float epsilon = 0.01f;
        bool intersects = (t > epsilon && t < (1.0f - epsilon) && u > epsilon && u < (1.0f - epsilon));
        
        if (intersects)
        {
            crossingPoint = Vector3.Lerp(p1, p2, t);
        }
        
        return intersects;
    }

    private void PrecomputeTurnCurves()
    {
        foreach (Node node in roadMap.Nodes)
        {
            List<Lane> incomingLanes = new List<Lane>();
            List<Lane> outgoingLanes = new List<Lane>();
            
            foreach (Road road in node.ConnectedRoads)
            {
                if (road.ForwardLane.EndNode == node)
                    incomingLanes.Add(road.ForwardLane);
                if (road.BackwardLane.EndNode == node)
                    incomingLanes.Add(road.BackwardLane);
                if (road.ForwardLane.StartNode == node)
                    outgoingLanes.Add(road.ForwardLane);
                if (road.BackwardLane.StartNode == node)
                    outgoingLanes.Add(road.BackwardLane);
            }
            
            // Trivial (2-road) intersections must allow their only turn, even if it's
            // sharper than the normal minimum.
            bool isTrivial = node.IsTrivialIntersection();

            foreach (Lane inLane in incomingLanes)
            {
                Lane bestFallback = null;
                float bestFallbackAngle = float.MaxValue;

                foreach (Lane outLane in outgoingLanes)
                {
                    if (IsUTurn(inLane, outLane))
                        continue;

                    if (!isTrivial && IsTurnTooSharp(inLane, outLane))
                    {
                        // Remember the least-sharp rejected turn as a fallback
                        float angle = TurnAngleDegrees(inLane, outLane);
                        if (angle < bestFallbackAngle)
                        {
                            bestFallbackAngle = angle;
                            bestFallback = outLane;
                        }
                        continue;
                    }

                    TurnCurve curve = new TurnCurve(inLane, outLane, config.TurnRadius);
                    inLane.AvailableTurns.Add(curve);
                }

                // A lane with no exits strands cars at its end forever and blocks everyone
                // behind them - keep the least-sharp turn even if it breaks the sharpness limit.
                if (inLane.AvailableTurns.Count == 0 && bestFallback != null)
                {
                    inLane.AvailableTurns.Add(new TurnCurve(inLane, bestFallback, config.TurnRadius));
                    Debug.LogWarning($"Lane {inLane.ID} at node {node.ID}: keeping sharp turn ({bestFallbackAngle:F0}°) as its only exit");
                }
            }
        }
    }

    private void FlattenOverlappingOppositeLeftTurns()
    {
        // Opposite-approach left arcs can cross at skewed nodes, forcing the conflict map
        // to serialize them; pull both control points toward their chords until they clear,
        // leaving pairs that can't clear without degenerating into a near-straight diagonal serialized.
        const float oppositeApproachMaxDot = -0.5f; // approaches at least ~120° apart
        const float triggerClearance = 2.0f;        // car width: closer than this, bodies overlap
        const float targetClearance = 2.5f;         // flatten until bodies clear with margin
        const float flattenStep = 0.05f;
        const float maxFlatten = 0.85f;             // past this the arc reads as a straight diagonal cut

        foreach (Node node in roadMap.Nodes)
        {
            List<TurnCurve> leftTurns = new List<TurnCurve>();
            foreach (Road road in node.ConnectedRoads)
            {
                CollectLeftTurnsAtNode(road.ForwardLane, node, leftTurns);
                CollectLeftTurnsAtNode(road.BackwardLane, node, leftTurns);
            }

            for (int i = 0; i < leftTurns.Count; i++)
            {
                for (int j = i + 1; j < leftTurns.Count; j++)
                {
                    TurnCurve a = leftTurns[i];
                    TurnCurve b = leftTurns[j];

                    // Same-lane turns already serialize by queue order; turns merging into
                    // the same lane share an endpoint and can never be separated - the
                    // conflict map handles both.
                    if (a.FromLane == b.FromLane || a.ToLane == b.ToLane)
                        continue;
                    if (Vector3.Dot(a.FromLane.GetDirection(), b.FromLane.GetDirection()) > oppositeApproachMaxDot)
                        continue;
                    if (MinCurveSeparation(a, b) >= triggerClearance)
                        continue;

                    Vector3 originalA = a.ControlPoint;
                    Vector3 originalB = b.ControlPoint;
                    Vector3 chordMidA = Vector3.Lerp(a.StartPoint, a.EndPoint, 0.5f);
                    Vector3 chordMidB = Vector3.Lerp(b.StartPoint, b.EndPoint, 0.5f);

                    bool cleared = false;
                    for (float k = flattenStep; k <= maxFlatten + 0.001f; k += flattenStep)
                    {
                        a.SetControlPoint(Vector3.Lerp(originalA, chordMidA, k));
                        b.SetControlPoint(Vector3.Lerp(originalB, chordMidB, k));

                        if (MinCurveSeparation(a, b) >= targetClearance)
                        {
                            cleared = true;
                            Debug.Log($"Node {node.ID}: flattened opposite-left pair " +
                                      $"{a.FromLane.ID}->{a.ToLane.ID} / {b.FromLane.ID}->{b.ToLane.ID} (k={k:F2})");
                            break;
                        }
                    }

                    if (!cleared)
                    {
                        a.SetControlPoint(originalA);
                        b.SetControlPoint(originalB);
                    }
                }
            }
        }
    }

    private void CollectLeftTurnsAtNode(Lane lane, Node node, List<TurnCurve> leftTurns)
    {
        // A lane's AvailableTurns all live at its end node
        if (lane.EndNode != node)
            return;

        foreach (TurnCurve turn in lane.AvailableTurns)
        {
            if (turn.Type == TurnCurve.TurnType.Left)
                leftTurns.Add(turn);
        }
    }

    /// <summary>
    /// Minimum XZ distance between two turn arcs, 0 if they cross. Samples each
    /// curve at the same resolution as IntersectionManager.TurnCurvesConflict so
    /// a pair separated here is guaranteed to pass the conflict map's check.
    /// </summary>
    private float MinCurveSeparation(TurnCurve a, TurnCurve b)
    {
        const int samples = 10;
        Vector3[] pointsA = new Vector3[samples + 1];
        Vector3[] pointsB = new Vector3[samples + 1];

        for (int i = 0; i <= samples; i++)
        {
            float t = i / (float)samples;
            pointsA[i] = a.GetPointOnCurve(t);
            pointsB[i] = b.GetPointOnCurve(t);
        }

        float minDistance = float.MaxValue;
        for (int i = 0; i < samples; i++)
        {
            for (int j = 0; j < samples; j++)
            {
                if (SegmentsIntersect2D(pointsA[i], pointsA[i + 1], pointsB[j], pointsB[j + 1]))
                    return 0f;

                minDistance = Mathf.Min(minDistance, PointToSegmentDistance2D(pointsA[i], pointsB[j], pointsB[j + 1]));
                minDistance = Mathf.Min(minDistance, PointToSegmentDistance2D(pointsA[i + 1], pointsB[j], pointsB[j + 1]));
                minDistance = Mathf.Min(minDistance, PointToSegmentDistance2D(pointsB[j], pointsA[i], pointsA[i + 1]));
                minDistance = Mathf.Min(minDistance, PointToSegmentDistance2D(pointsB[j + 1], pointsA[i], pointsA[i + 1]));
            }
        }

        return minDistance;
    }

    private bool SegmentsIntersect2D(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
    {
        // Endpoint touches count as intersections, matching IntersectionManager.LineSegmentsIntersect2D.
        float denom = (p1.x - p2.x) * (p3.z - p4.z) - (p1.z - p2.z) * (p3.x - p4.x);

        if (Mathf.Abs(denom) < 0.0001f)
            return false; // Parallel or coincident

        float t = ((p1.x - p3.x) * (p3.z - p4.z) - (p1.z - p3.z) * (p3.x - p4.x)) / denom;
        float u = -((p1.x - p2.x) * (p1.z - p3.z) - (p1.z - p2.z) * (p1.x - p3.x)) / denom;

        return t >= 0f && t <= 1f && u >= 0f && u <= 1f;
    }

    private float PointToSegmentDistance2D(Vector3 point, Vector3 segStart, Vector3 segEnd)
    {
        Vector2 p = new Vector2(point.x, point.z);
        Vector2 s = new Vector2(segStart.x, segStart.z);
        Vector2 e = new Vector2(segEnd.x, segEnd.z);

        Vector2 seg = e - s;
        float lengthSq = seg.sqrMagnitude;
        if (lengthSq < 0.0001f)
            return Vector2.Distance(p, s);

        float t = Mathf.Clamp01(Vector2.Dot(p - s, seg) / lengthSq);
        return Vector2.Distance(p, s + seg * t);
    }

    private bool IsUTurn(Lane inLane, Lane outLane)
    {
        return outLane.EndNode == inLane.StartNode;
    }

    private bool IsTurnTooSharp(Lane inLane, Lane outLane)
    {
        const float MAX_TURN_ANGLE = 140f; // corresponds to roads meeting at < 40°

        return TurnAngleDegrees(inLane, outLane) > MAX_TURN_ANGLE;
    }

    private float TurnAngleDegrees(Lane inLane, Lane outLane)
    {
        Vector3 incomingDir = inLane.GetDirection();
        Vector3 outgoingDir = outLane.GetDirection();

        float dot = Vector3.Dot(incomingDir, outgoingDir);
        return Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;
    }

    private void RemoveRandomNodes()
    {
        if (config.NodeRemovalPercentage <= 0)
            return;
        
        // Identify border nodes (keep these for closed loop)
        HashSet<Node> borderNodes = new HashSet<Node>();
        Node[,] grid = new Node[config.GridWidth, config.GridHeight];
        
        int index = 0;
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                grid[x, z] = roadMap.Nodes[index++];

                if (x == 0 || x == config.GridWidth - 1 || z == 0 || z == config.GridHeight - 1)
                {
                    borderNodes.Add(grid[x, z]);
                }
            }
        }
        
        List<Node> removableNodes = new List<Node>();
        foreach (Node node in roadMap.Nodes)
        {
            if (!borderNodes.Contains(node))
            {
                removableNodes.Add(node);
            }
        }
        
        int nodesToRemove = Mathf.RoundToInt(removableNodes.Count * config.NodeRemovalPercentage / 100f);

        for (int i = 0; i < nodesToRemove && removableNodes.Count > 0; i++)
        {
            int randomIndex = random.Next(removableNodes.Count);
            Node nodeToRemove = removableNodes[randomIndex];
            
            roadMap.Nodes.Remove(nodeToRemove);
            removableNodes.RemoveAt(randomIndex);
        }
        
        Debug.Log($"Removed {nodesToRemove} interior nodes");
    }
    
    private List<RoadConnection> RemoveInvalidConnections(List<RoadConnection> connections)
    {
        List<RoadConnection> validConnections = new List<RoadConnection>();
        
        foreach (var conn in connections)
        {
            if (roadMap.Nodes.Contains(conn.StartNode) && roadMap.Nodes.Contains(conn.EndNode))
            {
                validConnections.Add(conn);
            }
        }
        
        Debug.Log($"Removed {connections.Count - validConnections.Count} invalid connections");
        return validConnections;
    }
    
    private List<RoadConnection> RemoveRandomRoads(List<RoadConnection> connections)
    {
        if (config.RoadRemovalPercentage <= 0 || connections.Count == 0)
            return connections;
        
        int roadsToRemove = Mathf.RoundToInt(connections.Count * config.RoadRemovalPercentage / 100f);
        
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        int removedCount = 0;
        
        // Try to remove roads while maintaining connectivity and minimum node degree
        for (int i = 0; i < roadsToRemove && mutableConnections.Count > 1; i++)
        {
            int randomIndex = random.Next(mutableConnections.Count);
            RoadConnection candidate = mutableConnections[randomIndex];

            List<RoadConnection> testConnections = new List<RoadConnection>(mutableConnections);
            testConnections.RemoveAt(randomIndex);
            
            // Check both connectivity and minimum node degree (at least 2 roads per node)
            if (IsNetworkConnected(testConnections) && AllNodesHaveMinimumDegree(testConnections))
            {
                mutableConnections.RemoveAt(randomIndex);
                removedCount++;
            }
        }
        
        Debug.Log($"Removed {removedCount} roads while maintaining connectivity and minimum degree");
        return mutableConnections;
    }
    
    private bool AllNodesHaveMinimumDegree(List<RoadConnection> connections)
    {
        Dictionary<Node, int> nodeDegree = new Dictionary<Node, int>();
        
        foreach (Node node in roadMap.Nodes)
        {
            nodeDegree[node] = 0;
        }
        
        foreach (var conn in connections)
        {
            if (nodeDegree.ContainsKey(conn.StartNode))
                nodeDegree[conn.StartNode]++;
            if (nodeDegree.ContainsKey(conn.EndNode))
                nodeDegree[conn.EndNode]++;
        }
        
        // Every node needs at least 2 connections.
        foreach (var kvp in nodeDegree)
        {
            if (kvp.Value < 2)
            {
                return false;
            }
        }
        
        return true;
    }
    
    private List<RoadConnection> RemoveDeadEndNodes(List<RoadConnection> connections)
    {
        // Repeats until no degree-1 (dead-end) nodes remain.
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        bool removedAny = true;
        int totalRemoved = 0;
        
        while (removedAny)
        {
            removedAny = false;

            Dictionary<Node, int> nodeDegree = new Dictionary<Node, int>();
            foreach (Node node in roadMap.Nodes)
            {
                nodeDegree[node] = 0;
            }

            foreach (var conn in mutableConnections)
            {
                if (nodeDegree.ContainsKey(conn.StartNode))
                    nodeDegree[conn.StartNode]++;
                if (nodeDegree.ContainsKey(conn.EndNode))
                    nodeDegree[conn.EndNode]++;
            }

            List<Node> deadEndNodes = new List<Node>();
            foreach (var kvp in nodeDegree)
            {
                if (kvp.Value == 1)
                {
                    deadEndNodes.Add(kvp.Key);
                }
            }
            
            if (deadEndNodes.Count > 0)
            {
                foreach (Node node in deadEndNodes)
                {
                    roadMap.Nodes.Remove(node);
                }

                mutableConnections.RemoveAll(conn =>
                    deadEndNodes.Contains(conn.StartNode) || 
                    deadEndNodes.Contains(conn.EndNode));
                
                totalRemoved += deadEndNodes.Count;
                removedAny = true;
            }
        }
        
        if (totalRemoved > 0)
        {
            Debug.Log($"Removed {totalRemoved} dead-end nodes");
        }
        
        return mutableConnections;
    }
    
    private List<RoadConnection> RemoveSharpAngleRoads(List<RoadConnection> connections)
    {
        const float MIN_ANGLE_DEGREES = 20f;
        
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        int totalRemoved = 0;
        bool removedAny = true;
        
        while (removedAny)
        {
            removedAny = false;

            foreach (Node node in roadMap.Nodes)
            {
                List<RoadConnection> connectedRoads = new List<RoadConnection>();
                foreach (var conn in mutableConnections)
                {
                    if (conn.StartNode == node || conn.EndNode == node)
                    {
                        connectedRoads.Add(conn);
                    }
                }
                
                // Need at least 2 roads to have an angle
                if (connectedRoads.Count < 2)
                    continue;
                
                for (int i = 0; i < connectedRoads.Count; i++)
                {
                    for (int j = i + 1; j < connectedRoads.Count; j++)
                    {
                        RoadConnection road1 = connectedRoads[i];
                        RoadConnection road2 = connectedRoads[j];

                        Vector3 dir1 = road1.StartNode == node
                            ? (road1.EndNode.Position - node.Position).normalized
                            : (road1.StartNode.Position - node.Position).normalized;

                        Vector3 dir2 = road2.StartNode == node
                            ? (road2.EndNode.Position - node.Position).normalized
                            : (road2.StartNode.Position - node.Position).normalized;

                        float dot = Vector3.Dot(dir1, dir2);
                        float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;

                        if (angleDegrees < MIN_ANGLE_DEGREES)
                        {
                            // Remove the shorter road to minimize impact.
                            RoadConnection toRemove = road1.Length < road2.Length ? road1 : road2;

                            List<RoadConnection> testConnections = new List<RoadConnection>(mutableConnections);
                            testConnections.Remove(toRemove);
                            
                            if (IsNetworkConnected(testConnections) && AllNodesHaveMinimumDegree(testConnections))
                            {
                                mutableConnections.Remove(toRemove);
                                totalRemoved++;
                                removedAny = true;
                                Debug.Log($"Removed road {toRemove.ID} due to sharp angle ({angleDegrees:F1}°) at node {node.ID}");
                                break; // Break inner loop to re-evaluate this node
                            }
                        }
                    }
                    
                    if (removedAny)
                        break; // Break outer loop to restart from beginning
                }
                
                if (removedAny)
                    break; // Break node loop to restart
            }
        }
        
        if (totalRemoved > 0)
        {
            Debug.Log($"Removed {totalRemoved} road(s) due to sharp angles (< {MIN_ANGLE_DEGREES}°)");
        }

        return mutableConnections;
    }

    /// <summary>
    /// Safety net for "impossible intersection" geometry the earlier passes miss at high
    /// jitter: sub-minimum-angle spikes at a node, roads grazing an unrelated node, and
    /// unbridgeable at-grade overlaps between unrelated roads. Removals are guarded by a
    /// connectivity check (dead-ends may result but are cleaned up after; the graph never
    /// disconnects), and bridgeable crossings are left alone.
    /// </summary>
    private List<RoadConnection> ValidateAndRepairTopology(List<RoadConnection> connections)
    {
        const float MIN_ANGLE_DEGREES = 20f;
        float roadWidth = config.LaneWidth * 2f;
        // Foreign node closer than this means the road's lanes cover that intersection.
        float nodeClearance = roadWidth + config.LaneWidth * 0.5f;
        // Two unrelated road centrelines closer than this have overlapping lanes.
        float roadClearance = roadWidth * 0.9f;

        List<RoadConnection> conns = new List<RoadConnection>(connections);
        int repaired = 0;
        int guard = 0;
        int maxIterations = conns.Count * 4 + 20;
        bool changed = true;

        while (changed && guard++ < maxIterations)
        {
            changed = false;

            // 1. Sub-minimum-angle spike at a node.
            foreach (Node node in roadMap.Nodes)
            {
                List<RoadConnection> at = ConnectionsAtNode(conns, node);
                if (at.Count < 2) continue;
                if (FindSharpPair(node, at, MIN_ANGLE_DEGREES, out RoadConnection s1, out RoadConnection s2))
                {
                    RoadConnection shorter = s1.Length <= s2.Length ? s1 : s2;
                    RoadConnection longer = s1.Length <= s2.Length ? s2 : s1;
                    if (TryRemoveRoad(ref conns, shorter) || TryRemoveRoad(ref conns, longer))
                    {
                        changed = true;
                        repaired++;
                        break;
                    }
                }
            }
            if (changed) continue;

            // 2. Road grazing an unrelated node.
            foreach (var conn in conns)
            {
                if (FindGrazedNode(conn, nodeClearance) != null && TryRemoveRoad(ref conns, conn))
                {
                    changed = true;
                    repaired++;
                    break;
                }
            }
            if (changed) continue;

            // 3. Un-bridgeable at-grade overlap between two unrelated roads.
            for (int i = 0; i < conns.Count && !changed; i++)
            {
                for (int j = i + 1; j < conns.Count && !changed; j++)
                {
                    RoadConnection a = conns[i];
                    RoadConnection b = conns[j];
                    if (ShareNode(a, b)) continue;
                    if (!RoadsOverlapAtGrade(a, b, roadClearance)) continue;

                    // Leave it if it is a legitimate, bridgeable crossing.
                    bool crosses = LineSegmentsIntersect(
                        a.StartNode.Position, a.EndNode.Position,
                        b.StartNode.Position, b.EndNode.Position, out Vector3 cp);
                    if (crosses && config.EnableBridges &&
                        (CanCreateBridge(a, b, cp) || CanCreateBridge(b, a, cp)))
                        continue;

                    if (TryRemoveRoad(ref conns, a) || TryRemoveRoad(ref conns, b))
                    {
                        changed = true;
                        repaired++;
                    }
                }
            }
        }

        // Removals may have created dead-ends; clean them up (subtractive only, so this
        // can't introduce new violations).
        conns = RemoveInvalidConnections(conns);
        conns = RemoveDeadEndNodes(conns);
        conns = RemoveInvalidConnections(conns);

        if (repaired > 0)
        {
            Debug.Log($"Topology repair: resolved {repaired} impossible-intersection violation(s)");
        }

        return conns;
    }

    private List<RoadConnection> ConnectionsAtNode(List<RoadConnection> conns, Node node)
    {
        List<RoadConnection> result = new List<RoadConnection>();
        foreach (var c in conns)
        {
            if (c.StartNode == node || c.EndNode == node)
                result.Add(c);
        }
        return result;
    }

    private bool FindSharpPair(Node node, List<RoadConnection> at, float minAngleDegrees,
                               out RoadConnection r1, out RoadConnection r2)
    {
        for (int i = 0; i < at.Count; i++)
        {
            for (int j = i + 1; j < at.Count; j++)
            {
                Vector3 d1 = DirFromNode(at[i], node);
                Vector3 d2 = DirFromNode(at[j], node);
                float angle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(d1, d2), -1f, 1f)) * Mathf.Rad2Deg;
                if (angle < minAngleDegrees)
                {
                    r1 = at[i];
                    r2 = at[j];
                    return true;
                }
            }
        }
        r1 = default;
        r2 = default;
        return false;
    }

    private Vector3 DirFromNode(RoadConnection conn, Node node)
    {
        Node other = conn.StartNode == node ? conn.EndNode : conn.StartNode;
        return (other.Position - node.Position).normalized;
    }

    private Node FindGrazedNode(RoadConnection conn, float clearance)
    {
        foreach (Node node in roadMap.Nodes)
        {
            if (node == conn.StartNode || node == conn.EndNode)
                continue;
            float dist = DistanceFromPointToLineSegment(node.Position, conn.StartNode.Position, conn.EndNode.Position);
            if (dist < clearance)
                return node;
        }
        return null;
    }

    private bool ShareNode(RoadConnection a, RoadConnection b)
    {
        return a.StartNode == b.StartNode || a.StartNode == b.EndNode ||
               a.EndNode == b.StartNode || a.EndNode == b.EndNode;
    }

    private bool RoadsOverlapAtGrade(RoadConnection a, RoadConnection b, float clearance)
    {
        float dist = SegmentToSegmentDistanceXZ(
            a.StartNode.Position, a.EndNode.Position,
            b.StartNode.Position, b.EndNode.Position);
        return dist < clearance;
    }

    private float SegmentToSegmentDistanceXZ(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2)
    {
        // Crossing segments have zero separation.
        if (LineSegmentsIntersect(a1, a2, b1, b2))
            return 0f;
        // Otherwise the minimum is at one of the four endpoints.
        float d1 = DistanceFromPointToLineSegment(a1, b1, b2);
        float d2 = DistanceFromPointToLineSegment(a2, b1, b2);
        float d3 = DistanceFromPointToLineSegment(b1, a1, a2);
        float d4 = DistanceFromPointToLineSegment(b2, a1, a2);
        return Mathf.Min(Mathf.Min(d1, d2), Mathf.Min(d3, d4));
    }

    /// <summary>
    /// Remove a road if the network stays connected without it. The removal may leave
    /// a degree-1 dead-end (cleaned later) but never isolates a node.
    /// </summary>
    private bool TryRemoveRoad(ref List<RoadConnection> conns, RoadConnection road)
    {
        List<RoadConnection> test = new List<RoadConnection>(conns);
        test.Remove(road);
        if (test.Count == 0)
            return false;
        if (!IsNetworkConnected(test))
            return false;
        conns = test;
        return true;
    }

    private bool IsNetworkConnected(List<RoadConnection> connections)
    {
        if (roadMap.Nodes.Count == 0 || connections.Count == 0)
            return false;

        Dictionary<Node, List<Node>> adjacency = new Dictionary<Node, List<Node>>();
        foreach (Node node in roadMap.Nodes)
        {
            adjacency[node] = new List<Node>();
        }
        
        foreach (var conn in connections)
        {
            if (adjacency.ContainsKey(conn.StartNode) && adjacency.ContainsKey(conn.EndNode))
            {
                adjacency[conn.StartNode].Add(conn.EndNode);
                adjacency[conn.EndNode].Add(conn.StartNode);
            }
        }
        
        // BFS from first node to check if all nodes are reachable
        HashSet<Node> visited = new HashSet<Node>();
        Queue<Node> queue = new Queue<Node>();
        
        Node startNode = roadMap.Nodes[0];
        queue.Enqueue(startNode);
        visited.Add(startNode);
        
        while (queue.Count > 0)
        {
            Node current = queue.Dequeue();
            
            if (adjacency.ContainsKey(current))
            {
                foreach (Node neighbor in adjacency[current])
                {
                    if (!visited.Contains(neighbor))
                    {
                        visited.Add(neighbor);
                        queue.Enqueue(neighbor);
                    }
                }
            }
        }
        
        return visited.Count == roadMap.Nodes.Count;
    }
    
    private float RandomRange(float min, float max)
    {
        return min + (float)random.NextDouble() * (max - min);
    }

    public RoadMap GetRoadMap()
    {
        return roadMap;
    }

    public RoadGenerationConfig GetConfig()
    {
        return config;
    }

    [ContextMenu("Export Road Network to JSON")]
    public void ExportToJSONFromContextMenu()
    {
        string path = UnityEngine.Application.dataPath + "/../PythonAnalysis/road_network_export.json";
        ExportToJSON(path);
    }

    /// <summary>
    /// Includes nodes, roads, lanes, bridges, and turn curves.
    /// </summary>
    public void ExportToJSON(string filePath)
    {
        if (roadMap == null || roadMap.Nodes.Count == 0)
        {
            Debug.LogError("Cannot export: Road network is empty. Generate a network first.");
            return;
        }

        RoadNetworkExport export = new RoadNetworkExport();
        export.RandomSeed = roadMap.RandomSeed;
        export.Nodes = new List<NodeExport>();
        export.Roads = new List<RoadExport>();
        export.Lanes = new List<LaneExport>();
        export.Bridges = new List<BridgeExport>();
        export.TurnCurves = new List<TurnCurveExport>();

        foreach (Node node in roadMap.Nodes)
        {
            NodeExport ne = new NodeExport
            {
                ID = node.ID,
                X = node.Position.x,
                Y = node.Position.y,
                Z = node.Position.z,
                ConnectedRoadIDs = new List<int>()
            };

            foreach (Road road in node.ConnectedRoads)
            {
                ne.ConnectedRoadIDs.Add(road.ID);
            }

            export.Nodes.Add(ne);
        }

        int turnCurveID = 0;
        foreach (Road road in roadMap.Roads)
        {
            RoadExport re = new RoadExport
            {
                ID = road.ID,
                StartNodeID = road.StartNode.ID,
                EndNodeID = road.EndNode.ID,
                Length = road.Length,
                ForwardLaneID = road.ForwardLane.ID,
                BackwardLaneID = road.BackwardLane.ID
            };
            export.Roads.Add(re);

            export.Lanes.Add(ExportLane(road.ForwardLane, export.Bridges));
            turnCurveID = ExportLaneTurnCurves(road.ForwardLane, export.TurnCurves, turnCurveID);

            export.Lanes.Add(ExportLane(road.BackwardLane, export.Bridges));
            turnCurveID = ExportLaneTurnCurves(road.BackwardLane, export.TurnCurves, turnCurveID);
        }

        string json = UnityEngine.JsonUtility.ToJson(export, prettyPrint: true);

        try
        {
            string directory = System.IO.Path.GetDirectoryName(filePath);
            if (!System.IO.Directory.Exists(directory))
            {
                System.IO.Directory.CreateDirectory(directory);
            }

            System.IO.File.WriteAllText(filePath, json);
            Debug.Log($"Successfully exported road network to: {filePath}\n" +
                     $"Nodes: {export.Nodes.Count}, Roads: {export.Roads.Count}, " +
                     $"Lanes: {export.Lanes.Count}, Bridges: {export.Bridges.Count}, " +
                     $"Turn Curves: {export.TurnCurves.Count}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to export road network: {e.Message}");
        }
    }

    private LaneExport ExportLane(Lane lane, List<BridgeExport> bridges)
    {
        LaneExport le = new LaneExport
        {
            ID = lane.ID,
            StartNodeID = lane.StartNode.ID,
            EndNodeID = lane.EndNode.ID,
            StartX = lane.StartPosition.x,
            StartY = lane.StartPosition.y,
            StartZ = lane.StartPosition.z,
            EndX = lane.EndPosition.x,
            EndY = lane.EndPosition.y,
            EndZ = lane.EndPosition.z,
            Length = lane.Length,
            IsReversed = lane.IsReversed,
            RoadLength = lane.RoadLength,
            StopLineDistanceAtStart = lane.StopLineDistanceAtStart,
            BridgeIndices = new List<int>(),
            AvailableTurnIDs = new List<int>()
        };

        // Deduplicated - the same bridge object is shared by both lanes of a road.
        if (lane.Bridges != null && lane.Bridges.Count > 0)
        {
            foreach (var bridge in lane.Bridges)
            {
                int existingIndex = -1;
                for (int i = 0; i < bridges.Count; i++)
                {
                    var b = bridges[i];
                    if (Mathf.Approximately(b.CrossingX, bridge.CrossingPoint.x) &&
                        Mathf.Approximately(b.CrossingZ, bridge.CrossingPoint.z) &&
                        Mathf.Approximately(b.StartDistance, bridge.StartDistance))
                    {
                        existingIndex = i;
                        break;
                    }
                }

                if (existingIndex >= 0)
                {
                    le.BridgeIndices.Add(existingIndex);
                }
                else
                {
                    int bridgeIndex = bridges.Count;
                    bridges.Add(new BridgeExport
                    {
                        CrossingX = bridge.CrossingPoint.x,
                        CrossingZ = bridge.CrossingPoint.z,
                        StartDistance = bridge.StartDistance,
                        EndDistance = bridge.EndDistance,
                        PeakHeight = bridge.PeakHeight,
                        RampLength = bridge.RampLength
                    });
                    le.BridgeIndices.Add(bridgeIndex);
                }
            }
        }

        return le;
    }

    private int ExportLaneTurnCurves(Lane lane, List<TurnCurveExport> turnCurves, int startID)
    {
        int currentID = startID;

        foreach (var turn in lane.AvailableTurns)
        {
            TurnCurveExport te = new TurnCurveExport
            {
                ID = currentID,
                FromLaneID = turn.FromLane.ID,
                ToLaneID = turn.ToLane.ID,
                StartX = turn.StartPoint.x,
                StartY = turn.StartPoint.y,
                StartZ = turn.StartPoint.z,
                ControlX = turn.ControlPoint.x,
                ControlY = turn.ControlPoint.y,
                ControlZ = turn.ControlPoint.z,
                EndX = turn.EndPoint.x,
                EndY = turn.EndPoint.y,
                EndZ = turn.EndPoint.z,
                ArcLength = turn.ArcLength,
                TurnType = turn.Type.ToString()
            };

            turnCurves.Add(te);
            currentID++;
        }

        return currentID;
    }
}


