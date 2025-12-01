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
        
        // Initialize random with seed
        int seed = config.RandomSeed == 0 ? System.Environment.TickCount : config.RandomSeed;
        roadMap.RandomSeed = seed;
        random = new System.Random(seed);
        
        Debug.Log($"Generating road network with seed: {seed}");

        // Step 1: Create grid nodes with jitter
        CreateGridNodes();
        
        // Step 2: Build road topology (connections only, no geometry yet)
        List<RoadConnection> connections = CreateRoadConnections();
        
        // Step 2.5: Randomly remove nodes and roads for organic variation
        RemoveRandomNodes();
        connections = RemoveInvalidConnections(connections);
        connections = RemoveRandomRoads(connections);
        connections = RemoveDeadEndNodes(connections);
        
        // Step 2.6: Remove roads that meet at very sharp angles (< 15 degrees)
        connections = RemoveSharpAngleRoads(connections);
        
        // Step 2.7: Create bridges for remaining crossings (after all removals)
        CreateBridges(connections);
        
        // Step 3: Analyze all intersections and calculate stop distances
        Dictionary<(int, int), float> stopDistances = CalculateAllStopDistances(connections);
        
        // Step 4: Create roads with lanes using pre-computed stop distances
        CreateRoadsWithLanes(connections, stopDistances);
        
        // Step 5: Precompute turn curves
        PrecomputeTurnCurves();
        
        Debug.Log($"Generated road network: {roadMap.Nodes.Count} nodes, {roadMap.Roads.Count} roads");
        
        return roadMap;
    }

    private void CreateGridNodes()
    {
        int nodeId = 0;
        
        // NOTE: Small grids (3x3) with high jitter (>5) may cause lane overlap at intersections
        // due to wrap-around distance limitations. Use 4x4+ grids for jitter values above 5.
        
        // Calculate grid positions with consistent spacing for more regular appearance
        float[] xPositions = new float[config.GridWidth];
        float[] zPositions = new float[config.GridHeight];
        
        // Use average spacing with small variation per segment
        float avgSpacingX = (config.MinSegmentLength + config.MaxSegmentLength) * 0.5f;
        float avgSpacingZ = (config.MinSegmentLength + config.MaxSegmentLength) * 0.5f;
        
        float currentX = 0;
        for (int x = 0; x < config.GridWidth; x++)
        {
            xPositions[x] = currentX;
            // Add small variation (20% of range) to spacing
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
        
        // Create nodes at grid positions with reduced jitter (50% of configured value)
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                // Apply reduced jitter for more regular grid
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
                
                if (!ConnectionExists(connections, current, next) &&
                    (!isWrapAround || IsReasonableWrapDistance(current, next)))
                {
                    RoadConnection conn = new RoadConnection(roadId++, current, next);
                    
                    // Check for crossing and try to create bridge or reject
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
                else if (!ConnectionExists(connections, current, next) && isWrapAround && !IsReasonableWrapDistance(current, next))
                {
                    Debug.LogWarning($"Skipping wrap-around road from Node {current.ID} to {next.ID}: distance too large ({Vector3.Distance(current.Position, next.Position):F1})");
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
                
                if (!ConnectionExists(connections, current, next) &&
                    (!isWrapAround || IsReasonableWrapDistance(current, next)))
                {
                    RoadConnection conn = new RoadConnection(roadId++, current, next);
                    
                    // Check for crossing and try to create bridge or reject
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
                else if (!ConnectionExists(connections, current, next) && isWrapAround && !IsReasonableWrapDistance(current, next))
                {
                    Debug.LogWarning($"Skipping wrap-around road from Node {current.ID} to {next.ID}: distance too large ({Vector3.Distance(current.Position, next.Position):F1})");
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
            
            if (DiagonalPassesTooCloseToNodes(node1, node2, grid))
                continue;
            
            // Check if this diagonal would create sharp angles at either endpoint
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
    
    private bool DiagonalPassesTooCloseToNodes(Node start, Node end, Node[,] grid)
    {
        // Check if the line from start to end passes too close to any other node
        float minDistance = config.MinDistanceFromIntersection;
        
        for (int x = 0; x < config.GridWidth; x++)
        {
            for (int z = 0; z < config.GridHeight; z++)
            {
                Node node = grid[x, z];
                
                // Skip the endpoints of the diagonal
                if (node == start || node == end)
                    continue;
                
                // Calculate distance from node to line segment
                float distToSegment = DistanceFromPointToLineSegment(node.Position, start.Position, end.Position);
                
                if (distToSegment < minDistance)
                {
                    return true; // Too close
                }
            }
        }
        
        return false;
    }
    
    private float DistanceFromPointToLineSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        // Calculate distance from point to line segment on XZ plane
        Vector3 lineDir = lineEnd - lineStart;
        float lineLength = lineDir.magnitude;
        
        if (lineLength < 0.001f)
            return Vector3.Distance(point, lineStart);
        
        lineDir /= lineLength;
        
        // Project point onto line
        float t = Vector3.Dot(point - lineStart, lineDir);
        
        // Clamp to segment
        t = Mathf.Clamp(t, 0f, lineLength);
        
        // Find closest point on segment
        Vector3 closestPoint = lineStart + lineDir * t;
        
        // Return XZ distance (ignore Y)
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
        // Check if a wrap-around connection distance is reasonable
        // If nodes have shifted too much due to jitter, wrap-around creates very long roads
        float distance = Vector3.Distance(from.Position, to.Position);
        float maxReasonableDistance = config.MaxSegmentLength * 1.5f;
        
        return distance <= maxReasonableDistance;
    }
    
    private bool TryHandleRoadCrossing(ref RoadConnection newConn, List<RoadConnection> existingConnections)
    {
        // Check if this road would cross any existing roads
        // NOTE: We don't create bridges here - that happens later after all removals
        Vector3 p1 = newConn.StartNode.Position;
        Vector3 p2 = newConn.EndNode.Position;
        
        // Find ALL crossings
        foreach (var existingConn in existingConnections)
        {
            // Skip if roads share a node
            if (existingConn.StartNode == newConn.StartNode || existingConn.StartNode == newConn.EndNode ||
                existingConn.EndNode == newConn.StartNode || existingConn.EndNode == newConn.EndNode)
            {
                continue;
            }
            
            Vector3 p3 = existingConn.StartNode.Position;
            Vector3 p4 = existingConn.EndNode.Position;
            
            if (LineSegmentsIntersect(p1, p2, p3, p4, out Vector3 crossingPoint))
            {
                // Roads cross - check if we CAN create a bridge (when bridges are created later)
                if (!config.EnableBridges || !CanCreateBridge(newConn, existingConn, crossingPoint))
                {
                    return false; // Cannot create bridge, reject road
                }
                // Don't create bridge yet - just validate that it's possible
            }
        }
        
        return true; // All crossings can be bridged, road is valid
    }
    
    private bool CanCreateBridge(RoadConnection newConn, RoadConnection existingConn, Vector3 crossingPoint)
    {
        // Check condition 1: Not too close to intersections
        float distToStart = Vector3.Distance(newConn.StartNode.Position, crossingPoint);
        float distToEnd = Vector3.Distance(newConn.EndNode.Position, crossingPoint);
        
        if (distToStart < config.MinDistanceFromIntersection || distToEnd < config.MinDistanceFromIntersection)
        {
            return false; // Too close to intersection
        }
        
        // Check condition 2: Crossing angle not too shallow
        Vector3 dir1 = (newConn.EndNode.Position - newConn.StartNode.Position).normalized;
        Vector3 dir2 = (existingConn.EndNode.Position - existingConn.StartNode.Position).normalized;
        
        float dot = Mathf.Abs(Vector3.Dot(dir1, dir2));
        float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, 0f, 1f)) * Mathf.Rad2Deg;
        
        if (angleDegrees < config.MinBridgeCrossingAngle)
        {
            return false; // Angle too shallow
        }
        
        // Check condition 3: Road long enough for smooth ramp
        float requiredLength = config.MinBridgeRampLength * 2f;
        
        if (distToStart < config.MinBridgeRampLength || distToEnd < config.MinBridgeRampLength)
        {
            return false; // Not enough room for ramps
        }
        
        return true;
    }
    
    private void CreateBridges(List<RoadConnection> connections)
    {
        // After all removals, create bridges for all remaining road crossings
        if (!config.EnableBridges)
            return;
        
        int bridgeCount = 0;
        
        for (int i = 0; i < connections.Count; i++)
        {
            RoadConnection conn = connections[i];
            Vector3 p1 = conn.StartNode.Position;
            Vector3 p2 = conn.EndNode.Position;
            
            // Check against all other roads (only check j > i to avoid duplicate bridge creation)
            for (int j = i + 1; j < connections.Count; j++)
            {
                RoadConnection other = connections[j];
                
                // Skip if roads share a node
                if (other.StartNode == conn.StartNode || other.StartNode == conn.EndNode ||
                    other.EndNode == conn.StartNode || other.EndNode == conn.EndNode)
                {
                    continue;
                }
                
                Vector3 p3 = other.StartNode.Position;
                Vector3 p4 = other.EndNode.Position;
                
                if (LineSegmentsIntersect(p1, p2, p3, p4, out Vector3 crossingPoint))
                {
                    // Roads cross - check if we can create a bridge
                    // Only create bridge on ONE road (arbitrarily choose the first one)
                    if (CanCreateBridge(conn, other, crossingPoint))
                    {
                        AddBridge(ref conn, crossingPoint);
                        connections[i] = conn; // Update the struct in the list
                        bridgeCount++;
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
        // Calculate distances
        float distFromStart = Vector3.Distance(conn.StartNode.Position, crossingPoint);
        float distFromEnd = Vector3.Distance(conn.EndNode.Position, crossingPoint);
        
        // Use configured ramp length
        float rampLength = config.MinBridgeRampLength;
        
        // Create bridge that spans from (crossing - rampLength) to (crossing + rampLength)
        float startDist = distFromStart - rampLength;
        float endDist = distFromStart + rampLength;
        
        // Ensure we don't go beyond road bounds
        startDist = Mathf.Max(0f, startDist);
        endDist = Mathf.Min(conn.Length, endDist);
        
        BridgeInfo bridge = new BridgeInfo(
            crossingPoint,
            startDist,
            endDist,
            config.BridgeHeight,
            rampLength
        );
        
        // Add bridge to the list
        conn.Bridges.Add(bridge);
    }
    
    private Dictionary<(int, int), float> CalculateAllStopDistances(List<RoadConnection> connections)
    {
        // Pass 2: Analyze all intersections with full network knowledge
        Dictionary<(int, int), float> stopDistances = new Dictionary<(int, int), float>();
        
        foreach (var conn in connections)
        {
            // Calculate stop distance at start node
            float stopDistStart = CalculateStopDistanceAtNode(conn.StartNode, conn.EndNode, connections);
            stopDistances[(conn.ID, conn.StartNode.ID)] = stopDistStart;
            
            // Calculate stop distance at end node
            float stopDistEnd = CalculateStopDistanceAtNode(conn.EndNode, conn.StartNode, connections);
            stopDistances[(conn.ID, conn.EndNode.ID)] = stopDistEnd;
        }
        
        return stopDistances;
    }
    
    private float CalculateStopDistanceAtNode(Node atNode, Node towardNode, List<RoadConnection> connections)
    {
        // Calculate required stop line distance based on sharpest angle at this node
        // Uses full network knowledge to compute symmetric adjustments
        
        float baseDistance = config.StopLineDistance;
        
        // Direction of this road
        Vector3 thisRoadDir = (towardNode.Position - atNode.Position).normalized;
        
        float minAngleDegrees = 180f; // Start with straight (180°)
        
        // Check angle with all OTHER connections at this node
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
            
            // Calculate angle between roads
            float dot = Vector3.Dot(thisRoadDir, otherDir);
            float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;
            
            minAngleDegrees = Mathf.Min(minAngleDegrees, angleDegrees);
        }
        
        // Dynamically calculate stop line distance based on angle to prevent lane overlap
        // Using geometric formula: distance = (roadWidth / 2) / tan(angle / 2)
        // This ensures lanes don't overlap even at very sharp angles
        if (minAngleDegrees < 90f)
        {
            // Total road width (both lanes)
            float roadWidth = config.LaneWidth * 2f;
            
            // Convert to radians
            float angleRadians = minAngleDegrees * Mathf.Deg2Rad;
            
            // Calculate required distance using geometry
            // Add modest safety margin of 1.15x to account for turn curves
            float halfAngle = angleRadians / 2f;
            float geometricDistance = (roadWidth / 2f) / Mathf.Tan(halfAngle);
            float requiredDistance = geometricDistance * 1.15f;
            
            // Cap maximum cutoff to prevent overly aggressive cutbacks at very sharp angles
            float maxCutoff = config.StopLineDistance * 3f; // Max 3x the base distance
            requiredDistance = Mathf.Min(requiredDistance, maxCutoff);
            
            // Use the larger of base distance or calculated distance
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
        
        // Check if two line segments intersect on the XZ plane
        // Using parametric line segment intersection test
        
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
        
        // Intersection occurs if both t and u are between 0 and 1
        // Use small epsilon to avoid rejecting roads that barely touch at endpoints
        const float epsilon = 0.01f;
        bool intersects = (t > epsilon && t < (1.0f - epsilon) && u > epsilon && u < (1.0f - epsilon));
        
        if (intersects)
        {
            // Calculate crossing point
            crossingPoint = Vector3.Lerp(p1, p2, t);
        }
        
        return intersects;
    }

    private void PrecomputeTurnCurves()
    {
        // For each node (intersection), find all possible turns
        foreach (Node node in roadMap.Nodes)
        {
            // Get all lanes ending at this node
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
            
            // Check if this is a trivial intersection (only 2 roads)
            // Trivial intersections must allow their only turn regardless of angle
            bool isTrivial = node.IsTrivialIntersection();
            
            // Create turn curves for all valid combinations (excluding U-turns and sharp turns)
            foreach (Lane inLane in incomingLanes)
            {
                foreach (Lane outLane in outgoingLanes)
                {
                    // Prevent U-turn (going back on the same road)
                    if (IsUTurn(inLane, outLane))
                        continue;
                    
                    // Prevent turns that are too sharp (< 40 degrees)
                    // EXCEPT for trivial intersections where this is the only path
                    if (!isTrivial && IsTurnTooSharp(inLane, outLane))
                        continue;
                    
                    TurnCurve curve = new TurnCurve(inLane, outLane, config.TurnRadius);
                    inLane.AvailableTurns.Add(curve);
                }
            }
        }
    }

    private bool IsUTurn(Lane inLane, Lane outLane)
    {
        // Check if the outgoing lane goes back to where the incoming lane came from
        return outLane.EndNode == inLane.StartNode;
    }
    
    private bool IsTurnTooSharp(Lane inLane, Lane outLane)
    {
        // Check if the turn angle is too sharp (> 140 degrees turn required)
        // This corresponds to roads meeting at < 40 degrees
        const float MAX_TURN_ANGLE = 140f; // degrees
        
        Vector3 incomingDir = inLane.GetDirection();
        Vector3 outgoingDir = outLane.GetDirection();
        
        // Calculate angle between directions
        float dot = Vector3.Dot(incomingDir, outgoingDir);
        float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;
        
        // Sharp turn = large angle change (close to 180° = U-turn)
        return angleDegrees > MAX_TURN_ANGLE;
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
                
                // Mark border nodes
                if (x == 0 || x == config.GridWidth - 1 || z == 0 || z == config.GridHeight - 1)
                {
                    borderNodes.Add(grid[x, z]);
                }
            }
        }
        
        // Get interior nodes that can be removed
        List<Node> removableNodes = new List<Node>();
        foreach (Node node in roadMap.Nodes)
        {
            if (!borderNodes.Contains(node))
            {
                removableNodes.Add(node);
            }
        }
        
        // Calculate how many to remove
        int nodesToRemove = Mathf.RoundToInt(removableNodes.Count * config.NodeRemovalPercentage / 100f);
        
        // Randomly select and remove nodes
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
        // Remove connections that reference deleted nodes
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
        
        // Calculate how many roads to try removing
        int roadsToRemove = Mathf.RoundToInt(connections.Count * config.RoadRemovalPercentage / 100f);
        
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        int removedCount = 0;
        
        // Try to remove roads while maintaining connectivity and minimum node degree
        for (int i = 0; i < roadsToRemove && mutableConnections.Count > 1; i++)
        {
            int randomIndex = random.Next(mutableConnections.Count);
            RoadConnection candidate = mutableConnections[randomIndex];
            
            // Test if network stays valid without this road
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
        // Count connections per node
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
        
        // Check all nodes have at least 2 connections
        foreach (var kvp in nodeDegree)
        {
            if (kvp.Value < 2)
            {
                return false; // Node has fewer than 2 roads
            }
        }
        
        return true;
    }
    
    private List<RoadConnection> RemoveDeadEndNodes(List<RoadConnection> connections)
    {
        // Remove any nodes that have only 1 connection (dead ends)
        // Keep iterating until no more dead ends exist
        
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        bool removedAny = true;
        int totalRemoved = 0;
        
        while (removedAny)
        {
            removedAny = false;
            
            // Count connections per node
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
            
            // Find and remove nodes with degree 1
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
                // Remove these nodes
                foreach (Node node in deadEndNodes)
                {
                    roadMap.Nodes.Remove(node);
                }
                
                // Remove connections involving these nodes
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
        // Remove roads that meet at angles less than 20 degrees at intersections
        const float MIN_ANGLE_DEGREES = 20f;
        
        List<RoadConnection> mutableConnections = new List<RoadConnection>(connections);
        int totalRemoved = 0;
        bool removedAny = true;
        
        while (removedAny)
        {
            removedAny = false;
            
            // Check each node for sharp angles between connected roads
            foreach (Node node in roadMap.Nodes)
            {
                // Get all roads connected to this node
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
                
                // Check all pairs of roads at this node
                for (int i = 0; i < connectedRoads.Count; i++)
                {
                    for (int j = i + 1; j < connectedRoads.Count; j++)
                    {
                        RoadConnection road1 = connectedRoads[i];
                        RoadConnection road2 = connectedRoads[j];
                        
                        // Get directions from the node
                        Vector3 dir1 = road1.StartNode == node 
                            ? (road1.EndNode.Position - node.Position).normalized
                            : (road1.StartNode.Position - node.Position).normalized;
                        
                        Vector3 dir2 = road2.StartNode == node
                            ? (road2.EndNode.Position - node.Position).normalized
                            : (road2.StartNode.Position - node.Position).normalized;
                        
                        // Calculate angle
                        float dot = Vector3.Dot(dir1, dir2);
                        float angleDegrees = Mathf.Acos(Mathf.Clamp(dot, -1f, 1f)) * Mathf.Rad2Deg;
                        
                        // If angle is too sharp, remove one of the roads
                        if (angleDegrees < MIN_ANGLE_DEGREES)
                        {
                            // Remove the shorter road to minimize impact
                            RoadConnection toRemove = road1.Length < road2.Length ? road1 : road2;
                            
                            // Check if removing this road would disconnect the network or create dead ends
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
    
    private bool IsNetworkConnected(List<RoadConnection> connections)
    {
        if (roadMap.Nodes.Count == 0 || connections.Count == 0)
            return false;
        
        // Build adjacency list
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
        
        // Network is connected if all nodes were visited
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

    // ============================================================================
    // EXPORT METHODS FOR PYTHON VISUALIZATION
    // ============================================================================

    /// <summary>
    /// Export the road network to JSON file for Python visualization.
    /// Right-click in Unity Inspector and select "Export Road Network to JSON" to use.
    /// </summary>
    [ContextMenu("Export Road Network to JSON")]
    public void ExportToJSONFromContextMenu()
    {
        string path = UnityEngine.Application.dataPath + "/../PythonAnalysis/road_network_export.json";
        ExportToJSON(path);
    }

    /// <summary>
    /// Export the complete road network to a JSON file.
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

        // Export nodes
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

        // Export roads and lanes
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

            // Export forward lane and its turn curves
            export.Lanes.Add(ExportLane(road.ForwardLane, export.Bridges));
            turnCurveID = ExportLaneTurnCurves(road.ForwardLane, export.TurnCurves, turnCurveID);

            // Export backward lane and its turn curves
            export.Lanes.Add(ExportLane(road.BackwardLane, export.Bridges));
            turnCurveID = ExportLaneTurnCurves(road.BackwardLane, export.TurnCurves, turnCurveID);
        }

        string json = UnityEngine.JsonUtility.ToJson(export, prettyPrint: true);

        try
        {
            // Ensure directory exists
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

        // Export bridges (deduplicated - same bridge object shared by both lanes)
        if (lane.Bridges != null && lane.Bridges.Count > 0)
        {
            foreach (var bridge in lane.Bridges)
            {
                // Check if this bridge was already exported
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
                    // Bridge already exported, just reference it
                    le.BridgeIndices.Add(existingIndex);
                }
                else
                {
                    // Export new bridge
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


