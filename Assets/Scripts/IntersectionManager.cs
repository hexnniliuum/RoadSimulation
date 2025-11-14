using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Manages intersection states, turn conflicts, and right-of-way rules.
/// Implements "priority to the right" traffic logic.
/// </summary>
public class IntersectionManager
{
    // State tracking per node
    private Dictionary<Node, IntersectionState> intersectionStates;
    
    // Pre-computed conflict map: which turn curves intersect
    private Dictionary<TurnCurve, HashSet<TurnCurve>> turnConflicts;
    
    // Configuration
    private float deadlockTimeout = 5f;
    
    /// <summary>
    /// Initialize intersection management system with road network.
    /// </summary>
    public void Initialize(RoadMap roadMap)
    {
        intersectionStates = new Dictionary<Node, IntersectionState>();
        turnConflicts = new Dictionary<TurnCurve, HashSet<TurnCurve>>();
        
        // Debug.Log("IntersectionManager: Initializing...");
        
        // Build intersection states for each node with 2+ roads
        foreach (var node in roadMap.Nodes)
        {
            if (node.ConnectedRoads.Count >= 2)
            {
                IntersectionState state = new IntersectionState(node);
                intersectionStates[node] = state;
                
                // Debug.Log($"Intersection created at Node {node.ID} with {node.ConnectedRoads.Count} roads");
            }
        }
        
        // Pre-compute turn curve conflicts
        BuildTurnConflictMap();
        
        Debug.Log($"[IntersectionManager] Initialized {intersectionStates.Count} intersections, {turnConflicts.Count} turn curves analyzed");
    }
    
    /// <summary>
    /// Check if a car can enter an intersection based on conflicts and priority rules.
    /// </summary>
    public bool CanEnter(CarController car, Lane fromLane, TurnCurve intendedTurn, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            Debug.LogWarning($"[DEADLOCK] IntersectionManager: No state for intersection at Node {intersection.ID}");
            return true; // Not a managed intersection
        }
        
        var state = intersectionStates[intersection];
        
        // Check 1: Must be first in queue for this lane
        if (state.waitingQueues.ContainsKey(fromLane))
        {
            var queue = state.waitingQueues[fromLane];
            if (queue.Count > 0 && queue.Peek() != car)
            {
                // Debug: Not first in queue
                return false; // Not your turn yet
            }
        }
        
        // Check 2: DEADLOCK TIMEOUT - check FIRST to break circular deadlocks
        // This must be before priority/conflict checks to be effective
        if (state.waitingQueues.ContainsKey(fromLane))
        {
            var queue = state.waitingQueues[fromLane];
            if (queue.Count > 0 && queue.Peek() == car)
            {
                // Check if this car has been waiting too long
                float waitTime = Time.time - state.GetWaitStartTime(car);
                
                // Log wait time periodically for debugging (only for Node 7)
                if (intersection.ID == 7 && waitTime > 2f && Mathf.FloorToInt(waitTime) != Mathf.FloorToInt(waitTime - Time.deltaTime))
                {
                    Debug.Log($"[NODE-7] Car {car.CarID}: Waiting {waitTime:F1}s (timeout at {deadlockTimeout}s)");
                    LogIntersectionState(intersection, state);
                }
                
                if (waitTime > deadlockTimeout)
                {
                    if (intersection.ID == 7)
                        Debug.LogWarning($"[NODE-7] Car {car.CarID}: FORCING ENTRY after {waitTime:F1}s timeout");
                    else
                        Debug.LogWarning($"[DEADLOCK-BREAK] Car {car.CarID} at Node {intersection.ID}: Forcing entry");
                    return true; // Force entry to break deadlock - SKIP all other checks
                }
            }
        }
        
        // Check 3: Path conflict - does intended turn overlap with active crossings?
        foreach (var activeKvp in state.activeCrossings)
        {
            TurnCurve activeTurn = activeKvp.Value;
            
            if (turnConflicts.ContainsKey(intendedTurn) && 
                turnConflicts[intendedTurn].Contains(activeTurn))
            {
                if (intersection.ID == 7)
                    Debug.Log($"[NODE-7] Car {car.CarID}: BLOCKED by path conflict with Car {activeKvp.Key.CarID}");
                return false; // Path overlaps with active vehicle
            }
        }
        
        // Check 4: Priority to right
        if (HasPriorityVehicleOnRight(car, fromLane, intersection, out int blockingCarID))
        {
            if (intersection.ID == 7)
                Debug.Log($"[NODE-7] Car {car.CarID}: BLOCKED by priority-to-right (Car {blockingCarID})");
            return false; // Vehicle on right has priority
        }
        
        return true;
    }
    
    /// <summary>
    /// Register a car entering an intersection.
    /// </summary>
    public void RegisterEntry(CarController car, Node intersection, TurnCurve turn)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            Debug.LogWarning($"[IntersectionManager] Cannot register entry - no state for Node {intersection.ID}");
            return;
        }
        
        var state = intersectionStates[intersection];
        state.activeCrossings[car] = turn;
        
        // Debug.Log($"Car {car.CarID}: Entered intersection at Node {intersection.ID}");
    }
    
    /// <summary>
    /// Register a car exiting an intersection.
    /// </summary>
    public void RegisterExit(CarController car, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            Debug.LogWarning($"[IntersectionManager] Cannot register exit - no state for Node {intersection.ID}");
            return;
        }
        
        var state = intersectionStates[intersection];
        state.activeCrossings.Remove(car);
        
        // Debug.Log($"Car {car.CarID}: Exited intersection at Node {intersection.ID}");
    }
    
    /// <summary>
    /// Add a car to the waiting queue for an intersection approach.
    /// </summary>
    public void UpdateCarWaitingAt(CarController car, Lane lane, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return;
        }
        
        var state = intersectionStates[intersection];
        
        if (!state.waitingQueues.ContainsKey(lane))
        {
            state.waitingQueues[lane] = new Queue<CarController>();
        }
        
        // Only add if not already in queue
        if (!state.waitingQueues[lane].Contains(car))
        {
            state.waitingQueues[lane].Enqueue(car);
            state.RecordWaitStartTime(car, Time.time);
            // Debug.Log($"Car {car.CarID}: Added to waiting queue at Node {intersection.ID}, lane {lane.ID}");
        }
    }
    
    /// <summary>
    /// Remove a car from the waiting queue.
    /// </summary>
    public void RemoveCarWaitingAt(CarController car, Lane lane, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return;
        }
        
        var state = intersectionStates[intersection];
        
        if (state.waitingQueues.ContainsKey(lane))
        {
            var queue = state.waitingQueues[lane];
            if (queue.Count > 0 && queue.Peek() == car)
            {
                queue.Dequeue();
                state.ClearWaitStartTime(car);
                // Debug.Log($"Car {car.CarID}: Removed from waiting queue at Node {intersection.ID}");
            }
        }
    }
    
    /// <summary>
    /// Log detailed intersection state for debugging deadlocks (Node 7 only).
    /// </summary>
    private void LogIntersectionState(Node intersection, IntersectionState state)
    {
        if (intersection.ID != 7) return; // Only log Node 7
        
        Debug.Log($"[NODE-7] === STATE: {state.incomingLanes.Count} incoming lanes ===");
        
        // Log active crossings
        Debug.Log($"[NODE-7] Active crossings: {state.activeCrossings.Count}");
        foreach (var crossing in state.activeCrossings)
        {
            Debug.Log($"[NODE-7]   - Car {crossing.Key.CarID} (state: {crossing.Key.CurrentCarState}) from Lane {crossing.Value.FromLane.ID} to Lane {crossing.Value.ToLane.ID}");
        }
        
        // Log waiting queues
        int totalWaiting = 0;
        foreach (var queueKvp in state.waitingQueues)
        {
            if (queueKvp.Value.Count > 0)
            {
                totalWaiting += queueKvp.Value.Count;
                var queueList = new System.Collections.Generic.List<CarController>(queueKvp.Value);
                string carList = string.Join(", ", queueList.ConvertAll(c => $"Car{c.CarID}({c.CurrentCarState})"));
                Debug.Log($"[NODE-7] Waiting on Lane {queueKvp.Key.ID}: {queueKvp.Value.Count} cars [{carList}]");
            }
        }
        
        if (totalWaiting == 0)
        {
            Debug.Log($"[NODE-7] No cars waiting");
        }
    }
    
    /// <summary>
    /// Check if there's a vehicle on the right with priority.
    /// Uses cross product to determine spatial relationship.
    /// ONLY checks cars still waiting (not already crossing).
    /// </summary>
    private bool HasPriorityVehicleOnRight(CarController requestingCar, Lane fromLane, Node node, out int blockingCarID)
    {
        blockingCarID = -1;
        var state = intersectionStates[node];
        
        Vector3 carDirection = (fromLane.EndPosition - fromLane.StartPosition).normalized;
        
        foreach (var incomingLane in state.incomingLanes)
        {
            if (incomingLane == fromLane) continue;
            
            // Get direction of other lane
            Vector3 otherDir = (incomingLane.EndPosition - incomingLane.StartPosition).normalized;
            
            // Cross product: NEGATIVE Y means other lane is to our right (clockwise)
            // Positive Y means left (counter-clockwise)
            float cross = Vector3.Cross(carDirection, otherDir).y;
            
            if (cross < -0.1f) // On right side (negative = clockwise)
            {
                // CRITICAL FIX: Only check cars WAITING, not those already crossing
                // Cars already in the intersection are handled by path conflict checks
                if (state.waitingQueues.ContainsKey(incomingLane))
                {
                    var queue = state.waitingQueues[incomingLane];
                    if (queue.Count > 0)
                    {
                        // Make sure this car is actually first in their queue (ready to go)
                        // Don't block for cars behind in queue
                        CarController carOnRight = queue.Peek();
                        
                        // Additional check: don't block if that car is already crossing
                        // (it may still be in queue but transitioning)
                        if (carOnRight != null && 
                            carOnRight.CurrentCarState != CarController.CarState.Crossing)
                        {
                            blockingCarID = carOnRight.CarID;
                            return true; // Vehicle on right has priority
                        }
                    }
                }
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// Pre-compute which turn curves conflict (intersect spatially).
    /// </summary>
    private void BuildTurnConflictMap()
    {
        // Collect all turn curves from all intersections
        List<TurnCurve> allTurns = new List<TurnCurve>();
        
        foreach (var state in intersectionStates.Values)
        {
            foreach (var lane in state.incomingLanes)
            {
                allTurns.AddRange(lane.AvailableTurns);
            }
        }
        
        // Debug.Log($"IntersectionManager: Analyzing {allTurns.Count} turn curves for conflicts");
        
        // Check each pair of turns at the same intersection
        for (int i = 0; i < allTurns.Count; i++)
        {
            TurnCurve turnA = allTurns[i];
            
            if (!turnConflicts.ContainsKey(turnA))
            {
                turnConflicts[turnA] = new HashSet<TurnCurve>();
            }
            
            for (int j = i + 1; j < allTurns.Count; j++)
            {
                TurnCurve turnB = allTurns[j];
                
                // Only check turns at the same intersection
                if (turnA.FromLane.EndNode != turnB.FromLane.EndNode)
                {
                    continue;
                }
                
                // Check if they conflict
                if (TurnCurvesConflict(turnA, turnB))
                {
                    turnConflicts[turnA].Add(turnB);
                    
                    if (!turnConflicts.ContainsKey(turnB))
                    {
                        turnConflicts[turnB] = new HashSet<TurnCurve>();
                    }
                    turnConflicts[turnB].Add(turnA);
                    
                    // Debug.Log($"Conflict detected: Turn {turnA.FromLane.ID}->{turnA.ToLane.ID} conflicts with {turnB.FromLane.ID}->{turnB.ToLane.ID}");
                }
            }
        }
    }
    
    /// <summary>
    /// Check if two turn curves conflict using line segment intersection.
    /// </summary>
    private bool TurnCurvesConflict(TurnCurve a, TurnCurve b)
    {
        // Sample points along both curves
        List<Vector3> pointsA = SampleCurve(a, 10);
        List<Vector3> pointsB = SampleCurve(b, 10);
        
        // Check each segment pair for intersection
        for (int i = 0; i < pointsA.Count - 1; i++)
        {
            for (int j = 0; j < pointsB.Count - 1; j++)
            {
                // Check 2D intersection (XZ plane)
                if (LineSegmentsIntersect2D(pointsA[i], pointsA[i+1], pointsB[j], pointsB[j+1]))
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// Sample points along a turn curve.
    /// </summary>
    private List<Vector3> SampleCurve(TurnCurve curve, int samples)
    {
        List<Vector3> points = new List<Vector3>();
        
        for (int i = 0; i <= samples; i++)
        {
            float t = i / (float)samples;
            points.Add(curve.GetPositionAtTime(t));
        }
        
        return points;
    }
    
    /// <summary>
    /// Check if two line segments intersect in 2D (XZ plane).
    /// </summary>
    private bool LineSegmentsIntersect2D(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
    {
        float x1 = p1.x, z1 = p1.z;
        float x2 = p2.x, z2 = p2.z;
        float x3 = p3.x, z3 = p3.z;
        float x4 = p4.x, z4 = p4.z;
        
        float denom = (x1 - x2) * (z3 - z4) - (z1 - z2) * (x3 - x4);
        
        if (Mathf.Abs(denom) < 0.0001f)
        {
            return false; // Parallel or coincident
        }
        
        float t = ((x1 - x3) * (z3 - z4) - (z1 - z3) * (x3 - x4)) / denom;
        float u = -((x1 - x2) * (z1 - z3) - (z1 - z2) * (x1 - x3)) / denom;
        
        // Check if intersection point is within both segments
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            return true;
        }
        
        return false;
    }
}

/// <summary>
/// State tracking for a single intersection.
/// </summary>
public class IntersectionState
{
    public Node Node;
    
    // Cars currently crossing through intersection (mapped to their turn curve)
    public Dictionary<CarController, TurnCurve> activeCrossings;
    
    // Cars waiting to enter, organized by incoming lane
    public Dictionary<Lane, Queue<CarController>> waitingQueues;
    
    // Cached list of incoming lanes for this intersection
    public List<Lane> incomingLanes;
    
    // Track wait start times for deadlock detection
    private Dictionary<CarController, float> waitStartTimes;
    
    public IntersectionState(Node node)
    {
        Node = node;
        activeCrossings = new Dictionary<CarController, TurnCurve>();
        waitingQueues = new Dictionary<Lane, Queue<CarController>>();
        incomingLanes = new List<Lane>();
        waitStartTimes = new Dictionary<CarController, float>();
        
        // Build list of incoming lanes (lanes that end at this node)
        foreach (var road in node.ConnectedRoads)
        {
            if (road.ForwardLane.EndNode == node)
            {
                incomingLanes.Add(road.ForwardLane);
            }
            if (road.BackwardLane.EndNode == node)
            {
                incomingLanes.Add(road.BackwardLane);
            }
        }
    }
    
    public void RecordWaitStartTime(CarController car, float time)
    {
        if (!waitStartTimes.ContainsKey(car))
        {
            waitStartTimes[car] = time;
        }
    }
    
    public float GetWaitStartTime(CarController car)
    {
        return waitStartTimes.ContainsKey(car) ? waitStartTimes[car] : Time.time;
    }
    
    public void ClearWaitStartTime(CarController car)
    {
        waitStartTimes.Remove(car);
    }
}

