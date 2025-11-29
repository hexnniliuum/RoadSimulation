using System.Collections.Generic;
using UnityEngine;
using System.Linq;

/// <summary>
/// Manages intersection states, turn conflicts, and right-of-way rules.
/// Implements "priority to the right" traffic logic.
/// </summary>
public class IntersectionManager
{
    // State tracking per node
    private Dictionary<Node, IntersectionState> intersectionStates;
    
    // Traffic lights
    private Dictionary<Node, StopLightController> stopLights;
    
    // Pre-computed conflict map: which turn curves intersect
    private Dictionary<TurnCurve, HashSet<TurnCurve>> turnConflicts;
    
    // Configuration
    private float deadlockTimeout = 3f;
    
    // Deadlock breaking: track which car is currently allowed to force entry
    private Dictionary<Node, CarController> carBreakingDeadlock;
    
    /// <summary>
    /// Initialize intersection management system with road network.
    /// </summary>
    public void Initialize(RoadMap roadMap, RoadGenerationConfig config)
    {
        intersectionStates = new Dictionary<Node, IntersectionState>();
        stopLights = new Dictionary<Node, StopLightController>();
        turnConflicts = new Dictionary<TurnCurve, HashSet<TurnCurve>>();
        carBreakingDeadlock = new Dictionary<Node, CarController>();
        
        // Debug.Log("IntersectionManager: Initializing...");
        
        // Build intersection states for each node with 2+ roads
        foreach (var node in roadMap.Nodes)
        {
            if (node.ConnectedRoads.Count >= 2)
            {
                IntersectionState state = new IntersectionState(node);
                intersectionStates[node] = state;
                carBreakingDeadlock[node] = null; // Initialize with no breaker
            }
        }

        // Determine which nodes get traffic lights based on configuration
        // Candidates: Non-trivial intersections (more than 2 roads)
        var candidateNodes = roadMap.Nodes
            .Where(n => !n.IsTrivialIntersection() && n.ConnectedRoads.Count >= 2)
            .OrderByDescending(n => n.ConnectedRoads.Count) // Prioritize highest connectivity
            .ThenBy(n => n.ID) // Deterministic tie-breaking
            .ToList();

        // Calculate count based on percentage (0-100)
        float percentage = Mathf.Clamp(config.TrafficLightPercentage, 0f, 100f);
        int targetCount = Mathf.RoundToInt(candidateNodes.Count * (percentage / 100f));
        
        // Create stop lights for the top N nodes
        for (int i = 0; i < targetCount; i++)
        {
            Node node = candidateNodes[i];
            stopLights[node] = new StopLightController(node);
        }
        
        // Pre-compute turn curve conflicts
        BuildTurnConflictMap();
        
        Debug.Log($"[IntersectionManager] Initialized {intersectionStates.Count} intersections. Traffic Lights: {stopLights.Count}/{candidateNodes.Count} ({percentage:F0}%)");
    }
    
    /// <summary>
    /// Update traffic lights.
    /// </summary>
    public void Update(float deltaTime)
    {
        foreach (var light in stopLights.Values)
        {
            light.Update(deltaTime);
        }
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
        
        // Check 0: Traffic Lights (Overrides everything else)
        if (stopLights.ContainsKey(intersection))
        {
            var lightState = stopLights[intersection].GetLightState(fromLane);
            if (lightState == StopLightController.LightState.Red)
            {
                return false; // Stop on Red
            }
            if (lightState == StopLightController.LightState.Yellow)
            {
                // For simplicity, treat Yellow as Red (stop if you can)
                // In a real sim, we'd check distance, but here we want to clear the intersection
                return false; 
            }
            // Green light: Proceed to other checks (still need to check for blocked intersection)
        }
        
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
        
        // Check 2: DEADLOCK DETECTION - intersection-level, not per-car
        // Only trigger if ALL cars at intersection are stuck
        CheckAndBreakDeadlock(intersection, state, car);
        
        // Check 2.5: Is this the car chosen to break deadlock?
        if (carBreakingDeadlock.ContainsKey(intersection) && carBreakingDeadlock[intersection] == car)
        {
            // if (intersection.ID == 7)
            //     Debug.LogWarning($"[NODE-7] Car {car.CarID}: DEADLOCK BREAKER - forcing entry");
            // else
            //     Debug.LogWarning($"[DEADLOCK-BREAK] Car {car.CarID} at Node {intersection.ID}: Breaking deadlock");
            
            return true; // This car is allowed to force entry - SKIP all other checks
        }
        
        // Check 3: Path conflict - does intended turn overlap with active crossings?
        foreach (var activeKvp in state.activeCrossings)
        {
            TurnCurve activeTurn = activeKvp.Value;
            
            if (turnConflicts.ContainsKey(intendedTurn) && 
                turnConflicts[intendedTurn].Contains(activeTurn))
            {
                // if (intersection.ID == 7)
                //     Debug.Log($"[NODE-7] Car {car.CarID}: BLOCKED by path conflict with Car {activeKvp.Key.CarID}");
                return false; // Path overlaps with active vehicle
            }
        }
        
        // Check 4: Priority to right (ONLY if no traffic lights)
        // If there are traffic lights, they handle priority phases
        if (!stopLights.ContainsKey(intersection))
        {
            if (HasPriorityVehicleOnRight(car, fromLane, intersection, out int blockingCarID))
            {
                // if (intersection.ID == 7)
                //     Debug.Log($"[NODE-7] Car {car.CarID}: BLOCKED by priority-to-right (Car {blockingCarID})");
                return false; // Vehicle on right has priority
            }
        }
        
        return true;
    }
    
    /// <summary>
    /// Get the current light state for a specific lane at an intersection.
    /// Returns Green if no light exists.
    /// </summary>
    public StopLightController.LightState GetLightState(Node intersection, Lane lane)
    {
        if (stopLights.ContainsKey(intersection))
        {
            return stopLights[intersection].GetLightState(lane);
        }
        return StopLightController.LightState.Green;
    }
    
    /// <summary>
    /// Check if a node has a traffic light controller.
    /// </summary>
    public bool HasTrafficLight(Node intersection)
    {
        return stopLights.ContainsKey(intersection);
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
        
        // Clear deadlock breaker if this car was it
        if (carBreakingDeadlock[intersection] == car)
        {
            carBreakingDeadlock[intersection] = null;
            // if (intersection.ID == 7)
            //     Debug.Log($"[NODE-7] Deadlock cleared - Car {car.CarID} entered");
        }
        
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
    /// Check if there are any cars waiting at the given intersection.
    /// Used for pre-approach queue detection.
    /// </summary>
    public bool HasWaitingCars(Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return false;
        }
        
        var state = intersectionStates[intersection];
        
        // Check if any lane has waiting cars
        foreach (var queueKvp in state.waitingQueues)
        {
            if (queueKvp.Value.Count > 0)
            {
                return true;
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// Check if there are multiple cars approaching the same intersection (within detection radius).
    /// Returns true if 2+ cars are converging, indicating potential congestion.
    /// </summary>
    public bool HasApproachingCars(Node intersection, CarController excludeCar, float detectionRadius)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return false;
        }
        
        Vector3 intersectionPos = intersection.Position;
        int approachingCount = 0;
        
        // Find all cars in the simulation
        CarController[] allCars = UnityEngine.Object.FindObjectsOfType<CarController>();
        
        foreach (var car in allCars)
        {
            // Skip the car we're checking for
            if (car == excludeCar)
                continue;
            
            // Check if car is within detection radius of intersection
            float distance = Vector3.Distance(car.transform.position, intersectionPos);
            
            if (distance <= detectionRadius)
            {
                // Check if this car is actually heading towards this intersection
                // (either in Approaching state targeting this node, or on a lane that ends at this node)
                bool isHeadingToIntersection = false;
                
                if (car.CurrentCarState == CarController.CarState.Approaching && 
                    car.CurrentLane != null && 
                    car.CurrentLane.EndNode == intersection)
                {
                    isHeadingToIntersection = true;
                }
                else if (car.CurrentCarState == CarController.CarState.OnLane && 
                         car.CurrentPathType == CarController.PathType.Lane &&
                         car.CurrentLane != null && 
                         car.CurrentLane.EndNode == intersection)
                {
                    isHeadingToIntersection = true;
                }
                
                if (isHeadingToIntersection)
                {
                    approachingCount++;
                    
                    // If we find at least 1 other car approaching, return true
                    // (2+ cars total including the excludeCar)
                    if (approachingCount >= 1)
                        return true;
                }
            }
        }
        
        return false;
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
    /// Check if entire intersection is deadlocked and designate one car to break it.
    /// Only ONE car at a time is allowed to force entry.
    /// </summary>
    private void CheckAndBreakDeadlock(Node intersection, IntersectionState state, CarController requestingCar)
    {
        // Don't reassign if we already have a deadlock breaker
        if (carBreakingDeadlock[intersection] != null)
        {
            return; // Let the current breaker finish
        }
        
        // If someone is actively crossing, no deadlock
        if (state.activeCrossings.Count > 0)
        {
            return;
        }
        
        // Collect all first-in-queue cars that are waiting
        List<CarController> waitingCars = new List<CarController>();
        float earliestWaitTime = float.MaxValue;
        CarController longestWaitingCar = null;
        
        foreach (var queueKvp in state.waitingQueues)
        {
            if (queueKvp.Value.Count > 0)
            {
                CarController firstCar = queueKvp.Value.Peek();
                
                // Only consider cars in WaitingToEnter state (not just approaching)
                if (firstCar.CurrentCarState == CarController.CarState.WaitingToEnter)
                {
                    waitingCars.Add(firstCar);
                    
                    float waitStartTime = state.GetWaitStartTime(firstCar);
                    if (waitStartTime < earliestWaitTime)
                    {
                        earliestWaitTime = waitStartTime;
                        longestWaitingCar = firstCar;
                    }
                }
            }
        }
        
        // Need at least 2 cars waiting to be a deadlock
        if (waitingCars.Count < 2)
        {
            return; // Not a deadlock, just normal waiting
        }
        
        // Check if ALL waiting cars have been waiting > timeout
        float currentTime = Time.time;
        bool allStuck = true;
        
        foreach (var car in waitingCars)
        {
            float waitTime = currentTime - state.GetWaitStartTime(car);
            if (waitTime < deadlockTimeout)
            {
                allStuck = false;
                break;
            }
        }
        
        // DEADLOCK DETECTED: All cars stuck for > 3 seconds
        if (allStuck && longestWaitingCar != null)
        {
            carBreakingDeadlock[intersection] = longestWaitingCar;
            
            // Deadlock logging disabled
            // if (intersection.ID == 7)
            // {
            //     float longestWait = currentTime - earliestWaitTime;
            //     Debug.LogWarning($"[NODE-7] DEADLOCK DETECTED: {waitingCars.Count} cars stuck. Designating Car {longestWaitingCar.CarID} (waiting {longestWait:F1}s) to break it");
            //     LogIntersectionState(intersection, state);
            // }
            // else
            // {
            //     Debug.LogWarning($"[DEADLOCK] Node {intersection.ID}: {waitingCars.Count} cars stuck, Car {longestWaitingCar.CarID} will break it");
            // }
        }
    }
    
    // /// <summary>
    // /// Log detailed intersection state for debugging deadlocks (Node 7 only).
    // /// </summary>
    // private void LogIntersectionState(Node intersection, IntersectionState state)
    // {
    //     if (intersection.ID != 7) return; // Only log Node 7
    //     
    //     Debug.Log($"[NODE-7] === STATE: {state.incomingLanes.Count} incoming lanes ===");
    //     
    //     // Log active crossings
    //     Debug.Log($"[NODE-7] Active crossings: {state.activeCrossings.Count}");
    //     foreach (var crossing in state.activeCrossings)
    //     {
    //         Debug.Log($"[NODE-7]   - Car {crossing.Key.CarID} (state: {crossing.Key.CurrentCarState}) from Lane {crossing.Value.FromLane.ID} to Lane {crossing.Value.ToLane.ID}");
    //     }
    //     
    //     // Log waiting queues
    //     int totalWaiting = 0;
    //     foreach (var queueKvp in state.waitingQueues)
    //     {
    //         if (queueKvp.Value.Count > 0)
    //         {
    //             totalWaiting += queueKvp.Value.Count;
    //             var queueList = new System.Collections.Generic.List<CarController>(queueKvp.Value);
    //             string carList = string.Join(", ", queueList.ConvertAll(c => $"Car{c.CarID}({c.CurrentCarState})"));
    //             Debug.Log($"[NODE-7] Waiting on Lane {queueKvp.Key.ID}: {queueKvp.Value.Count} cars [{carList}]");
    //         }
    //     }
    //     
    //     if (totalWaiting == 0)
    //     {
    //         Debug.Log($"[NODE-7] No cars waiting");
    //     }
    // }
    
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
