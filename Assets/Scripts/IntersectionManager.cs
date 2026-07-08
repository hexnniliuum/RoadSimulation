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

    // When the current deadlock breaker was designated (for expiry if it
    // never manages to enter, e.g. because its exit is blocked)
    private Dictionary<Node, float> breakerAssignedTime;

    // Cars committed to running a yellow light, per node. Arbitrated so two
    // cars with conflicting turns can never both commit to the same phase change.
    private Dictionary<Node, Dictionary<CarController, TurnCurve>> committedRuns;

    // Track cars granted entry this frame to prevent simultaneous conflicting entries
    private Dictionary<Node, HashSet<CarController>> carsGrantedThisFrame;
    private Dictionary<Node, HashSet<TurnCurve>> turnsGrantedThisFrame;

    // Cached from config for stuck-car detection
    private float detectionDistance = 25f;
    
    /// <summary>
    /// Initialize intersection management system with road network.
    /// </summary>
    public void Initialize(RoadMap roadMap, RoadGenerationConfig config)
    {
        intersectionStates = new Dictionary<Node, IntersectionState>();
        stopLights = new Dictionary<Node, StopLightController>();
        turnConflicts = new Dictionary<TurnCurve, HashSet<TurnCurve>>();
        carBreakingDeadlock = new Dictionary<Node, CarController>();
        breakerAssignedTime = new Dictionary<Node, float>();
        committedRuns = new Dictionary<Node, Dictionary<CarController, TurnCurve>>();
        carsGrantedThisFrame = new Dictionary<Node, HashSet<CarController>>();
        turnsGrantedThisFrame = new Dictionary<Node, HashSet<TurnCurve>>();

        deadlockTimeout = config.DeadlockTimeout;
        detectionDistance = config.IntersectionDetectionDistance;

        // Build intersection states for each node with 2+ roads
        foreach (var node in roadMap.Nodes)
        {
            if (node.ConnectedRoads.Count >= 2)
            {
                IntersectionState state = new IntersectionState(node);
                intersectionStates[node] = state;
                carBreakingDeadlock[node] = null; // Initialize with no breaker
                breakerAssignedTime[node] = 0f;
                committedRuns[node] = new Dictionary<CarController, TurnCurve>();
                carsGrantedThisFrame[node] = new HashSet<CarController>();
                turnsGrantedThisFrame[node] = new HashSet<TurnCurve>();
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
    /// Update traffic lights and reset per-frame tracking.
    /// </summary>
    public void Update(float deltaTime)
    {
        // Reset per-frame entry tracking
        foreach (var node in carsGrantedThisFrame.Keys)
        {
            carsGrantedThisFrame[node].Clear();
            turnsGrantedThisFrame[node].Clear();
        }
        
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
        // A car that committed to running a yellow it cannot stop for skips
        // the light gate (commitments are arbitrated in TryCommitRunLight);
        // everything below - queue order, conflicts - still applies to it.
        if (stopLights.ContainsKey(intersection))
        {
            var lightState = stopLights[intersection].GetLightState(fromLane);
            if (lightState != StopLightController.LightState.Green && !car.IsCommittedToRunLight)
            {
                return false; // Stop on Red/Yellow
            }
            // Green light: Proceed to other checks (still need to check for blocked intersection)
        }

        // Check 1: Must be first in queue for this lane
        if (state.waitingQueues.ContainsKey(fromLane))
        {
            var queue = state.waitingQueues[fromLane];
            while (queue.Count > 0 && queue.Peek() == null)
            {
                queue.Dequeue(); // drop cars destroyed while queued
            }
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
            if (activeKvp.Key == null)
                continue; // destroyed car, cleaned up in ResetCarState

            if (TurnsConflict(intendedTurn, activeKvp.Value))
            {
                return false; // Path overlaps with active vehicle
            }
        }
        
        // Check 3.5: Path conflict with turns granted this SAME FRAME
        // This prevents two cars from entering simultaneously with conflicting paths
        if (turnsGrantedThisFrame.ContainsKey(intersection))
        {
            foreach (var grantedTurn in turnsGrantedThisFrame[intersection])
            {
                if (TurnsConflict(intendedTurn, grantedTurn))
                {
                    return false; // Another car was granted a conflicting turn this frame
                }
            }
        }
        
        // Check 4: Priority to right (ONLY if no traffic lights)
        // If there are traffic lights, they handle priority phases
        if (!stopLights.ContainsKey(intersection))
        {
            // Physics exemption: a car that can no longer stop at the line
            // (the right-side car arrived at its stop line at the last moment)
            // must not be forced into an instant stop - it proceeds, and the
            // stationary right-side car waits one more turn via Check 3.
            float distanceToLine = fromLane.Length - car.DistanceAlongPath;
            float minStoppingDistance = (car.CurrentSpeed * car.CurrentSpeed) / (2f * car.MaxBraking * 0.85f);
            bool physicallyCommitted = distanceToLine < minStoppingDistance;

            if (!physicallyCommitted &&
                HasPriorityVehicleOnRight(car, fromLane, intersection, out int blockingCarID))
            {
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
    /// A car that cannot stop for a yellow light asks to run it. Granted only
    /// if no active crossing and no other committed car holds a conflicting
    /// turn, so two conflicting approaches can never both run the same yellow.
    /// </summary>
    public bool TryCommitRunLight(CarController car, Node intersection, TurnCurve turn)
    {
        if (!intersectionStates.ContainsKey(intersection) || !committedRuns.ContainsKey(intersection))
        {
            return true; // Not a managed intersection
        }

        var state = intersectionStates[intersection];

        foreach (var crossing in state.activeCrossings)
        {
            if (crossing.Key == null || crossing.Key == car)
                continue;
            if (TurnsConflict(turn, crossing.Value))
            {
                return false;
            }
        }

        var runs = committedRuns[intersection];
        foreach (var kvp in runs)
        {
            if (kvp.Key == null || kvp.Key == car)
                continue;
            if (TurnsConflict(turn, kvp.Value))
            {
                return false;
            }
        }

        runs[car] = turn;
        return true;
    }

    /// <summary>
    /// Release a run-light commitment (car stopped, entered, or left the approach).
    /// </summary>
    public void CancelRunLight(CarController car, Node intersection)
    {
        if (committedRuns.ContainsKey(intersection))
        {
            committedRuns[intersection].Remove(car);
        }
    }

    /// <summary>
    /// Check whether this car is the designated deadlock breaker for the node.
    /// </summary>
    public bool IsDeadlockBreaker(CarController car, Node intersection)
    {
        return carBreakingDeadlock.ContainsKey(intersection) && carBreakingDeadlock[intersection] == car;
    }

    /// <summary>
    /// Clear all per-car state (queues, crossings, breakers, commitments).
    /// Must be called when cars are despawned, otherwise destroyed references
    /// left in the queues permanently block their lanes.
    /// </summary>
    public void ResetCarState()
    {
        foreach (var state in intersectionStates.Values)
        {
            state.activeCrossings.Clear();
            state.waitingQueues.Clear();
            state.ClearAllWaitTimes();
        }
        foreach (var node in new List<Node>(carBreakingDeadlock.Keys))
        {
            carBreakingDeadlock[node] = null;
        }
        foreach (var runs in committedRuns.Values)
        {
            runs.Clear();
        }
        foreach (var set in carsGrantedThisFrame.Values)
        {
            set.Clear();
        }
        foreach (var set in turnsGrantedThisFrame.Values)
        {
            set.Clear();
        }
    }

    /// <summary>
    /// Whether two turn curves overlap spatially (per the precomputed map).
    /// </summary>
    public bool TurnsMayConflict(TurnCurve a, TurnCurve b)
    {
        return TurnsConflict(a, b);
    }

    private bool TurnsConflict(TurnCurve a, TurnCurve b)
    {
        return turnConflicts.ContainsKey(a) && turnConflicts[a].Contains(b);
    }
    
    /// <summary>
    /// Grant entry permission to a car (call this when CanEnter returns true and car will enter).
    /// This tracks the granted turn for same-frame conflict detection.
    /// </summary>
    public void GrantEntry(CarController car, Node intersection, TurnCurve turn)
    {
        if (carsGrantedThisFrame.ContainsKey(intersection))
        {
            carsGrantedThisFrame[intersection].Add(car);
            turnsGrantedThisFrame[intersection].Add(turn);
        }
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

        // Also track in per-frame grants if not already there
        if (carsGrantedThisFrame.ContainsKey(intersection))
        {
            carsGrantedThisFrame[intersection].Add(car);
            turnsGrantedThisFrame[intersection].Add(turn);
        }

        // Any run-light commitment is fulfilled by actually entering
        CancelRunLight(car, intersection);

        // Clear deadlock breaker if this car was it
        if (carBreakingDeadlock[intersection] == car)
        {
            carBreakingDeadlock[intersection] = null;
        }
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
        
        // Check if any lane has waiting cars that are actually at the stop line
        foreach (var queueKvp in state.waitingQueues)
        {
            foreach (var car in queueKvp.Value)
            {
                // Only count cars actually waiting at stop line, not those still approaching
                if (car.CurrentCarState == CarController.CarState.WaitingToEnter)
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    /// <summary>
    /// Check if there are cars waiting specifically on the given lane at the intersection.
    /// More precise than HasWaitingCars for deciding when to brake.
    /// </summary>
    public bool HasWaitingCarsOnLane(Lane lane, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return false;
        }
        
        var state = intersectionStates[intersection];
        
        if (!state.waitingQueues.ContainsKey(lane))
        {
            return false;
        }
        
        var queue = state.waitingQueues[lane];
        
        // Check if there's a car ahead of us on this lane that's actually stopped
        foreach (var car in queue)
        {
            if (car.CurrentCarState == CarController.CarState.WaitingToEnter)
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
        
        // Only add if not already in queue. Wait timing for deadlock detection
        // is recorded later, when the car is actually stuck at the stop line.
        if (!state.waitingQueues[lane].Contains(car))
        {
            state.waitingQueues[lane].Enqueue(car);
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
            while (queue.Count > 0 && queue.Peek() == null)
            {
                queue.Dequeue(); // drop cars destroyed while queued
            }
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
        // Signalized intersections are cleared by the light cycle and must
        // never be forced: a breaker designated during a long red would later
        // bypass the conflict checks the moment its phase turns green.
        if (stopLights.ContainsKey(intersection))
        {
            return;
        }

        // Keep the current breaker unless it failed to enter within the
        // timeout (e.g. its exit lane is blocked) - then try another car
        CarController currentBreaker = carBreakingDeadlock[intersection];
        if (currentBreaker != null)
        {
            if (Time.time - breakerAssignedTime[intersection] <= deadlockTimeout)
            {
                return; // Let the current breaker finish
            }
        }
        carBreakingDeadlock[intersection] = null;

        // If someone is actively crossing, no deadlock
        bool anyActiveCrossing = false;
        foreach (var crossing in state.activeCrossings)
        {
            if (crossing.Key != null)
            {
                anyActiveCrossing = true;
                break;
            }
        }
        if (anyActiveCrossing)
        {
            return;
        }

        // Collect all first-in-queue cars stuck at (or crawling near) the stop
        // line. Cross-boundary following means a blocked head can stop a few
        // units short of the line while still Approaching, so state alone
        // isn't enough to detect it.
        List<CarController> waitingCars = new List<CarController>();
        float earliestWaitTime = float.MaxValue;
        CarController longestWaitingCar = null;
        float currentTime = Time.time;

        foreach (var queueKvp in state.waitingQueues)
        {
            var queue = queueKvp.Value;
            while (queue.Count > 0 && queue.Peek() == null)
            {
                queue.Dequeue(); // drop cars destroyed while queued
            }
            if (queue.Count == 0)
            {
                continue;
            }

            CarController firstCar = queue.Peek();
            if (!IsStuckAtStopLine(firstCar))
            {
                state.ClearWaitStartTime(firstCar); // moving again - reset timer
                continue;
            }

            // Wait timing starts when the car actually gets stuck, not when it
            // enters the detection ring 25 units out
            state.RecordWaitStartTime(firstCar, currentTime);

            waitingCars.Add(firstCar);
            float waitStartTime = state.GetWaitStartTime(firstCar);
            if (waitStartTime < earliestWaitTime)
            {
                earliestWaitTime = waitStartTime;
                longestWaitingCar = firstCar;
            }
        }

        if (waitingCars.Count == 0)
        {
            return;
        }

        // Two stuck approaches after the timeout is a classic mutual deadlock;
        // a single stuck approach (e.g. blocked by a jam that loops through
        // other intersections) gets forced through after twice the timeout.
        float requiredWait = waitingCars.Count >= 2 ? deadlockTimeout : deadlockTimeout * 2f;

        foreach (var car in waitingCars)
        {
            if (currentTime - state.GetWaitStartTime(car) < requiredWait)
            {
                return; // someone is still within the patience window
            }
        }

        if (longestWaitingCar != null)
        {
            carBreakingDeadlock[intersection] = longestWaitingCar;
            breakerAssignedTime[intersection] = currentTime;
        }
    }

    /// <summary>
    /// True if the car is effectively parked at the intersection entrance
    /// (either formally waiting, or stopped just short of the stop line).
    /// </summary>
    private bool IsStuckAtStopLine(CarController car)
    {
        if (car.CurrentCarState == CarController.CarState.WaitingToEnter)
        {
            return true;
        }
        if (car.CurrentCarState != CarController.CarState.Approaching || car.CurrentLane == null)
        {
            return false;
        }

        float distanceToStopLine = car.CurrentLane.Length - car.DistanceAlongPath;
        return car.CurrentSpeed < 0.5f && distanceToStopLine < detectionDistance;
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
    /// Yields to a right-side queue head that is either at its stop line
    /// (WaitingToEnter) OR close and still approaching - the real rule recognises
    /// an approaching car (blinker on) well before it stops, not the instant it
    /// reaches the line. Only yields when the two paths actually cross.
    /// </summary>
    private bool HasPriorityVehicleOnRight(CarController requestingCar, Lane fromLane, Node node, out int blockingCarID)
    {
        blockingCarID = -1;
        var state = intersectionStates[node];

        Vector3 carDirection = (fromLane.EndPosition - fromLane.StartPosition).normalized;
        float myDistanceToLine = fromLane.Length - requestingCar.DistanceAlongPath;

        // A right-side car counts as "approaching" once it is within this distance of
        // its own stop line (or is already closer to its line than we are to ours).
        float approachHorizon = detectionDistance * 0.6f;

        foreach (var incomingLane in state.incomingLanes)
        {
            if (incomingLane == fromLane) continue;

            // Get direction of other lane
            Vector3 otherDir = (incomingLane.EndPosition - incomingLane.StartPosition).normalized;

            // Cross product: NEGATIVE Y means other lane is to our right (clockwise)
            // Positive Y means left (counter-clockwise)
            float cross = Vector3.Cross(carDirection, otherDir).y;
            if (cross >= -0.1f) continue; // not on our right

            if (!state.waitingQueues.ContainsKey(incomingLane)) continue;
            var queue = state.waitingQueues[incomingLane];
            while (queue.Count > 0 && queue.Peek() == null)
            {
                queue.Dequeue(); // drop cars destroyed while queued
            }
            if (queue.Count == 0) continue;

            CarController carOnRight = queue.Peek();
            if (carOnRight == null) continue;

            // Recognise the right-side car if it is at its line, or approaching and
            // near it - not only fully stopped cars.
            bool atLine = carOnRight.CurrentCarState == CarController.CarState.WaitingToEnter;
            bool nearAndApproaching = false;
            if (carOnRight.CurrentCarState == CarController.CarState.Approaching)
            {
                float theirDistanceToLine = incomingLane.Length - carOnRight.DistanceAlongPath;
                nearAndApproaching = theirDistanceToLine <= approachHorizon ||
                                     theirDistanceToLine <= myDistanceToLine;
            }
            if (!atLine && !nearAndApproaching) continue;

            TurnCurve theirTurn = carOnRight.IntendedTurn;
            // Turn-type priority overrides priority-to-right: a higher-priority
            // movement (straight over right/left, right over left) does not yield to
            // a lower-priority car merely because it is on the right - that car must
            // yield to us by turn type instead. Equal or higher priority still
            // follows priority-to-right below.
            if (theirTurn != null && requestingCar.IntendedTurn != null &&
                TurnPriority(theirTurn.Type) < TurnPriority(requestingCar.IntendedTurn.Type))
            {
                continue;
            }
            // Only give way when their path actually crosses ours. Both turns are
            // pre-selected by the time a car is Approaching; if the turn is somehow
            // unknown, yield to stay safe.
            if (theirTurn != null && requestingCar.IntendedTurn != null &&
                !TurnsConflict(requestingCar.IntendedTurn, theirTurn))
            {
                continue; // paths don't cross - no need to yield
            }

            blockingCarID = carOnRight.CarID;
            return true; // Vehicle on right has priority
        }

        return false;
    }

    /// <summary>
    /// Right-of-way rank of a turn type: straight (2) beats right (1) beats
    /// left (0). Priority-to-right only applies between equal-or-higher ranks;
    /// a lower-rank car on the right must yield by turn type instead.
    /// </summary>
    private static int TurnPriority(TurnCurve.TurnType type)
    {
        if (type == TurnCurve.TurnType.Straight) return 2;
        if (type == TurnCurve.TurnType.Right) return 1;
        return 0; // Left
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

                // Turns leaving from the same lane share a start point but can
                // never be active side by side out of order: queue order and
                // car-following already serialize those cars. Marking them as
                // conflicts would force one-car-at-a-time flow through greens.
                if (turnA.FromLane == turnB.FromLane)
                {
                    continue;
                }

                // Two turns are only compatible (safe to run together) when their
                // sampled paths don't actually cross - so opposite-approach left
                // turns proceed simultaneously exactly when their arcs stay clear,
                // and are serialized when the arcs overlap. The honest geometric
                // check below is the single source of truth; there is no turn-type
                // exemption, because exempting a genuinely-crossing pair would let
                // two cars drive through the same point at once.

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

    public void ClearAllWaitTimes()
    {
        waitStartTimes.Clear();
    }
}
