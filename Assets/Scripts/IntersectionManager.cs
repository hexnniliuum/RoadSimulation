using System.Collections.Generic;
using UnityEngine;
using System.Linq;

/// <summary>
/// Arbitrates intersection right-of-way: turn conflicts, queue order, priority-to-right, and deadlock breaking.
/// </summary>
public class IntersectionManager
{
    private Dictionary<Node, IntersectionState> intersectionStates;

    private Dictionary<Node, StopLightController> stopLights;

    // Precomputed: turn -> turns it conflicts with
    private Dictionary<TurnCurve, HashSet<TurnCurve>> turnConflicts;

    private float deadlockTimeout = 3f;

    // Car currently allowed to force entry through a deadlocked intersection
    private Dictionary<Node, CarController> carBreakingDeadlock;

    // Timestamp the breaker was designated; used to expire it if it never
    // manages to enter (e.g. its exit is blocked)
    private Dictionary<Node, float> breakerAssignedTime;

    // Cars committed to running a yellow, per node. Arbitrated so two cars
    // with conflicting turns can never both commit to the same phase change.
    private Dictionary<Node, Dictionary<CarController, TurnCurve>> committedRuns;

    // Latched roundabout entry grants: a car granted while still able to stop keeps
    // the grant through the final meters where it physically can't stop anymore.
    // Other cars' gate checks see these as reserved gaps, so a fast approach can't
    // slip past the gate between its last stoppable evaluation and the line.
    private Dictionary<Node, Dictionary<CarController, RoundaboutTurnCurve>> committedEntries;

    // Turns granted this frame, checked so two cars can't be granted conflicting entries simultaneously
    private Dictionary<Node, HashSet<CarController>> carsGrantedThisFrame;
    private Dictionary<Node, HashSet<TurnCurve>> turnsGrantedThisFrame;

    // Cached from config for stuck-car detection
    private float detectionDistance = 25f;

    // Cached from config for roundabout merge-time projection
    private float roundaboutSpeedLimit = 8f;

    public void Initialize(RoadMap roadMap, RoadGenerationConfig config)
    {
        intersectionStates = new Dictionary<Node, IntersectionState>();
        stopLights = new Dictionary<Node, StopLightController>();
        turnConflicts = new Dictionary<TurnCurve, HashSet<TurnCurve>>();
        carBreakingDeadlock = new Dictionary<Node, CarController>();
        breakerAssignedTime = new Dictionary<Node, float>();
        committedRuns = new Dictionary<Node, Dictionary<CarController, TurnCurve>>();
        committedEntries = new Dictionary<Node, Dictionary<CarController, RoundaboutTurnCurve>>();
        carsGrantedThisFrame = new Dictionary<Node, HashSet<CarController>>();
        turnsGrantedThisFrame = new Dictionary<Node, HashSet<TurnCurve>>();

        deadlockTimeout = config.DeadlockTimeout;
        detectionDistance = config.IntersectionDetectionDistance;
        roundaboutSpeedLimit = config.RoundaboutSpeedLimit;

        foreach (var node in roadMap.Nodes)
        {
            if (node.ConnectedRoads.Count >= 2)
            {
                IntersectionState state = new IntersectionState(node);
                intersectionStates[node] = state;
                carBreakingDeadlock[node] = null;
                breakerAssignedTime[node] = 0f;
                committedRuns[node] = new Dictionary<CarController, TurnCurve>();
                committedEntries[node] = new Dictionary<CarController, RoundaboutTurnCurve>();
                carsGrantedThisFrame[node] = new HashSet<CarController>();
                turnsGrantedThisFrame[node] = new HashSet<TurnCurve>();
            }
        }

        // Roundabouts are self-regulating; they never get a signal.
        var candidateNodes = roadMap.Nodes
            .Where(n => !n.IsTrivialIntersection() && n.ConnectedRoads.Count >= 2 && !n.IsRoundabout)
            .OrderByDescending(n => n.ConnectedRoads.Count)
            .ThenBy(n => n.ID) // deterministic tie-break
            .ToList();

        float percentage = Mathf.Clamp(config.TrafficLightPercentage, 0f, 100f);
        int targetCount = Mathf.RoundToInt(candidateNodes.Count * (percentage / 100f));

        for (int i = 0; i < targetCount; i++)
        {
            Node node = candidateNodes[i];
            stopLights[node] = new StopLightController(node);
        }

        BuildTurnConflictMap();
        
        Debug.Log($"[IntersectionManager] Initialized {intersectionStates.Count} intersections. Traffic Lights: {stopLights.Count}/{candidateNodes.Count} ({percentage:F0}%)");
    }
    
    /// <summary>
    /// Update traffic lights and reset per-frame tracking.
    /// </summary>
    public void Update(float deltaTime)
    {
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
    /// Returns whether the car may proceed. Does not itself grant entry —
    /// call GrantEntry/RegisterEntry separately once it does.
    /// </summary>
    public bool CanEnter(CarController car, Lane fromLane, TurnCurve intendedTurn, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            Debug.LogWarning($"[DEADLOCK] IntersectionManager: No state for intersection at Node {intersection.ID}");
            return true; // Not a managed intersection
        }
        
        var state = intersectionStates[intersection];
        
        // Check 0: traffic lights override everything else. A car committed to
        // running a yellow it can't stop for skips this gate (commitments are
        // arbitrated in TryCommitRunLight); every check below still applies to it.
        if (stopLights.ContainsKey(intersection))
        {
            var lightState = stopLights[intersection].GetLightState(fromLane);
            if (lightState != StopLightController.LightState.Green && !car.IsCommittedToRunLight)
            {
                return false; // Stop on Red/Yellow
            }
        }

        // Check 1: must be first in queue for this lane
        if (state.waitingQueues.ContainsKey(fromLane))
        {
            var queue = state.waitingQueues[fromLane];
            while (queue.Count > 0 && queue.Peek() == null)
            {
                queue.Dequeue(); // drop cars destroyed while queued
            }
            if (queue.Count > 0 && queue.Peek() != car)
            {
                return false; // Not your turn yet
            }
        }

        // Check 2: deadlock detection, intersection-level (triggers only once every queued car is stuck)
        CheckAndBreakDeadlock(intersection, state, car);

        // Check 2.5: designated deadlock breaker skips every remaining check
        if (carBreakingDeadlock.ContainsKey(intersection) && carBreakingDeadlock[intersection] == car)
        {
            return true;
        }

        // Roundabouts replace the conflict-map and priority checks below with ring
        // occupancy: yield only to circulating traffic near the entry point, so
        // several cars can use the ring at once.
        if (intersection.IsRoundabout)
        {
            return RoundaboutCanEnter(car, fromLane, intendedTurn as RoundaboutTurnCurve, intersection, state);
        }

        // Check 3: does the intended turn overlap an active crossing?
        foreach (var activeKvp in state.activeCrossings)
        {
            if (activeKvp.Key == null)
                continue; // destroyed car, cleaned up in ResetCarState

            if (TurnsConflict(intendedTurn, activeKvp.Value))
            {
                return false; // Path overlaps with active vehicle
            }
        }

        // Check 3.5: same, but against turns granted this same frame (stops two cars entering simultaneously with conflicting paths)
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

        // Check 4: priority-to-right. Lit intersections handle priority via phases instead.
        if (!stopLights.ContainsKey(intersection))
        {
            // A car already too close to stop can't be forced to brake instantly;
            // it proceeds and the yielding car simply waits one more cycle (Check 3 still applies).
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
    /// Green if the intersection has no light.
    /// </summary>
    public StopLightController.LightState GetLightState(Node intersection, Lane lane)
    {
        if (stopLights.ContainsKey(intersection))
        {
            return stopLights[intersection].GetLightState(lane);
        }
        return StopLightController.LightState.Green;
    }

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

    public bool IsDeadlockBreaker(CarController car, Node intersection)
    {
        return carBreakingDeadlock.ContainsKey(intersection) && carBreakingDeadlock[intersection] == car;
    }

    /// <summary>
    /// Clears all per-car state. Must be called on despawn, or a destroyed
    /// car's reference lingers in a queue and permanently blocks that lane.
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
        foreach (var entries in committedEntries.Values)
        {
            entries.Clear();
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
    /// Records a granted turn for same-frame conflict detection (Check 3.5).
    /// Call after CanEnter returns true and the car will actually enter.
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
    /// Marks the car as crossing; also fulfills any run-light commitment and clears it as the deadlock breaker.
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

        // covers callers that call RegisterEntry without a prior GrantEntry
        if (carsGrantedThisFrame.ContainsKey(intersection))
        {
            carsGrantedThisFrame[intersection].Add(car);
            turnsGrantedThisFrame[intersection].Add(turn);
        }

        CancelRunLight(car, intersection);

        // The reservation served its purpose; the car now blocks via activeCrossings.
        if (committedEntries.ContainsKey(intersection))
        {
            committedEntries[intersection].Remove(car);
        }

        if (carBreakingDeadlock[intersection] == car)
        {
            carBreakingDeadlock[intersection] = null;
        }
    }

    public void RegisterExit(CarController car, Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            Debug.LogWarning($"[IntersectionManager] Cannot register exit - no state for Node {intersection.ID}");
            return;
        }

        var state = intersectionStates[intersection];
        state.activeCrossings.Remove(car);
    }

    public bool HasWaitingCars(Node intersection)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return false;
        }

        var state = intersectionStates[intersection];

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
    /// Same "actually stopped" check as HasWaitingCars, but scoped to a single lane.
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
    /// True if at least 2 cars total (excludeCar plus 1+ others) are approaching within radius.
    /// </summary>
    public bool HasApproachingCars(Node intersection, CarController excludeCar, float detectionRadius)
    {
        if (!intersectionStates.ContainsKey(intersection))
        {
            return false;
        }

        Vector3 intersectionPos = intersection.Position;
        int approachingCount = 0;

        CarController[] allCars = UnityEngine.Object.FindObjectsOfType<CarController>();

        foreach (var car in allCars)
        {
            if (car == excludeCar)
                continue;

            float distance = Vector3.Distance(car.transform.position, intersectionPos);

            if (distance <= detectionRadius)
            {
                // Heading here means either still approaching this node, or already on a lane that ends here
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

                    // approachingCount excludes excludeCar, so >=1 here means 2+ total
                    if (approachingCount >= 1)
                        return true;
                }
            }
        }

        return false;
    }

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

        // Dedup only; the wait timer for deadlock detection starts later, once actually stuck at the line
        if (!state.waitingQueues[lane].Contains(car))
        {
            state.waitingQueues[lane].Enqueue(car);
        }
    }

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
            }
        }
    }

    /// <summary>
    /// Designates at most one car at a time to force entry once the whole intersection is deadlocked.
    /// </summary>
    private void CheckAndBreakDeadlock(Node intersection, IntersectionState state, CarController requestingCar)
    {
        // Never force entry at a signalized intersection: a breaker picked during a long
        // red would bypass conflict checks the moment its phase turns green.
        if (stopLights.ContainsKey(intersection))
        {
            return;
        }

        // Keep the current breaker unless it timed out without entering (e.g. blocked exit) - then pick a new one
        CarController currentBreaker = carBreakingDeadlock[intersection];
        if (currentBreaker != null)
        {
            if (Time.time - breakerAssignedTime[intersection] <= deadlockTimeout)
            {
                return; // Let the current breaker finish
            }
        }
        carBreakingDeadlock[intersection] = null;

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

        // First-in-queue cars stuck at, or crawling just short of, the stop line.
        // A blocked head can stall a few units early while still "Approaching",
        // so CarState alone doesn't catch it (see IsStuckAtStopLine).
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
                state.ClearWaitStartTime(firstCar);
                continue;
            }

            // Wait timer starts once actually stuck, not when it first enters the detection ring
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

        // Two stuck approaches after the timeout is a mutual deadlock; a single stuck
        // approach (e.g. a jam looping through other intersections) waits twice as long.
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
    /// True if formally waiting, or stopped/crawling just short of the stop line.
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

    /// <summary>
    /// True if a right-side queue head has priority: it must be at its stop
    /// line or close and approaching (not only fully stopped), and its path
    /// must actually cross ours.
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

            Vector3 otherDir = (incomingLane.EndPosition - incomingLane.StartPosition).normalized;

            // cross.y negative = other lane is to our right (clockwise)
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
            // A higher-priority turn (straight > right > left) always wins, even against
            // a car on our right; equal-or-higher turns still yield via priority-to-right below.
            if (theirTurn != null && requestingCar.IntendedTurn != null &&
                TurnPriority(theirTurn.Type) < TurnPriority(requestingCar.IntendedTurn.Type))
            {
                continue;
            }
            // Only yield if their path actually crosses ours; if either turn is somehow
            // still unknown, yield to be safe (both should be pre-selected by Approaching).
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

    // Clearance margins around a ring merge point, sized beyond a car length: the
    // projection assumes constant speed, so margins must absorb a car slowing down
    // or speeding up over the projection horizon before the actual merge.
    private const float RoundaboutMergeAheadMargin = 10f;
    private const float RoundaboutMergeBehindMin = 8f;
    private const float RoundaboutMergeBehindHeadway = 1.2f;

    /// <summary>
    /// Time for an entering car to cover the given distance, accounting for acceleration
    /// toward ring speed. Needed because a flat distance/speed projection underestimates
    /// arrival time for a slow, queue-restarting car and would grant it entry too early.
    /// </summary>
    private float ProjectedMergeTime(CarController car, float distance)
    {
        float top = Mathf.Max(roundaboutSpeedLimit, car.CurrentSpeed);
        float accel = Mathf.Max(car.MaxAcceleration, 0.1f);
        float timeToTop = (top - car.CurrentSpeed) / accel;
        float distWhileAccelerating = car.CurrentSpeed * timeToTop + 0.5f * accel * timeToTop * timeToTop;

        if (distWhileAccelerating >= distance)
        {
            return (Mathf.Sqrt(car.CurrentSpeed * car.CurrentSpeed + 2f * accel * distance)
                    - car.CurrentSpeed) / accel;
        }

        return timeToTop + (distance - distWhileAccelerating) / top;
    }

    /// <summary>
    /// Roundabout entry with commitment latching: a grant issued while the car can still
    /// stop is remembered and carries it through once inside stopping distance, even if
    /// clearance flips - other gate checks already saw the reservation. Without the latch,
    /// an approach's final stretch (inside stopping distance at merge speed) would be ungated.
    /// </summary>
    private bool RoundaboutCanEnter(CarController car, Lane fromLane, RoundaboutTurnCurve myTurn,
                                    Node intersection, IntersectionState state)
    {
        if (myTurn == null)
        {
            return true; // not a roundabout path (shouldn't happen); other checks still guard
        }

        var committed = committedEntries[intersection];

        // A committed car that ended up (nearly) stopped anyway lost its slot;
        // holding the reservation would block the ring for nothing.
        if (committed.ContainsKey(car) && car.CurrentSpeed < 1f)
        {
            committed.Remove(car);
        }

        float distanceToLine = fromLane.Length - car.DistanceAlongPath;
        float minStoppingDistance = (car.CurrentSpeed * car.CurrentSpeed) / (2f * car.MaxBraking * 0.85f);

        if (RoundaboutEntryClear(car, fromLane, myTurn, state, committed))
        {
            // Latch the grant just before the car becomes physically committed, so
            // the reservation exists for the whole unstoppable window.
            if (car.CurrentSpeed >= 3f && distanceToLine < minStoppingDistance * 1.5f + 1f)
            {
                committed[car] = myTurn;
            }

            return true;
        }

        // Clearance lost after latching: honor the reservation only while the car genuinely
        // can no longer stop. Circulating traffic braking after the grant (e.g. yielding to
        // another merger) can invalidate the reserved slot; revoke while stopping is still possible.
        if (committed.ContainsKey(car))
        {
            if (distanceToLine > minStoppingDistance + 0.5f)
            {
                committed.Remove(car);
                return false;
            }
            return true;
        }

        return false;
    }

    /// <summary>
    /// Roundabout right-of-way: circulating traffic wins. Entry is allowed only if every
    /// car on the ring (or committed to enter it) will be clear past the merge point, or far
    /// enough behind to follow smoothly, once this car actually reaches the merge - which is
    /// what lets multiple cars share the ring instead of only ever admitting one at a time.
    /// </summary>
    private bool RoundaboutEntryClear(CarController car, Lane fromLane, RoundaboutTurnCurve myTurn,
                                      IntersectionState state,
                                      Dictionary<CarController, RoundaboutTurnCurve> committed)
    {
        float distToMerge = (fromLane.Length - car.DistanceAlongPath) + myTurn.EntryConnectorLength;
        float myTimeToMerge = ProjectedMergeTime(car, distToMerge);

        foreach (var crossing in state.activeCrossings)
        {
            if (crossing.Key == null || crossing.Key == car)
                continue;
            if (!(crossing.Value is RoundaboutTurnCurve theirTurn) || !theirTurn.SharesRingWith(myTurn))
                continue;

            // A registered car still covering the last stretch of its lane carries a
            // lane distance, which read as curve distance would project it near the
            // exit and hide it from the gate exactly while it merges.
            CarController other = crossing.Key;
            float progress = other.CurrentPathType == CarController.PathType.TurnCurve
                ? theirTurn.SignedRingProgress(other.DistanceAlongPath)
                : -((theirTurn.FromLane.Length - other.DistanceAlongPath) + theirTurn.EntryConnectorLength);

            if (BlocksMyMerge(theirTurn, progress, other.CurrentSpeed, myTurn, myTimeToMerge))
            {
                return false;
            }
        }

        foreach (var kvp in committed)
        {
            CarController other = kvp.Key;
            if (other == null || other == car)
                continue;
            if (other.CurrentPathType != CarController.PathType.Lane || other.CurrentLane == null)
                continue; // already entered - covered by activeCrossings
            if (!kvp.Value.SharesRingWith(myTurn))
                continue;

            // Still on its approach lane: negative ring progress by the distance
            // it has left to its own merge point.
            float progress = -((kvp.Value.FromLane.Length - other.DistanceAlongPath)
                               + kvp.Value.EntryConnectorLength);
            if (BlocksMyMerge(kvp.Value, progress, other.CurrentSpeed, myTurn, myTimeToMerge))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Whether a car on (or committed to) the ring, at the given signed ring
    /// progress and speed, will conflict with a merge at myTurn's entry point
    /// myTimeToMerge seconds from now.
    /// </summary>
    private bool BlocksMyMerge(RoundaboutTurnCurve theirTurn, float progress, float theirSpeed,
                               RoundaboutTurnCurve myTurn, float myTimeToMerge)
    {
        if (progress > theirTurn.RingArcLength)
            return false; // already on its exit connector, diverging from the ring

        float ring = myTurn.RingCircumference;
        float futureProgress = Mathf.Min(progress + theirSpeed * myTimeToMerge, theirTurn.RingArcLength);
        if (futureProgress < 0f)
            return false; // still short of its own merge point at my merge time - not on the ring yet
        float remainingAfter = theirTurn.RingArcLength - futureProgress;
        float theirFutureAbs = RoundaboutTurnCurve.RingMod(
            theirTurn.EntryRingDistance + futureProgress, ring);

        float aheadOfMyEntry = RoundaboutTurnCurve.RingMod(theirFutureAbs - myTurn.EntryRingDistance, ring);
        if (aheadOfMyEntry < RoundaboutMergeAheadMargin)
        {
            return true; // would occupy my merge point when I arrive
        }

        float behindMyEntry = ring - aheadOfMyEntry;
        bool stillReachesMyEntry = behindMyEntry <= remainingAfter + 0.5f;
        float behindMargin = Mathf.Max(RoundaboutMergeBehindMin, theirSpeed * RoundaboutMergeBehindHeadway);
        return stillReachesMyEntry && behindMyEntry < behindMargin;
    }

    /// <summary>
    /// Right-of-way rank of a turn type: straight (2) beats right (1) beats left (0).
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
        List<TurnCurve> allTurns = new List<TurnCurve>();

        foreach (var state in intersectionStates.Values)
        {
            // Roundabout paths all share the ring, so spatial sampling would mark
            // nearly every pair as conflicting and serialize the whole roundabout;
            // their interaction is handled by ring occupancy instead.
            if (state.Node.IsRoundabout)
                continue;

            foreach (var lane in state.incomingLanes)
            {
                allTurns.AddRange(lane.AvailableTurns);
            }
        }

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

                if (turnA.FromLane.EndNode != turnB.FromLane.EndNode)
                {
                    continue;
                }

                // Turns from the same lane are already serialized by queue order and
                // car-following; marking them as conflicts would force one-car-at-a-time
                // flow through a green light needlessly.
                if (turnA.FromLane == turnB.FromLane)
                {
                    continue;
                }

                // Turns conflict only if their sampled paths actually cross - no turn-type
                // exemption. Opposite-approach lefts run together only when their arcs stay clear.
                if (TurnCurvesConflict(turnA, turnB))
                {
                    turnConflicts[turnA].Add(turnB);

                    if (!turnConflicts.ContainsKey(turnB))
                    {
                        turnConflicts[turnB] = new HashSet<TurnCurve>();
                    }
                    turnConflicts[turnB].Add(turnA);
                }
            }
        }
    }

    /// <summary>
    /// Approximates each curve as line segments (see SampleCurve) and tests those for intersection.
    /// </summary>
    private bool TurnCurvesConflict(TurnCurve a, TurnCurve b)
    {
        List<Vector3> pointsA = SampleCurve(a, 10);
        List<Vector3> pointsB = SampleCurve(b, 10);

        for (int i = 0; i < pointsA.Count - 1; i++)
        {
            for (int j = 0; j < pointsB.Count - 1; j++)
            {
                if (LineSegmentsIntersect2D(pointsA[i], pointsA[i+1], pointsB[j], pointsB[j+1]))
                {
                    return true;
                }
            }
        }

        return false;
    }

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
    /// Segment intersection in the XZ ground plane; height (Y) is ignored.
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

public class IntersectionState
{
    public Node Node;

    public Dictionary<CarController, TurnCurve> activeCrossings;

    public Dictionary<Lane, Queue<CarController>> waitingQueues;

    public List<Lane> incomingLanes;

    // Wait start times, used for deadlock detection
    private Dictionary<CarController, float> waitStartTimes;

    public IntersectionState(Node node)
    {
        Node = node;
        activeCrossings = new Dictionary<CarController, TurnCurve>();
        waitingQueues = new Dictionary<Lane, Queue<CarController>>();
        incomingLanes = new List<Lane>();
        waitStartTimes = new Dictionary<CarController, float>();

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
