# Stage 4: Intersection Logic - Implementation Plan

## OverviewO
Implement "priority to the right" intersection logic using a data-driven approach. The system will manage vehicle entry/exit from intersections, enforce right-of-way rules, and prevent collisions.

---

## Core Requirements

### 1. Stop Point Constraint
- **Requirement**: Vehicle stop point must NOT exceed the lane cutoff distance at intersection
- **Implementation**: Each lane already has `StopLineDistanceAtStart` calculated during generation
- **Solution**: Stop point = `Lane.StartPosition + direction * (Lane.Length - epsilon)` where epsilon ensures we don't overshoot

### 2. Path Overlap Detection
- **Requirement**: Vehicle can only enter if its intended path doesn't overlap with paths of vehicles currently in intersection
- **Challenge**: Need to determine which turn curves intersect
- **Solution**: Pre-compute turn curve conflicts during road generation (new data structure: `TurnConflictMap`)

### 3. Stop Logic Scope
- **Requirement**: Stop logic ONLY applies to incoming lanes (before entering intersection)
- **Requirement**: Once in intersection, vehicle cannot stop UNLESS there's a car ahead on same path
- **Implementation**: State machine with clear boundaries between `Approaching`, `Crossing`, and `OnLane` states

### 4. Car-to-Car Collision Avoidance
- **Requirement**: Detect stopped cars ahead and brake to avoid overlap
- **Current Status**: Already partially implemented in `CarController.FindCarAhead()`
- **Enhancement Needed**: Must work across state transitions (lane → intersection → lane)

---

## Data Structures

### 1. IntersectionManager (New Class)
```csharp
public class IntersectionManager
{
    // State tracking per node
    private Dictionary<Node, IntersectionState> intersectionStates;
    
    // Pre-computed conflict map
    private Dictionary<TurnCurve, HashSet<TurnCurve>> turnConflicts;
    
    public void Initialize(RoadMap roadMap);
    public bool CanEnter(CarController car, Lane fromLane, TurnCurve intendedTurn, Node intersection);
    public void RegisterEntry(CarController car, Node intersection, TurnCurve turn);
    public void RegisterExit(CarController car, Node intersection);
    public void UpdateCarWaitingAt(CarController car, Lane lane, Node intersection);
    public void RemoveCarWaitingAt(CarController car, Lane lane, Node intersection);
}
```

### 2. IntersectionState (New Class)
```csharp
public class IntersectionState
{
    public Node Node;
    
    // Cars currently crossing through intersection
    public Dictionary<CarController, TurnCurve> activeCrossings;
    
    // Cars waiting to enter, organized by incoming lane
    public Dictionary<Lane, Queue<CarController>> waitingQueues;
    
    // Approaching lanes for this node (cached)
    public List<Lane> incomingLanes;
}
```

### 3. CarController State Machine (Modified)
```csharp
public enum CarState
{
    OnLane,          // Traveling on a lane normally
    Approaching,     // Within detection distance of intersection
    WaitingToEnter,  // Stopped at stop line, waiting for right-of-way
    Crossing,        // Currently in intersection (on turn curve)
}

private CarState currentState = CarState.OnLane;
private Node targetIntersection;
private TurnCurve intendedTurn;
private float stopLineDistance; // Cached stop point for current lane
```

---

## Implementation Steps

### Phase 1: IntersectionManager Setup

#### Step 1.1: Create IntersectionManager.cs
- Implement basic structure
- Add initialization method that processes RoadMap
- Build `IntersectionState` for each Node with 2+ connected roads
- Cache incoming lanes per intersection

#### Step 1.2: Build Turn Conflict Map
- **Goal**: Pre-compute which turn curves intersect spatially
- **Method**: For each pair of turns at same intersection:
  - Sample points along both curves (10 samples each)
  - Check if minimum distance between any two points < threshold (e.g., 2 * lane width)
  - If yes, mark as conflicting
- **Storage**: `Dictionary<TurnCurve, HashSet<TurnCurve>> turnConflicts`

#### Step 1.3: Implement Priority-to-Right Logic
```csharp
private bool HasPriorityVehicleOnRight(CarController requestingCar, Lane fromLane, Node node)
{
    Vector3 carDirection = (fromLane.EndPosition - fromLane.StartPosition).normalized;
    Vector3 nodePos = node.Position;
    
    foreach (var incomingLane in intersectionStates[node].incomingLanes)
    {
        if (incomingLane == fromLane) continue;
        
        // Get direction of other lane
        Vector3 otherDir = (incomingLane.EndPosition - incomingLane.StartPosition).normalized;
        
        // Cross product: positive Y means other lane is to our right
        float cross = Vector3.Cross(carDirection, otherDir).y;
        
        if (cross > 0.1f) // On right side
        {
            // Check if there are cars waiting on that lane
            var queue = intersectionStates[node].waitingQueues[incomingLane];
            if (queue.Count > 0 && queue.Peek() != requestingCar)
            {
                return true; // Vehicle on right has priority
            }
        }
    }
    return false;
}
```

### Phase 2: CarController State Machine

#### Step 2.1: Add State Variables
- Add `CarState currentState`
- Add `Node targetIntersection`
- Add `TurnCurve intendedTurn`
- Add `float stopLineDistance`

#### Step 2.2: Implement Intersection Detection
```csharp
void Update()
{
    // ... existing speed calculation ...
    
    if (currentPathType == PathType.Lane)
    {
        HandleLaneMovement();
    }
    else // PathType.TurnCurve
    {
        HandleIntersectionMovement();
    }
}

void HandleLaneMovement()
{
    float distanceToEnd = currentLane.Length - distanceAlongPath;
    
    // Transition: OnLane -> Approaching
    if (distanceToEnd < INTERSECTION_DETECTION_DISTANCE && currentState == CarState.OnLane)
    {
        currentState = CarState.Approaching;
        targetIntersection = currentLane.EndNode;
        
        // Pre-select turn (random from available turns)
        if (currentLane.AvailableTurns.Count > 0)
        {
            intendedTurn = currentLane.AvailableTurns[Random.Range(0, currentLane.AvailableTurns.Count)];
        }
        else
        {
            Debug.LogError($"Car {CarID}: Lane {currentLane.ID} has no available turns!");
            currentState = CarState.OnLane;
            return;
        }
        
        // Register as waiting
        simulationManager.IntersectionManager.UpdateCarWaitingAt(this, currentLane, targetIntersection);
    }
    
    // Transition: Approaching -> WaitingToEnter or Crossing
    if (distanceToEnd < STOP_LINE_BUFFER && currentState == CarState.Approaching)
    {
        bool canEnter = simulationManager.IntersectionManager.CanEnter(
            this, currentLane, intendedTurn, targetIntersection
        );
        
        if (canEnter)
        {
            // Proceed through intersection
            currentState = CarState.Crossing;
            simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
        }
        else
        {
            // Must wait
            currentState = CarState.WaitingToEnter;
            stopLineDistance = distanceAlongPath; // Remember where we stopped
        }
    }
    
    // Handle WaitingToEnter state
    if (currentState == CarState.WaitingToEnter)
    {
        // Check if we can enter now
        bool canEnter = simulationManager.IntersectionManager.CanEnter(
            this, currentLane, intendedTurn, targetIntersection
        );
        
        if (canEnter)
        {
            currentState = CarState.Crossing;
            simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
            simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
        }
        else
        {
            // Force stop at stop line
            currentSpeed = 0f;
            distanceAlongPath = Mathf.Min(distanceAlongPath, currentLane.Length - 0.1f);
            return; // Don't move
        }
    }
    
    // Normal lane end transition
    if (distanceAlongPath >= currentLane.Length && currentState == CarState.Crossing)
    {
        TransitionToTurnCurve();
    }
}

void HandleIntersectionMovement()
{
    // In intersection - only stop for car ahead, not for right-of-way
    if (distanceAlongPath >= currentTurnCurve.ArcLength)
    {
        // Exiting intersection
        simulationManager.IntersectionManager.RegisterExit(this, targetIntersection);
        
        TransitionToNextLane();
        
        currentState = CarState.OnLane;
        targetIntersection = null;
        intendedTurn = null;
    }
}
```

#### Step 2.3: Modify CalculateTargetSpeed()
```csharp
private float CalculateTargetSpeed()
{
    // State-specific speed rules
    if (currentState == CarState.WaitingToEnter)
    {
        return 0f; // Must stop
    }
    
    // Find car ahead
    CarController carAhead = FindCarAhead();
    
    if (carAhead != null)
    {
        float distanceToCarAhead = CalculateDistanceToCarAhead(carAhead);
        
        // Emergency braking distance
        if (distanceToCarAhead < SafeFollowDistance)
        {
            return Mathf.Min(carAhead.CurrentSpeed * 0.9f, currentSpeed);
        }
        else if (distanceToCarAhead < LookAheadDistance)
        {
            // Gradual slowdown
            float speedFactor = distanceToCarAhead / LookAheadDistance;
            return Mathf.Lerp(carAhead.CurrentSpeed, TargetSpeed, speedFactor);
        }
    }
    
    // Approaching intersection - slow down slightly for safety
    if (currentState == CarState.Approaching)
    {
        return TargetSpeed * 0.8f;
    }
    
    return TargetSpeed;
}
```

### Phase 3: Integration with SimulationManager

#### Step 3.1: Add IntersectionManager Reference
```csharp
public class SimulationManager : MonoBehaviour
{
    // ... existing fields ...
    
    [Header("Intersection Management")]
    private IntersectionManager intersectionManager;
    
    public IntersectionManager IntersectionManager => intersectionManager;
    
    void Start()
    {
        // ... existing initialization ...
        
        // Initialize intersection management
        intersectionManager = new IntersectionManager();
        intersectionManager.Initialize(roadMap);
        
        if (spawnCarsOnStart)
        {
            SpawnCars();
        }
    }
}
```

---

## Constraint Implementation Details

### Constraint 1: Stop Point at Lane Cutoff
```csharp
// In CarController.HandleLaneMovement()
const float STOP_LINE_BUFFER = 0.5f; // Small buffer to avoid overshooting

if (distanceToEnd < STOP_LINE_BUFFER && currentState == CarState.Approaching)
{
    // distanceAlongPath is now at or very close to lane.Length
    // This naturally respects the stop line because Lane.Length = roadLength - stopLineDistanceAtStart
    // The lane is already cut short during generation!
}

// When waiting, clamp position
if (currentState == CarState.WaitingToEnter)
{
    distanceAlongPath = Mathf.Min(distanceAlongPath, currentLane.Length - STOP_LINE_BUFFER);
}
```

### Constraint 2: Path Overlap Detection
```csharp
// In IntersectionManager.CanEnter()
public bool CanEnter(CarController car, Lane fromLane, TurnCurve intendedTurn, Node intersection)
{
    var state = intersectionStates[intersection];
    
    // Check for conflicting active crossings
    foreach (var activeKvp in state.activeCrossings)
    {
        TurnCurve activeTurn = activeKvp.Value;
        
        // Check if turns conflict (pre-computed)
        if (turnConflicts.ContainsKey(intendedTurn) && 
            turnConflicts[intendedTurn].Contains(activeTurn))
        {
            return false; // Path overlaps with active vehicle
        }
    }
    
    // Check priority to right
    if (HasPriorityVehicleOnRight(car, fromLane, intersection))
    {
        return false;
    }
    
    return true;
}
```

### Constraint 3: Stop Logic Scope
```csharp
// State machine ensures this:
// - OnLane: Normal movement, look for cars ahead
// - Approaching: Slow down, prepare to stop, look for cars ahead
// - WaitingToEnter: FORCED STOP, speed = 0, no movement
// - Crossing: Normal movement on turn curve, ONLY stop for car ahead (not right-of-way)

// In HandleIntersectionMovement() - no right-of-way checks, only collision avoidance
void HandleIntersectionMovement()
{
    // Only consider car ahead, not intersection rules
    float targetSpeed = CalculateTargetSpeed(); // This handles car-ahead logic
    
    // No stopping for right-of-way here!
}
```

### Constraint 4: Stopped Car Detection
```csharp
// In CarController.FindCarAhead() - already works across lanes/turns
// Enhancement: Also check cars in WaitingToEnter state

private CarController FindCarAhead()
{
    // Get all cars from simulation manager
    var allCars = simulationManager.GetAllCars();
    
    foreach (var otherCar in allCars)
    {
        if (otherCar == this) continue;
        
        // Check if on same path
        if (currentPathType == PathType.Lane)
        {
            if (otherCar.CurrentLane == currentLane && 
                otherCar.currentPathType == PathType.Lane)
            {
                // Same lane - check if ahead
                if (otherCar.distanceAlongPath > distanceAlongPath)
                {
                    return otherCar;
                }
            }
        }
        else // PathType.TurnCurve
        {
            if (otherCar.CurrentTurnCurve == currentTurnCurve && 
                otherCar.currentPathType == PathType.TurnCurve)
            {
                // Same turn curve - check if ahead
                if (otherCar.distanceAlongPath > distanceAlongPath)
                {
                    return otherCar;
                }
            }
        }
    }
    
    return null;
}
```

---

## Potential Problems & Solutions

### Problem 1: Deadlock at 4-Way Intersection
**Scenario**: 4 cars arrive simultaneously from all 4 directions, all want to go straight
**Issue**: Each car has a car on its right, creating circular dependency
**Solution**: 
- Implement tie-breaker: First car to enter "Approaching" state gets priority
- Add timestamp to waiting queue entries
- Periodically "release" deadlocks after timeout (5 seconds)

```csharp
public class WaitingCarEntry
{
    public CarController Car;
    public float WaitStartTime;
}

// In CanEnter()
if (Time.time - waitStartTime > DEADLOCK_TIMEOUT)
{
    return true; // Force entry to break deadlock
}
```

### Problem 2: Car Stops Mid-Intersection
**Scenario**: Car enters intersection, then car ahead stops on turn curve
**Issue**: Blocking intersection
**Solution**: 
- Already handled by Constraint 3 - cars in intersection only stop for car ahead, which should be rare
- If it happens, it's realistic (gridlock)
- Optional: Add minimum speed in intersection (5 units/sec) to prevent complete stops

### Problem 3: Turn Curve Conflict Detection False Positives
**Scenario**: Two parallel turns (e.g., both going left) detected as conflicting
**Issue**: Reduces intersection throughput unnecessarily
**Solution**: 
- Use more sophisticated geometry check
- Check if curves actually intersect (line-line intersection), not just proximity
- Consider lane width in threshold calculation

```csharp
private bool TurnCurvesConflict(TurnCurve a, TurnCurve b)
{
    // Sample points
    List<Vector3> pointsA = SampleCurve(a, 10);
    List<Vector3> pointsB = SampleCurve(b, 10);
    
    // Check each segment pair
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
```

### Problem 4: Waiting Queue Order Violation
**Scenario**: Car farther back in queue enters before car at front
**Issue**: Unfair, unrealistic behavior
**Solution**: 
- Only allow `queue.Peek()` (front car) to request entry
- Other cars automatically denied in `CanEnter()`

```csharp
public bool CanEnter(CarController car, Lane fromLane, TurnCurve intendedTurn, Node intersection)
{
    var state = intersectionStates[intersection];
    var queue = state.waitingQueues[fromLane];
    
    // Must be first in queue
    if (queue.Count > 0 && queue.Peek() != car)
    {
        return false; // Not your turn yet
    }
    
    // ... rest of checks ...
}
```

### Problem 5: Cars Spawning in Intersection
**Scenario**: Car spawns on lane very close to intersection
**Issue**: Immediately enters intersection without proper state transitions
**Solution**: 
- Modify spawn logic in `SimulationManager.SpawnCars()` to avoid spawning within detection distance of lane end
- Minimum spawn distance: `minSpawnSeparation + INTERSECTION_DETECTION_DISTANCE`

```csharp
// In SimulationManager.SpawnCars()
float maxSpawnDistance = randomLane.Length - INTERSECTION_DETECTION_DISTANCE - 5f;
if (maxSpawnDistance < 5f)
{
    continue; // Lane too short, skip
}
float spawnDistance = Random.Range(0f, maxSpawnDistance);
```

### Problem 6: Car Doesn't Have Public Access to State
**Scenario**: `FindCarAhead()` needs to check other car's state to see if it's stopped
**Issue**: Private fields
**Solution**: 
- Add public properties to CarController
```csharp
public CarState CurrentState => currentState;
public float DistanceAlongPath => distanceAlongPath;
public PathType CurrentPathType => currentPathType;
```

---

## Testing Strategy

### Test 1: 2-Way Intersection (T-Junction)
- Spawn 2 cars: one from each road
- Verify: Car on right proceeds first
- Verify: Car on left waits, then proceeds

### Test 2: 4-Way Intersection (All Stop)
- Spawn 4 cars, one from each direction
- Verify: Priority to right is respected
- Verify: No deadlock (use tie-breaker)

### Test 3: High Traffic
- Spawn 20+ cars
- Verify: No collisions
- Verify: Smooth flow (no excessive waiting)
- Verify: Queues form and clear properly

### Test 4: Following Distance
- Spawn 2 cars on same lane, close together
- Verify: Rear car maintains safe distance
- Verify: Rear car stops when front car stops at intersection

### Test 5: Sharp Angle Intersection
- Test at intersection with 20° angle (minimum allowed)
- Verify: Stop line positioning is correct
- Verify: No lane overlap

---

## Configuration Parameters

Add to `RoadGenerationConfig.cs`:
```csharp
[Header("Intersection Logic")]
[Tooltip("Distance from lane end to start checking for intersection")]
public float IntersectionDetectionDistance = 15f;

[Tooltip("Buffer distance before stop line (prevents overshooting)")]
public float StopLineBuffer = 0.5f;

[Tooltip("Time to wait before forcing entry to break deadlock (seconds)")]
public float DeadlockTimeout = 5f;

[Tooltip("Threshold distance for turn curve conflict detection")]
public float TurnConflictThreshold = 4f; // 2x lane width
```

---

## Implementation Order

1. ✅ Create `IntersectionManager.cs` with basic structure
2. ✅ Create `IntersectionState` class
3. ✅ Implement `Initialize()` method - build state per node
4. ✅ Implement turn conflict pre-computation
5. ✅ Add state machine fields to `CarController`
6. ✅ Implement `HandleLaneMovement()` with state transitions
7. ✅ Implement `HandleIntersectionMovement()`
8. ✅ Implement priority-to-right logic in `IntersectionManager`
9. ✅ Implement `CanEnter()` method
10. ✅ Integrate with `SimulationManager`
11. ✅ Add public properties to `CarController` for state access
12. ✅ Test with 2-way intersection
13. ✅ Test with 4-way intersection
14. ✅ Handle edge cases (deadlock, spawning, etc.)
15. ✅ Update `SETUP_INSTRUCTIONS.md`

---

## Estimated Effort

- **IntersectionManager.cs**: ~150 lines
- **IntersectionState.cs**: ~50 lines
- **CarController.cs modifications**: ~200 lines (state machine)
- **SimulationManager.cs modifications**: ~20 lines
- **Testing & debugging**: 2-3 hours

**Total**: ~400-450 lines of code

---

## Success Criteria

- ✅ No car-to-car collisions at intersections
- ✅ Priority to right is consistently enforced
- ✅ Cars stop exactly at stop line (not beyond)
- ✅ No cars stop mid-intersection (except for car ahead)
- ✅ Queues form and clear in FIFO order
- ✅ No deadlocks lasting > 5 seconds
- ✅ System works with 2-way, 3-way, and 4-way intersections
- ✅ Visual confirmation in Scene view (Gizmos for waiting cars?)
- ✅ CSV logs show proper intersection behavior

---

## Future Enhancements (Post-Stage 4)

1. **Traffic Lights**: Replace priority-to-right with signal control
2. **Stop Signs**: Explicit STOP requirement even if no cars present
3. **Yield Signs**: Lower priority lanes
4. **Roundabouts**: Circular intersection with continuous flow
5. **Pedestrian Crossings**: Additional stop requirements
6. **Emergency Vehicles**: Priority override
7. **Turn Signals**: Visual indication of intended direction

---

*Plan created: 2024-11-14*
*Implementation: Approach 2 - Data-Driven*

