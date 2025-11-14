using UnityEngine;

/// <summary>
/// Controls individual car behavior: movement along lanes, speed control, and pathfinding.
/// Cars follow lanes and turn curves smoothly without teleportation.
/// </summary>
public class CarController : MonoBehaviour
{
    [Header("Car Identification")]
    public int CarID;
    
    [Header("Movement Settings")]
    [Tooltip("Target speed when no obstacles (units/second)")]
    public float TargetSpeed = 20f;
    
    [Tooltip("Maximum acceleration (units/second²)")]
    public float MaxAcceleration = 5f;
    
    [Tooltip("Maximum braking deceleration (units/second²)")]
    public float MaxBraking = 10f;
    
    [Tooltip("Physical length of car model (units) - Z dimension")]
    public float CarLength = 4f;
    
    [Tooltip("Distance to look ahead for obstacles (units)")]
    public float LookAheadDistance = 20f;
    
    [Tooltip("Minimum safe distance to car ahead (units) - braking starts at this distance")]
    public float SafeFollowDistance = 10f;
    
    [Tooltip("Minimum buffer space between stopped cars (units) - added to car length")]
    public float MinimumCarBuffer = 1f;
    
    [Header("Current State")]
    [SerializeField] private float currentSpeed;
    [SerializeField] private float distanceAlongPath;
    
    // Path following state
    public enum PathType { Lane, TurnCurve }
    private PathType currentPathType = PathType.Lane;
    private Lane currentLane;
    private TurnCurve currentTurnCurve;
    
    // Intersection state machine
    public enum CarState
    {
        OnLane,          // Traveling on a lane normally
        Approaching,     // Within detection distance of intersection
        WaitingToEnter,  // Stopped at stop line, waiting for right-of-way
        Crossing,        // Currently in intersection (on turn curve)
    }
    
    [SerializeField] private CarState currentState = CarState.OnLane;
    private Node targetIntersection;
    private TurnCurve intendedTurn;
    
    // Reference to simulation manager
    private SimulationManager simulationManager;
    
    // Configuration cache
    private float INTERSECTION_DETECTION_DISTANCE = 25f;
    private float STOP_LINE_BUFFER = 0.5f;
    
    // Public properties for state access
    public float CurrentSpeed => currentSpeed;
    public Lane CurrentLane => currentLane;
    public TurnCurve CurrentTurnCurve => currentTurnCurve;
    public CarState CurrentCarState => currentState;
    public float DistanceAlongPath => distanceAlongPath;
    public PathType CurrentPathType => currentPathType;
    
    /// <summary>
    /// Initialize car with starting lane and position.
    /// </summary>
    public void Initialize(int id, Lane startLane, float startDistance, SimulationManager manager)
    {
        CarID = id;
        currentLane = startLane;
        distanceAlongPath = startDistance;
        currentPathType = PathType.Lane;
        currentSpeed = 0f;
        currentState = CarState.OnLane;
        simulationManager = manager;
        
        // Load configuration parameters
        var config = manager.GetRoadGenerator()?.GetConfig();
        if (config != null)
        {
            INTERSECTION_DETECTION_DISTANCE = config.IntersectionDetectionDistance;
            STOP_LINE_BUFFER = config.StopLineBuffer;
        }
        
        // Set initial position and rotation
        UpdatePositionAndRotation();
    }
    
    void Update()
    {
        if (simulationManager == null || currentLane == null)
            return;
        
        float deltaTime = Time.deltaTime;
        
        // Calculate target speed (considers obstacles and intersection state)
        float targetSpeed = CalculateTargetSpeed();
        
        // Adjust current speed towards target speed
        if (currentSpeed < targetSpeed)
        {
            currentSpeed = Mathf.Min(currentSpeed + MaxAcceleration * deltaTime, targetSpeed);
        }
        else if (currentSpeed > targetSpeed)
        {
            currentSpeed = Mathf.Max(currentSpeed - MaxBraking * deltaTime, 0f);
        }
        
        // Move along current path
        float distanceDelta = currentSpeed * deltaTime;
        float newDistanceAlongPath = distanceAlongPath + distanceDelta;
        
        // Safety check: Don't move forward if it would cause collision with car ahead
        CarController carAhead = FindCarAhead();
        if (carAhead != null)
        {
            // Calculate predicted position (clamped to path bounds)
            Vector3 predictedPosition;
            if (currentPathType == PathType.Lane)
            {
                float clampedDistance = Mathf.Clamp(newDistanceAlongPath, 0f, currentLane.Length);
                predictedPosition = currentLane.GetPositionAtDistance(clampedDistance);
            }
            else
            {
                float clampedDistance = Mathf.Clamp(newDistanceAlongPath, 0f, currentTurnCurve.ArcLength);
                float t = clampedDistance / currentTurnCurve.ArcLength;
                predictedPosition = currentTurnCurve.GetPositionAtTime(t);
            }
            
            // Calculate center-to-center distance
            float centerToCenterDistance = Vector3.Distance(predictedPosition, carAhead.transform.position);
            
            // Calculate gap between bumpers (accounting for both car lengths)
            float myHalfLength = CarLength * 0.5f;
            float theirHalfLength = carAhead.CarLength * 0.5f;
            float predictedGap = centerToCenterDistance - myHalfLength - theirHalfLength;
            
            // EMERGENCY BRAKE ONLY - only prevent movement if we're dangerously close
            // Normal braking is handled by CalculateTargetSpeed()
            if (predictedGap < MinimumCarBuffer * 0.3f)
            {
                // Critical collision risk - emergency stop
                newDistanceAlongPath = distanceAlongPath;
                currentSpeed = 0f;
            }
        }
        
        distanceAlongPath = newDistanceAlongPath;
        
        // State-aware path management (NO old transition checks here!)
        if (currentPathType == PathType.Lane)
        {
            HandleLaneMovement();
        }
        else // PathType.TurnCurve
        {
            HandleIntersectionMovement();
        }
        
        // Update transform
        UpdatePositionAndRotation();
    }
    
    /// <summary>
    /// Calculate target speed based on obstacles ahead and intersection state.
    /// Accounts for car length and provides smooth braking.
    /// </summary>
    private float CalculateTargetSpeed()
    {
        // Find cars ahead on same path (including stopped cars at intersections)
        CarController carAhead = FindCarAhead();
        
        if (carAhead != null)
        {
            // Calculate distance to car ahead (center to center, 3D world space)
            float centerToCenterDistance = CalculateDistanceToCarAhead(carAhead);
            
            // Calculate actual gap between car bumpers (accounting for both car lengths)
            float myHalfLength = CarLength * 0.5f;
            float theirHalfLength = carAhead.CarLength * 0.5f;
            float gapBetweenBumpers = centerToCenterDistance - myHalfLength - theirHalfLength;
            
            // Target gap: maintain EXACTLY MinimumCarBuffer (not a minimum, but THE spacing)
            float targetGap = MinimumCarBuffer;
            
            // CRITICAL BRAKING ZONE - gradual deceleration to prevent sudden stops
            float criticalZone = targetGap * 2.0f; // Start braking at 2x the target gap
            
            if (gapBetweenBumpers < targetGap * 0.3f)
            {
                // Truly emergency - return 0 immediately
                return 0f;
            }
            else if (gapBetweenBumpers < criticalZone)
            {
                // Gradual braking zone: speed proportional to gap
                // At targetGap, speed = TargetSpeed * 0.3 (slow but not stopped)
                // At targetGap * 0.3, speed = 0 (emergency)
                float brakingFactor = Mathf.Clamp01((gapBetweenBumpers - targetGap * 0.3f) / (criticalZone - targetGap * 0.3f));
                return Mathf.Lerp(0f, TargetSpeed * 0.5f, brakingFactor);
            }
            
            // Check if car ahead is stopped or very slow
            bool carAheadStopped = carAhead.CurrentSpeed < 1.0f;
            
            if (carAheadStopped)
            {
                // Car ahead is stopped - maintain EXACT targetGap distance
                // This creates uniform queue spacing
                float gapError = gapBetweenBumpers - targetGap;
                
                if (Mathf.Abs(gapError) < 0.1f)
                {
                    // Gap is perfect (within 0.1 units), stop
                    return 0f;
                }
                else if (gapError > 0.1f)
                {
                    // Gap too large - creep forward to close it
                    // Speed proportional to gap error (max 8 units/sec for creeping)
                    float creepSpeed = Mathf.Clamp(gapError * 1.5f, 2f, 8f);
                    return creepSpeed;
                }
                else
                {
                    // Gap too small - emergency stop
                    return 0f;
                }
            }
            else
            {
                // Car ahead is moving - match speed while closing to targetGap
                float gapError = gapBetweenBumpers - targetGap;
                
                if (gapBetweenBumpers < SafeFollowDistance)
                {
                    // Close following - match speed but adjust for gap
                    if (gapError > 1f)
                    {
                        // Gap too large, speed up slightly to close it
                        return Mathf.Min(carAhead.CurrentSpeed * 1.05f, TargetSpeed);
                    }
                    else
                    {
                        // Gap good, match speed
                        return Mathf.Min(carAhead.CurrentSpeed * 0.95f, currentSpeed);
                    }
                }
                else if (gapBetweenBumpers < LookAheadDistance)
                {
                    // Within look-ahead - gradually adjust to match speed
                    float adjustmentFactor = (gapBetweenBumpers - SafeFollowDistance) / 
                                           (LookAheadDistance - SafeFollowDistance);
                    return Mathf.Lerp(carAhead.CurrentSpeed, TargetSpeed, adjustmentFactor);
                }
            }
        }
        
        // State-specific speed rules (after car-ahead checks)
        if (currentState == CarState.WaitingToEnter)
        {
            return 0f; // Must stop at intersection
        }
        
        // PRE-APPROACH: If on lane and getting close to intersection, check for queues
        // IMPORTANT: Only apply this if there's NO car ahead on the same lane
        // If there's a car ahead, the car-following logic above handles all speed adjustments
        // SKIP for trivial intersections (only 2 roads) - no collision possible
        if (currentState == CarState.OnLane && currentPathType == PathType.Lane && carAhead == null)
        {
            float distanceToEnd = currentLane.Length - distanceAlongPath;
            float preApproachDistance = INTERSECTION_DETECTION_DISTANCE * 1.5f; // 37.5 units
            
            if (distanceToEnd < preApproachDistance && currentLane.EndNode != null && !currentLane.EndNode.IsTrivialIntersection())
            {
                // Check if there are cars queued at the upcoming intersection
                bool hasQueue = simulationManager.IntersectionManager.HasWaitingCars(currentLane.EndNode);
                
                // Check if there are other cars approaching the same intersection (30 unit radius)
                bool hasApproaching = simulationManager.IntersectionManager.HasApproachingCars(
                    currentLane.EndNode, this, 30f);
                
                if (hasQueue || hasApproaching)
                {
                    // There's congestion (queue or multiple cars converging) - slow down preemptively
                    // From full speed at 37.5 units to 70% speed at 25 units
                    float speedReduction = Mathf.Clamp01((preApproachDistance - distanceToEnd) / (preApproachDistance - INTERSECTION_DETECTION_DISTANCE));
                    float preApproachSpeed = Mathf.Lerp(TargetSpeed, TargetSpeed * 0.7f, speedReduction);
                    
                    return preApproachSpeed;
                }
            }
        }
        
        // Approaching intersection - check if we need to brake for blocked intersection
        // Skip for trivial intersections (2 roads) - they should never have this state anyway
        if (currentState == CarState.Approaching && targetIntersection != null && intendedTurn != null && 
            !targetIntersection.IsTrivialIntersection())
        {
            float distanceToStopLine = currentLane.Length - distanceAlongPath;
            
            // Check if there's already a queue at this intersection
            bool hasQueue = simulationManager.IntersectionManager.HasWaitingCars(targetIntersection);
            
            // If there's a queue, we should join it without checking CanEnter
            // Otherwise, check if we can enter the intersection
            bool shouldBrake = false;
            
            if (hasQueue)
            {
                // There's a queue - we need to brake to join it
                shouldBrake = true;
            }
            else
            {
                // No queue - check if intersection is clear
                bool canEnter = simulationManager.IntersectionManager.CanEnter(
                    this, currentLane, intendedTurn, targetIntersection
                );
                
                shouldBrake = !canEnter;
            }
            
            if (shouldBrake)
            {
                // Intersection blocked - use PHYSICS-BASED braking
                
                // Calculate minimum stopping distance given current speed and max braking
                // Using: v² = u² + 2as → s = v²/(2a)
                float stoppingDistance = (currentSpeed * currentSpeed) / (2f * MaxBraking);
                
                // Add safety margin
                float safeStoppingDistance = stoppingDistance * 1.2f;
                
                if (distanceToStopLine <= STOP_LINE_BUFFER)
                {
                    // At stop line - full stop
                    return 0f;
                }
                else if (distanceToStopLine < safeStoppingDistance)
                {
                    // URGENT: Not enough distance to stop comfortably
                    // Calculate maximum safe speed we can achieve with smooth braking
                    // v = sqrt(2 * a * s)
                    float maxSafeSpeed = Mathf.Sqrt(2f * MaxBraking * 0.7f * distanceToStopLine);
                    
                    return Mathf.Max(maxSafeSpeed, 0f);
                }
                else
                {
                    // Comfortable braking zone - decelerate smoothly to arrive at stop line at low speed
                    // Target: arrive with ~2 m/s so we can stop gently
                    float targetArrivalSpeed = 2f;
                    float distanceForDeceleration = distanceToStopLine - STOP_LINE_BUFFER;
                    
                    // Calculate ideal speed for smooth deceleration: v = sqrt(v_f² + 2as)
                    float idealSpeed = Mathf.Sqrt(targetArrivalSpeed * targetArrivalSpeed + 
                                                  2f * MaxBraking * 0.5f * distanceForDeceleration);
                    
                    return Mathf.Min(idealSpeed, TargetSpeed);
                }
            }
            
            // No queue and intersection is clear - maintain normal approach speed
            return TargetSpeed * 0.95f;
        }
        
        return TargetSpeed;
    }
    
    /// <summary>
    /// Find the car directly ahead on the same path.
    /// </summary>
    private CarController FindCarAhead()
    {
        var allCars = simulationManager.GetAllCars();
        CarController closestAhead = null;
        float closestDistance = float.MaxValue;
        
        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;
            
            // Check if car is on same path segment
            bool onSamePath = false;
            
            if (currentPathType == PathType.Lane && car.currentPathType == PathType.Lane)
            {
                onSamePath = (car.currentLane == this.currentLane);
            }
            else if (currentPathType == PathType.TurnCurve && car.currentPathType == PathType.TurnCurve)
            {
                onSamePath = (car.currentTurnCurve == this.currentTurnCurve);
            }
            
            if (onSamePath)
            {
                float distanceDiff = car.distanceAlongPath - this.distanceAlongPath;
                
                if (distanceDiff > 0 && distanceDiff < closestDistance)
                {
                    closestDistance = distanceDiff;
                    closestAhead = car;
                }
            }
        }
        
        return closestAhead;
    }
    
    /// <summary>
    /// Calculate distance to a car ahead.
    /// Uses 3D world space distance for accuracy on bridges and elevation changes.
    /// </summary>
    private float CalculateDistanceToCarAhead(CarController carAhead)
    {
        // Always use 3D world space distance for accuracy
        // This accounts for bridges, elevation changes, and ensures proper spacing
        Vector3 worldDistance3D = carAhead.transform.position - transform.position;
        float distance3D = worldDistance3D.magnitude;
        
        // For cars on same path, verify they're actually ahead (not behind)
        bool isActuallyAhead = false;
        
        if (currentPathType == PathType.Lane && carAhead.currentPathType == PathType.Lane &&
            carAhead.currentLane == this.currentLane)
        {
            // Same lane - check path distance
            isActuallyAhead = carAhead.distanceAlongPath > this.distanceAlongPath;
        }
        else if (currentPathType == PathType.TurnCurve && carAhead.currentPathType == PathType.TurnCurve &&
                 carAhead.currentTurnCurve == this.currentTurnCurve)
        {
            // Same turn curve - check path distance
            isActuallyAhead = carAhead.distanceAlongPath > this.distanceAlongPath;
        }
        else
        {
            // Different paths - check if ahead using forward direction
            Vector3 toOtherCar = (carAhead.transform.position - transform.position).normalized;
            float dot = Vector3.Dot(transform.forward, toOtherCar);
            isActuallyAhead = dot > 0.5f; // Car is generally in front
        }
        
        // Return distance only if car is actually ahead, otherwise return large value
        return isActuallyAhead ? distance3D : float.MaxValue;
    }
    
    /// <summary>
    /// Handle lane movement logic including intersection detection and state transitions.
    /// </summary>
    private void HandleLaneMovement()
    {
        // Safety check
        if (currentPathType != PathType.Lane)
        {
            Debug.LogError($"Car {CarID}: HandleLaneMovement called but pathType is {currentPathType}!");
            return;
        }
        
        float distanceToEnd = currentLane.Length - distanceAlongPath;
        
        // Transition: OnLane -> Approaching (skip for trivial intersections with only 2 roads)
        if (distanceToEnd < INTERSECTION_DETECTION_DISTANCE && currentState == CarState.OnLane)
        {
            // Verify lane has endpoint
            if (currentLane.EndNode == null)
            {
                Debug.LogError($"Car {CarID}: Lane {currentLane.ID} has no EndNode!");
                return;
            }
            
            // Skip intersection logic for trivial intersections (only 2 roads connected)
            // These are just straight-through continuations with no collision risk
            if (!currentLane.EndNode.IsTrivialIntersection())
            {
                currentState = CarState.Approaching;
                targetIntersection = currentLane.EndNode;
                
                // Pre-select turn (ONLY once!)
                if (currentLane.AvailableTurns.Count > 0)
                {
                    intendedTurn = currentLane.AvailableTurns[Random.Range(0, currentLane.AvailableTurns.Count)];
                    // Debug.Log($"Car {CarID}: Approaching intersection at Node {targetIntersection.ID}, selected turn to lane {intendedTurn.ToLane.ID}");
                }
                else
                {
                    Debug.LogError($"[CarController] Car {CarID}: Lane {currentLane.ID} has no available turns!");
                    currentSpeed = 0f;
                    return;
                }
                
                // Register as waiting
                simulationManager.IntersectionManager.UpdateCarWaitingAt(this, currentLane, targetIntersection);
            }
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
                simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
                // Debug.Log($"Car {CarID}: Granted entry to intersection at Node {targetIntersection.ID}");
            }
            else
            {
                // Must wait
                currentState = CarState.WaitingToEnter;
                // Debug.Log($"Car {CarID}: Waiting to enter intersection at Node {targetIntersection.ID}");
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
                // Debug.Log($"Car {CarID}: Cleared to enter intersection at Node {targetIntersection.ID}");
            }
            else
            {
                // Must wait - but respect car ahead if there is one
                currentSpeed = 0f;
                
                // Check if there's a car ahead on same lane
                CarController carAhead = FindCarAhead();
                if (carAhead != null && carAhead.currentPathType == PathType.Lane && 
                    carAhead.currentLane == this.currentLane)
                {
                    // There's a car ahead - calculate safe position accounting for car lengths
                    // Get car ahead's position in 3D space
                    Vector3 carAheadPosition = carAhead.transform.position;
                    Vector3 myPosition = currentLane.GetPositionAtDistance(distanceAlongPath);
                    
                    // Calculate center-to-center distance
                    float centerDistance = Vector3.Distance(myPosition, carAheadPosition);
                    
                    // Calculate required gap (both half-lengths + buffer)
                    float requiredGap = (CarLength * 0.5f) + (carAhead.CarLength * 0.5f) + MinimumCarBuffer;
                    
                    // If we're too close, move back
                    if (centerDistance < requiredGap)
                    {
                        // Calculate how far back we need to be on the path
                        float moveBackAmount = requiredGap - centerDistance;
                        distanceAlongPath = Mathf.Max(0f, distanceAlongPath - moveBackAmount);
                    }
                }
                else
                {
                    // No car ahead - stop at stop line
                    distanceAlongPath = Mathf.Min(distanceAlongPath, currentLane.Length - 0.1f);
                }
                
                return; // Don't allow further movement
            }
        }
        
        // Transition to turn curve when at lane end
        if (distanceAlongPath >= currentLane.Length)
        {
            // For trivial intersections (2 roads), set up minimal state for direct transition
            bool isTrivial = currentLane.EndNode != null && currentLane.EndNode.IsTrivialIntersection();
            
            if (isTrivial && currentState == CarState.OnLane)
            {
                // Trivial intersection - set up turn and state directly
                if (currentLane.AvailableTurns.Count > 0)
                {
                    intendedTurn = currentLane.AvailableTurns[0]; // Only one turn in trivial intersection
                    currentState = CarState.Crossing;
                    targetIntersection = currentLane.EndNode;
                    TransitionToTurnCurve();
                }
                else
                {
                    Debug.LogError($"Car {CarID}: Trivial intersection but no turns available!");
                    distanceAlongPath = currentLane.Length - 0.01f;
                    currentSpeed = 0f;
                }
            }
            else if (currentState == CarState.Crossing)
            {
                TransitionToTurnCurve();
            }
            else
            {
                // Shouldn't reach lane end without being in Crossing state (unless trivial intersection)
                Debug.LogWarning($"Car {CarID}: Reached lane end in state {currentState}, clamping position");
                distanceAlongPath = currentLane.Length - 0.01f;
                currentSpeed = 0f;
            }
        }
    }
    
    /// <summary>
    /// Handle intersection (turn curve) movement logic.
    /// </summary>
    private void HandleIntersectionMovement()
    {
        // Safety check
        if (currentPathType != PathType.TurnCurve)
        {
            Debug.LogError($"Car {CarID}: HandleIntersectionMovement called but pathType is {currentPathType}!");
            return;
        }
        
        // Should be in Crossing state while on turn curve
        if (currentState != CarState.Crossing)
        {
            Debug.LogWarning($"Car {CarID}: On turn curve but state is {currentState}, correcting to Crossing");
            currentState = CarState.Crossing;
        }
        
        // Exit when turn complete
        if (distanceAlongPath >= currentTurnCurve.ArcLength)
        {
            // Unregister from intersection
            if (targetIntersection != null)
            {
                simulationManager.IntersectionManager.RegisterExit(this, targetIntersection);
                // Debug.Log($"Car {CarID}: Exited intersection at Node {targetIntersection.ID}");
            }
            else
            {
                Debug.LogWarning($"[CarController] Car {CarID}: Completing turn but targetIntersection is null!");
            }
            
            TransitionToNextLane();
        }
    }
    
    /// <summary>
    /// Transition from lane end to turn curve (entering intersection).
    /// MUST be in Crossing state before calling.
    /// </summary>
    private void TransitionToTurnCurve()
    {
        if (currentState != CarState.Crossing)
        {
            Debug.LogError($"Car {CarID}: Attempting to enter intersection without Crossing state!");
            return;
        }
        
        if (intendedTurn == null)
        {
            Debug.LogError($"Car {CarID}: No intended turn selected!");
            currentSpeed = 0f;
            return;
        }
        
        // Use the PRE-SELECTED turn, don't pick a new one
        currentTurnCurve = intendedTurn;
        currentPathType = PathType.TurnCurve;
        distanceAlongPath = 0f;
        
        // Debug.Log($"Car {CarID}: Transitioned to turn curve");
        
        // State remains Crossing - don't change it!
    }
    
    /// <summary>
    /// Transition from turn curve to destination lane (exiting intersection).
    /// Handles full state cleanup.
    /// </summary>
    private void TransitionToNextLane()
    {
        // Transition to new lane
        currentLane = currentTurnCurve.ToLane;
        currentPathType = PathType.Lane;
        distanceAlongPath = 0f;
        
        // NOW clear intersection-related state
        currentState = CarState.OnLane;
        targetIntersection = null;
        intendedTurn = null;
        currentTurnCurve = null; // Clear reference
        
        // Debug.Log($"Car {CarID}: Transitioned to lane {currentLane.ID}");
        
        // Check if new lane is too short - could immediately re-trigger Approaching
        float distanceToNewEnd = currentLane.Length;
        if (distanceToNewEnd < INTERSECTION_DETECTION_DISTANCE)
        {
            // Debug.LogWarning($"Car {CarID}: Transitioned to very short lane {currentLane.ID} (length: {distanceToNewEnd})");
            // This is OK - the next Update() will handle it naturally
        }
    }
    
    /// <summary>
    /// Update car's position and rotation based on current path progress.
    /// </summary>
    private void UpdatePositionAndRotation()
    {
        Vector3 position;
        Vector3 direction;
        
        if (currentPathType == PathType.Lane)
        {
            position = currentLane.GetPositionAtDistance(distanceAlongPath);
            direction = currentLane.GetDirection();
        }
        else // TurnCurve
        {
            float t = distanceAlongPath / currentTurnCurve.ArcLength;
            position = currentTurnCurve.GetPositionAtTime(t);
            direction = currentTurnCurve.GetDirectionAtTime(t);
        }
        
        // Update transform
        transform.position = position;
        
        if (direction.sqrMagnitude > 0.001f)
        {
            transform.rotation = Quaternion.LookRotation(direction, Vector3.up);
        }
    }
    
    /// <summary>
    /// Get current world position (for logging).
    /// </summary>
    public Vector3 GetPosition()
    {
        return transform.position;
    }
}

