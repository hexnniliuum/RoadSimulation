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
    private float INTERSECTION_DETECTION_DISTANCE = 15f;
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
            
            // If predicted movement would violate minimum buffer, prevent movement
            if (predictedGap < MinimumCarBuffer)
            {
                // Don't move forward - maintain current position
                newDistanceAlongPath = distanceAlongPath;
                currentSpeed = 0f; // Force stop
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
            
            // Minimum safe gap (buffer only, car lengths already subtracted)
            float minimumGap = MinimumCarBuffer;
            
            // Emergency stop if gap is too small
            if (gapBetweenBumpers < minimumGap)
            {
                return 0f; // Critical: prevent collision
            }
            
            // Critical braking zone: very close to car ahead
            if (gapBetweenBumpers < minimumGap + 2f)
            {
                // Aggressive braking but proportional to gap
                float urgencyFactor = (gapBetweenBumpers - minimumGap) / 2f;
                return Mathf.Max(carAhead.CurrentSpeed * 0.5f, currentSpeed * urgencyFactor);
            }
            
            // Check if car ahead is stopped or very slow
            bool carAheadStopped = carAhead.CurrentSpeed < 1.0f;
            
            if (carAheadStopped)
            {
                // Approaching a stopped car - gradual deceleration based on distance
                float stoppingDistance = SafeFollowDistance;
                
                if (gapBetweenBumpers < stoppingDistance)
                {
                    // Linear deceleration: speed proportional to distance
                    float speedFactor = gapBetweenBumpers / stoppingDistance;
                    float targetStoppingSpeed = TargetSpeed * speedFactor;
                    
                    // Ensure we're slowing down, not speeding up
                    return Mathf.Min(targetStoppingSpeed, currentSpeed * 0.9f);
                }
            }
            else
            {
                // Car ahead is moving - follow at safe distance
                if (gapBetweenBumpers < SafeFollowDistance)
                {
                    // Close following - match speed with slight reduction
                    return Mathf.Min(carAhead.CurrentSpeed * 0.95f, currentSpeed);
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
        
        // Approaching intersection - slow down slightly for safety
        if (currentState == CarState.Approaching)
        {
            return TargetSpeed * 0.85f;
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
        
        // Transition: OnLane -> Approaching
        if (distanceToEnd < INTERSECTION_DETECTION_DISTANCE && currentState == CarState.OnLane)
        {
            // Verify lane has endpoint
            if (currentLane.EndNode == null)
            {
                Debug.LogError($"Car {CarID}: Lane {currentLane.ID} has no EndNode!");
                return;
            }
            
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
        
        // Transition to turn curve ONLY when in Crossing state and at lane end
        if (distanceAlongPath >= currentLane.Length)
        {
            if (currentState == CarState.Crossing)
            {
                TransitionToTurnCurve();
            }
            else
            {
                // Shouldn't reach lane end without being in Crossing state
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

