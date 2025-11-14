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
    
    [Tooltip("Distance to look ahead for obstacles (units)")]
    public float LookAheadDistance = 15f;
    
    [Tooltip("Minimum safe distance to car ahead (units)")]
    public float SafeFollowDistance = 5f;
    
    [Header("Current State")]
    [SerializeField] private float currentSpeed;
    [SerializeField] private float distanceAlongPath;
    
    // Path following state
    private enum PathType { Lane, TurnCurve }
    private PathType currentPathType = PathType.Lane;
    private Lane currentLane;
    private TurnCurve currentTurnCurve;
    
    // Reference to simulation manager
    private SimulationManager simulationManager;
    
    public float CurrentSpeed => currentSpeed;
    public Lane CurrentLane => currentLane;
    public TurnCurve CurrentTurnCurve => currentTurnCurve;
    
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
        simulationManager = manager;
        
        // Set initial position and rotation
        UpdatePositionAndRotation();
    }
    
    void Update()
    {
        if (simulationManager == null || currentLane == null)
            return;
        
        float deltaTime = Time.deltaTime;
        
        // Check for obstacles ahead
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
        distanceAlongPath += distanceDelta;
        
        // Check if we've reached the end of current path segment
        if (currentPathType == PathType.Lane)
        {
            if (distanceAlongPath >= currentLane.Length)
            {
                TransitionToNextPath();
            }
        }
        else // TurnCurve
        {
            if (distanceAlongPath >= currentTurnCurve.ArcLength)
            {
                TransitionToNextPath();
            }
        }
        
        // Update transform
        UpdatePositionAndRotation();
    }
    
    /// <summary>
    /// Calculate target speed based on obstacles ahead.
    /// </summary>
    private float CalculateTargetSpeed()
    {
        // Find cars ahead on same path
        CarController carAhead = FindCarAhead();
        
        if (carAhead != null)
        {
            // Calculate distance to car ahead
            float distanceToCarAhead = CalculateDistanceToCarAhead(carAhead);
            
            if (distanceToCarAhead < SafeFollowDistance)
            {
                // Too close - match speed or brake
                return Mathf.Min(carAhead.CurrentSpeed * 0.9f, currentSpeed);
            }
            else if (distanceToCarAhead < LookAheadDistance)
            {
                // Within look-ahead distance - gradually match speed
                float speedFactor = (distanceToCarAhead - SafeFollowDistance) / 
                                   (LookAheadDistance - SafeFollowDistance);
                return Mathf.Lerp(carAhead.CurrentSpeed, TargetSpeed, speedFactor);
            }
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
    /// </summary>
    private float CalculateDistanceToCarAhead(CarController carAhead)
    {
        if (currentPathType == PathType.Lane && carAhead.currentPathType == PathType.Lane &&
            carAhead.currentLane == this.currentLane)
        {
            return carAhead.distanceAlongPath - this.distanceAlongPath;
        }
        else if (currentPathType == PathType.TurnCurve && carAhead.currentPathType == PathType.TurnCurve &&
                 carAhead.currentTurnCurve == this.currentTurnCurve)
        {
            return carAhead.distanceAlongPath - this.distanceAlongPath;
        }
        
        // Different paths - use world space distance as approximation
        return Vector3.Distance(transform.position, carAhead.transform.position);
    }
    
    /// <summary>
    /// Transition to next path segment (lane or turn curve).
    /// </summary>
    private void TransitionToNextPath()
    {
        if (currentPathType == PathType.Lane)
        {
            // At end of lane - choose a turn curve
            if (currentLane.AvailableTurns.Count > 0)
            {
                // Random turn selection (excluding turning back)
                TurnCurve selectedTurn = currentLane.AvailableTurns[Random.Range(0, currentLane.AvailableTurns.Count)];
                
                currentTurnCurve = selectedTurn;
                currentPathType = PathType.TurnCurve;
                distanceAlongPath = 0f;
            }
            else
            {
                // No turns available - shouldn't happen in a proper network
                Debug.LogWarning($"Car {CarID}: No available turns from lane {currentLane.ID}");
                currentSpeed = 0f;
            }
        }
        else // TurnCurve
        {
            // At end of turn curve - transition to destination lane
            currentLane = currentTurnCurve.ToLane;
            currentPathType = PathType.Lane;
            distanceAlongPath = 0f;
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

