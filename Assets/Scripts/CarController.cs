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

    [Tooltip("Minimum buffer space between stopped cars (units) - added to car length")]
    public float MinimumCarBuffer = 1f;

    [Tooltip("Reaction-time headway kept behind moving cars (seconds)")]
    public float FollowHeadway = 0.6f;

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

    // Traffic light stop/go decision. Made ONCE per approach and latched so a
    // car that cannot brake in time commits to clearing the intersection
    // instead of flip-flopping between braking and running every frame.
    private enum LightDecision { None, Stop, Go }
    private LightDecision lightDecision = LightDecision.None;

    // Set when a Stop decision was made inside the comfortable stopping
    // distance (run-light commitment refused): brake harder than the comfort
    // profile so the car still reaches ~0 at the line instead of slamming
    private bool forcedStop = false;

    // Reference to simulation manager
    private SimulationManager simulationManager;

    // Configuration cache
    private float INTERSECTION_DETECTION_DISTANCE = 25f;
    private float STOP_LINE_BUFFER = 0.5f;

    // Turn-priority yield look-ahead: a yielding turn (left/right) gives way to a
    // conflicting higher-priority car that is either already within
    // INTERSECTION_DETECTION_DISTANCE of its stop line (any speed - includes cars
    // stopped or queued at the line) OR arriving within this many seconds (a faster
    // car still farther out). The near horizon is bound to the detection distance on
    // purpose: a car inside it has entered its Approaching state, so yielding to it
    // matches the pre-existing right-of-way behaviour exactly and cannot regress when
    // the detection distance is reconfigured.
    private const float YIELD_TIME_HORIZON = 3f; // seconds of look-ahead for fast, still-distant cars

    // Public properties for state access
    public float CurrentSpeed => currentSpeed;
    public Lane CurrentLane => currentLane;
    public TurnCurve CurrentTurnCurve => currentTurnCurve;
    public CarState CurrentCarState => currentState;
    public float DistanceAlongPath => distanceAlongPath;
    public PathType CurrentPathType => currentPathType;
    // The turn this car pre-selected on approach (null until Approaching). Exposed so
    // the IntersectionManager can apply turn-aware right-of-way before the car stops.
    public TurnCurve IntendedTurn => intendedTurn;

    // True while this car has committed to continuing through a yellow light
    // and is still approaching the stop line (checked by IntersectionManager
    // so the light gate doesn't force an instant stop at the line)
    public bool IsCommittedToRunLight =>
        lightDecision == LightDecision.Go && currentState == CarState.Approaching;

    // Deceleration used for planned (non-emergency) braking; the remaining
    // headroom up to MaxBraking absorbs tracking error and surprises
    private float ComfortableBraking => MaxBraking * 0.7f;

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
        lightDecision = LightDecision.None;
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
        // IntersectionManager is not serialized, so it does not survive a
        // domain reload during play mode - stand still instead of throwing
        // thousands of exceptions per second
        if (simulationManager == null || currentLane == null ||
            simulationManager.IntersectionManager == null)
            return;

        float deltaTime = Time.deltaTime;

        // Locate the nearest car ahead along our travel path once per frame;
        // shared by the speed controller and the emergency stop check
        float gapToCarAhead;
        CarController carAhead = FindCarAhead(out gapToCarAhead);

        // Calculate target speed (considers obstacles and intersection state)
        float targetSpeed = CalculateTargetSpeed(carAhead, gapToCarAhead);

        // Adjust current speed towards target speed
        if (currentSpeed < targetSpeed)
        {
            currentSpeed = Mathf.Min(currentSpeed + MaxAcceleration * deltaTime, targetSpeed);
        }
        else if (currentSpeed > targetSpeed)
        {
            // Brake proportionally to the speed error: gentle for small
            // corrections, full MaxBraking authority for genuine stops
            float brakeRate = Mathf.Min(MaxBraking, (currentSpeed - targetSpeed) * 8f);
            currentSpeed = Mathf.Max(currentSpeed - brakeRate * deltaTime, targetSpeed);
        }

        // Move along current path
        float distanceDelta = currentSpeed * deltaTime;
        float newDistanceAlongPath = distanceAlongPath + distanceDelta;

        // EMERGENCY BRAKE ONLY - never move into the car ahead. Normal spacing
        // is maintained by CalculateTargetSpeed; this is the last resort.
        if (carAhead != null)
        {
            float predictedGap = gapToCarAhead - distanceDelta
                                 - (CarLength + carAhead.CarLength) * 0.5f;
            if (predictedGap < MinimumCarBuffer * 0.3f)
            {
                newDistanceAlongPath = distanceAlongPath;
                currentSpeed = 0f;
            }
        }

        distanceAlongPath = newDistanceAlongPath;

        // State-aware path management
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
    /// Target speed is the minimum of independent constraints (cruise speed,
    /// car ahead, intersection rules) so the tightest one always wins. Each
    /// constraint is smooth and physically trackable, which is what prevents
    /// the sudden stops the old branch-chain logic produced.
    /// </summary>
    private float CalculateTargetSpeed(CarController carAhead, float gapToCarAhead)
    {
        float limit = TargetSpeed;

        if (carAhead != null)
        {
            limit = Mathf.Min(limit, SafeFollowingSpeed(carAhead, gapToCarAhead));
        }

        limit = Mathf.Min(limit, IntersectionSpeedLimit());

        return limit;
    }

    /// <summary>
    /// Fastest speed that still lets us settle in behind the car ahead with
    /// MinimumCarBuffer spacing using comfortable braking (Krauss safe speed
    /// with a reaction-time headway). Monotonic in gap, so braking starts
    /// early and ramps smoothly instead of slamming near the queue tail.
    /// </summary>
    private float SafeFollowingSpeed(CarController carAhead, float centerGap)
    {
        float bumperGap = centerGap - (CarLength + carAhead.CarLength) * 0.5f;

        if (bumperGap < MinimumCarBuffer * 0.3f)
        {
            return 0f; // dangerously close - stop immediately
        }

        float freeGap = Mathf.Max(0f, bumperGap - MinimumCarBuffer);
        float leaderSpeed = carAhead.CurrentSpeed;
        float headway = ComfortableBraking * FollowHeadway;

        return Mathf.Max(0f, -headway + Mathf.Sqrt(headway * headway +
                         leaderSpeed * leaderSpeed + 2f * ComfortableBraking * freeGap));
    }

    /// <summary>
    /// Speed limit imposed by the intersection at the end of the current lane:
    /// traffic lights (with a latched stop/go decision), right-of-way and
    /// conflict gates, and the don't-block-the-box rule.
    /// </summary>
    private float IntersectionSpeedLimit()
    {
        // Only relevant while on a lane heading into a managed intersection
        if (currentPathType != PathType.Lane || currentState == CarState.Crossing)
            return TargetSpeed;
        if (currentLane.EndNode == null || currentLane.EndNode.IsTrivialIntersection())
            return TargetSpeed;

        if (currentState == CarState.WaitingToEnter)
        {
            return 0f; // Must stop at intersection
        }

        float distanceToStopLine = currentLane.Length - distanceAlongPath;
        if (distanceToStopLine > INTERSECTION_DETECTION_DISTANCE * 1.5f)
            return TargetSpeed;

        Node intersection = currentLane.EndNode;
        float limit = TargetSpeed * 0.95f; // cautious default approach

        if (simulationManager.IntersectionManager.HasTrafficLight(intersection))
        {
            var lightState = simulationManager.IntersectionManager.GetLightState(intersection, currentLane);
            UpdateLightDecision(lightState, distanceToStopLine);

            if (lightDecision == LightDecision.Go)
            {
                limit = TargetSpeed; // committed: clear the intersection promptly
            }
            else if (lightState == StopLightController.LightState.Green)
            {
                limit = TargetSpeed * 0.85f; // stay stoppable in case of yellow
            }
            else
            {
                // Red, or a yellow we decided to stop for - brake to the line
                float decel = forcedStop ? MaxBraking * 0.95f : ComfortableBraking;
                return BrakingSpeedForDistance(distanceToStopLine - STOP_LINE_BUFFER, decel);
            }
        }

        // Right-of-way / conflict / downstream-space gate (applies on green too)
        if (currentState == CarState.Approaching && intendedTurn != null && targetIntersection != null)
        {
            if (!CanProceedIntoIntersection())
            {
                // Brake on the accelerate-then-stop envelope (harmonic blend of
                // MaxAcceleration and ComfortableBraking), not the pure braking
                // curve: a queued car restarting a few units short of the line
                // accelerates under the pure curve and only meets it at the line,
                // arriving too fast to hold and eligible for the escape hatch in
                // CanProceedIntoIntersection. The blended curve is the fastest
                // profile that can still stop at the line from a standing restart
                // anywhere along the approach, so arrival stays below the hatch's
                // mover threshold and the yield gate keeps its authority.
                float restartSafeDecel = MaxAcceleration * ComfortableBraking
                                         / (MaxAcceleration + ComfortableBraking);
                limit = Mathf.Min(limit, BrakingSpeedForDistance(distanceToStopLine - STOP_LINE_BUFFER, restartSafeDecel));
            }
        }

        return limit;
    }

    /// <summary>
    /// Full entry check: manager grants entry AND the exit lane has room for
    /// us ("don't block the box"). A designated deadlock breaker skips the
    /// box rule - it must force through to unjam the intersection.
    /// </summary>
    private bool CanProceedIntoIntersection()
    {
        bool canEnter = simulationManager.IntersectionManager.CanEnter(
            this, currentLane, intendedTurn, targetIntersection
        );

        if (!canEnter)
        {
            return false;
        }

        // Box rule and yield rule don't apply to a committed run-light car
        // (it can't stop anyway) or a designated deadlock breaker
        if (lightDecision == LightDecision.Go ||
            simulationManager.IntersectionManager.IsDeadlockBreaker(this, targetIntersection))
        {
            return true;
        }

        if (HasRoomBeyondTurn(intendedTurn) && !MustYieldToApproachingTraffic() && !MustYieldByTurnPriority())
        {
            return true;
        }

        // Escape hatch: if the box/yield constraint materialized inside our
        // physical stopping distance, enter anyway - cross-boundary following
        // parks us on the curve if needed, which beats an instant stop. Only
        // a genuine mover qualifies: a slow or stopped car can always hold,
        // and a car at the line has negative distanceToStop, so without the
        // speed gate every waiting car would take the hatch and bypass the
        // yield rules the moment the light turns green.
        if (currentSpeed < 3f)
        {
            return false;
        }

        float distanceToStop = currentLane.Length - distanceAlongPath - STOP_LINE_BUFFER;
        float minStoppingDistance = (currentSpeed * currentSpeed) / (2f * MaxBraking);
        return distanceToStop < minStoppingDistance * 0.9f;
    }

    /// <summary>
    /// True if a conflicting car is bearing down on this intersection too
    /// fast to stop. Entering now would deny it at the last moment and force
    /// an impossible brake, so we hold at the line instead - the same way a
    /// permissive left turn yields to oncoming traffic.
    /// </summary>
    private bool MustYieldToApproachingTraffic()
    {
        if (intendedTurn == null || targetIntersection == null)
            return false;

        var allCars = simulationManager.GetAllCars();

        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;
            if (car.currentPathType != PathType.Lane || car.currentState != CarState.Approaching)
                continue;
            if (car.currentLane == null || car.currentLane.EndNode != targetIntersection)
                continue;
            if (car.currentLane == currentLane)
                continue; // same-lane cars are queue-ordered, not yielded to
            if (car.intendedTurn == null || car.currentSpeed < 3f)
                continue; // slow enough to stop and queue normally
            if (!simulationManager.IntersectionManager.TurnsMayConflict(intendedTurn, car.intendedTurn))
                continue;

            float theirDistance = car.currentLane.Length - car.distanceAlongPath;
            float theirStoppingDistance = (car.currentSpeed * car.currentSpeed) / (2f * car.MaxBraking * 0.85f);
            if (theirDistance < theirStoppingDistance * 1.25f + 2f)
            {
                return true; // they cannot stop in time - let them through first
            }
        }

        return false;
    }

    /// <summary>
    /// Turn-type right-of-way (right-hand traffic): straight beats right beats
    /// left. A yielding movement (left, or right vs a straight) gives way to any
    /// conflicting higher-priority car that is close enough to matter: either within
    /// INTERSECTION_DETECTION_DISTANCE of its stop line at ANY speed (so we yield to
    /// a stopped or crawling car queued at the line and never cut in front of it), or
    /// arriving within YIELD_TIME_HORIZON seconds (so we also yield to a faster car
    /// that is still farther out - the case the old state-gated check missed and let
    /// a left cut in). We skip a car only when it is both beyond the detection
    /// distance AND slower to arrive than the time horizon. A car that has not chosen
    /// its turn yet (still OnLane) is treated by its worst plausible (highest-priority
    /// conflicting) movement. Approach angle is irrelevant - only the turn types and
    /// whether the paths actually cross. At a signal, cars held on a red phase are
    /// ignored so we don't stall on our own green.
    /// </summary>
    private bool MustYieldByTurnPriority()
    {
        if (intendedTurn == null || targetIntersection == null)
            return false;

        int myPriority = TurnPriority(intendedTurn.Type);
        if (myPriority >= 2)
            return false; // a straight movement never yields by turn type

        var manager = simulationManager.IntersectionManager;
        bool signalized = manager.HasTrafficLight(targetIntersection);

        var allCars = simulationManager.GetAllCars();
        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;
            if (car.currentPathType != PathType.Lane || car.currentLane == null)
                continue;
            if (car.currentLane == currentLane || car.currentLane.EndNode != targetIntersection)
                continue;
            // A car already in the box is handled by the active-crossing check in
            // CanEnter; don't double-count it here.
            if (car.currentState == CarState.Crossing)
                continue;

            // Consider this car only if it is close enough to matter: near the line
            // at any speed (a stopped/queued higher-priority car we must not cut in
            // front of) or arriving soon (a faster car still farther out). Skip it
            // only when it is both beyond the detection distance and slow to arrive.
            float theirDistToLine = car.currentLane.Length - car.distanceAlongPath;
            float theirTimeToArrival = theirDistToLine / Mathf.Max(car.currentSpeed, 0.1f);
            if (theirDistToLine > INTERSECTION_DETECTION_DISTANCE && theirTimeToArrival > YIELD_TIME_HORIZON)
                continue; // genuinely far and not arriving soon - a large enough gap

            // A car parked short of the line is boxed in behind its own queue
            // head (queue spacing keeps followers 4+ units back) and cannot be
            // the next to enter from its lane, so it holds no immediate claim.
            // Yielding to it stalls both approaches when its own head is a
            // yielding turn too: each head waits on a follower that can never
            // arrive while the heads hold.
            if (car.currentSpeed < 0.5f && theirDistToLine > 3f)
                continue;

            // Only give way to a strictly higher-priority movement whose path
            // crosses ours. An OnLane car has no chosen turn yet, so assume its
            // worst plausible movement.
            if (!HasHigherPriorityConflict(car, myPriority, manager))
                continue;

            // Ignore cars that aren't entitled to move (held at a red on a
            // different phase); yielding to them would stall us on our own green.
            if (signalized)
            {
                var theirLight = manager.GetLightState(targetIntersection, car.currentLane);
                if (theirLight != StopLightController.LightState.Green && !car.IsCommittedToRunLight)
                    continue;
            }

            return true; // a conflicting higher-priority car is close enough - yield
        }

        return false;
    }

    /// <summary>
    /// Whether <paramref name="car"/> has (or, if it has not chosen a turn yet,
    /// could have) a movement that both outranks our priority and crosses our
    /// intended turn. OnLane cars have no assigned turn, so every turn their lane
    /// offers is considered and the worst case wins - this is what stops a left
    /// from ignoring a straight that simply has not started its approach yet.
    /// </summary>
    private bool HasHigherPriorityConflict(CarController car, int myPriority, IntersectionManager manager)
    {
        if (car.intendedTurn != null)
        {
            return TurnPriority(car.intendedTurn.Type) > myPriority
                && manager.TurnsMayConflict(intendedTurn, car.intendedTurn);
        }

        if (car.currentLane.AvailableTurns == null)
            return false;

        foreach (var candidate in car.currentLane.AvailableTurns)
        {
            if (TurnPriority(candidate.Type) > myPriority
                && manager.TurnsMayConflict(intendedTurn, candidate))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Right-of-way rank of a turn type: straight (2) beats right (1) beats
    /// left (0). Used so left turns yield to straights and rights, and rights
    /// yield to straights, independent of which approach they come from.
    /// </summary>
    private static int TurnPriority(TurnCurve.TurnType type)
    {
        if (type == TurnCurve.TurnType.Straight) return 2;
        if (type == TurnCurve.TurnType.Right) return 1;
        return 0; // Left
    }

    /// <summary>
    /// Maintain the once-per-approach stop/go decision for traffic lights.
    /// Go is only ever chosen on a yellow we physically cannot stop for, and
    /// only if the IntersectionManager confirms no conflicting car has
    /// already committed to running the same phase change.
    /// </summary>
    private void UpdateLightDecision(StopLightController.LightState lightState, float distanceToStopLine)
    {
        if (lightState == StopLightController.LightState.Green)
        {
            SetLightDecision(LightDecision.None); // fresh decision at next yellow
            return;
        }

        // A car that ended up (nearly) stopped has no business running the light
        if (lightDecision == LightDecision.Go && currentSpeed < 1f)
        {
            SetLightDecision(LightDecision.Stop);
            return;
        }

        // A refused commitment (forced stop) gets retried while the yellow
        // lasts: if the conflicting crosser clears in time, running the light
        // beats the physically impossible stop we were left with
        if (lightDecision == LightDecision.Stop && forcedStop && currentSpeed > 3f &&
            lightState == StopLightController.LightState.Yellow && intendedTurn != null &&
            simulationManager.IntersectionManager.TryCommitRunLight(this, currentLane.EndNode, intendedTurn))
        {
            SetLightDecision(LightDecision.Go);
            return;
        }

        if (lightDecision != LightDecision.None)
            return; // decision already latched for this approach

        if (lightState == StopLightController.LightState.Yellow)
        {
            // Stop unless that would take more than ~85% of maximum braking;
            // the margin above ComfortableBraking absorbs tracking error
            float stoppingDistance = (currentSpeed * currentSpeed) / (2f * MaxBraking * 0.85f);
            bool canStop = distanceToStopLine - STOP_LINE_BUFFER > stoppingDistance;

            if (!canStop && intendedTurn != null &&
                simulationManager.IntersectionManager.TryCommitRunLight(this, currentLane.EndNode, intendedTurn))
            {
                SetLightDecision(LightDecision.Go);
            }
            else
            {
                SetLightDecision(LightDecision.Stop);
                forcedStop = !canStop; // commitment refused: brake hard, not comfortably
            }
        }
        else
        {
            // First observed on red: always stop, never run a stale red
            SetLightDecision(LightDecision.Stop);
        }
    }

    private void SetLightDecision(LightDecision decision)
    {
        if (decision == lightDecision)
            return;

        // Release any run-light commitment held with the manager
        if (lightDecision == LightDecision.Go && simulationManager != null)
        {
            Node node = targetIntersection ?? currentLane?.EndNode;
            if (node != null)
            {
                simulationManager.IntersectionManager.CancelRunLight(this, node);
            }
        }

        lightDecision = decision;
        if (decision != LightDecision.Stop)
        {
            forcedStop = false;
        }
    }

    /// <summary>
    /// Speed limit that decelerates smoothly to a stop over the given distance
    /// at the given deceleration, with a small creep floor so cars actually
    /// reach the stop line instead of parking short of it.
    /// </summary>
    private float BrakingSpeedForDistance(float distance, float deceleration)
    {
        if (distance <= 0f)
        {
            return 0f;
        }

        float speed = Mathf.Sqrt(2f * deceleration * distance);
        return Mathf.Max(speed, 1.5f);
    }

    /// <summary>
    /// Check the destination lane of a turn has space for this car to fully
    /// exit the intersection ("don't block the box"). Cars still crossing
    /// toward the same lane reserve space too, so the answer flips early -
    /// while we can still brake comfortably - rather than at the last moment.
    /// </summary>
    private bool HasRoomBeyondTurn(TurnCurve turn)
    {
        if (turn == null)
            return true;

        var allCars = simulationManager.GetAllCars();
        float reservedByCrossers = 0f;
        float firstCarDistance = float.MaxValue;
        float firstCarHalfLength = 0f;

        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;

            if (car.currentPathType == PathType.Lane && car.currentLane == turn.ToLane)
            {
                if (car.distanceAlongPath < firstCarDistance)
                {
                    firstCarDistance = car.distanceAlongPath;
                    firstCarHalfLength = car.CarLength * 0.5f;
                }
            }
            else if (car.currentPathType == PathType.TurnCurve && car.currentTurnCurve != null &&
                     car.currentTurnCurve.ToLane == turn.ToLane)
            {
                // Will land near the lane start before we get there
                reservedByCrossers += car.CarLength + MinimumCarBuffer;
            }
        }

        if (firstCarDistance == float.MaxValue)
        {
            firstCarDistance = turn.ToLane.Length;
            firstCarHalfLength = 0f;
        }

        float required = CarLength * 0.5f + firstCarHalfLength + MinimumCarBuffer + reservedByCrossers;
        return firstCarDistance >= required;
    }

    /// <summary>
    /// Find the nearest car ahead along our travel path, looking across
    /// lane->turn and turn->lane boundaries so leaders never vanish at
    /// transitions. Returns the along-path center-to-center gap.
    /// </summary>
    private CarController FindCarAhead(out float centerGap)
    {
        centerGap = float.MaxValue;
        CarController closestAhead = null;
        var allCars = simulationManager.GetAllCars();

        // The turn we expect to take from the current lane, if predictable
        TurnCurve nextTurn = null;
        if (currentPathType == PathType.Lane)
        {
            nextTurn = intendedTurn;
            if (nextTurn == null && currentLane.EndNode != null &&
                currentLane.EndNode.IsTrivialIntersection() && currentLane.AvailableTurns.Count > 0)
            {
                nextTurn = currentLane.AvailableTurns[0];
            }
        }

        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;

            float gap = float.MaxValue;

            if (currentPathType == PathType.Lane)
            {
                float remaining = currentLane.Length - distanceAlongPath;

                if (car.currentPathType == PathType.Lane && car.currentLane == currentLane)
                {
                    float diff = car.distanceAlongPath - distanceAlongPath;
                    if (diff > 0f)
                        gap = diff;
                }
                else if (car.currentPathType == PathType.TurnCurve && car.currentTurnCurve != null &&
                         car.currentTurnCurve.FromLane == currentLane)
                {
                    // Leader is crossing the intersection we're heading into
                    gap = remaining + car.distanceAlongPath;
                }
                else if (nextTurn != null && car.currentPathType == PathType.Lane &&
                         car.currentLane == nextTurn.ToLane)
                {
                    // Leader is on the lane we'll enter after our turn
                    gap = remaining + nextTurn.ArcLength + car.distanceAlongPath;
                }
            }
            else // on a TurnCurve
            {
                float remaining = currentTurnCurve.ArcLength - distanceAlongPath;

                if (car.currentPathType == PathType.TurnCurve && car.currentTurnCurve == currentTurnCurve)
                {
                    float diff = car.distanceAlongPath - distanceAlongPath;
                    if (diff > 0f)
                        gap = diff;
                }
                else if (car.currentPathType == PathType.Lane && car.currentLane == currentTurnCurve.ToLane)
                {
                    // Leader is on our destination lane
                    gap = remaining + car.distanceAlongPath;
                }
            }

            if (gap < centerGap)
            {
                centerGap = gap;
                closestAhead = car;
            }
        }

        return closestAhead;
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
            if (CanProceedIntoIntersection())
            {
                // Proceed through intersection
                currentState = CarState.Crossing;
                simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
                simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
                SetLightDecision(LightDecision.None);
            }
            else
            {
                // Must wait
                currentState = CarState.WaitingToEnter;
                SetLightDecision(LightDecision.None);
            }
        }

        // Handle WaitingToEnter state
        if (currentState == CarState.WaitingToEnter)
        {
            // Check if we can enter now
            if (CanProceedIntoIntersection())
            {
                currentState = CarState.Crossing;
                simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
                simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
                SetLightDecision(LightDecision.None);
            }
            else
            {
                // Must wait
                currentSpeed = 0f;

                // Resolve any overlap with a car ahead on the same lane
                // gradually (rate-limited) instead of teleporting backwards
                float gapAhead;
                CarController carAhead = FindCarAhead(out gapAhead);
                if (carAhead != null && carAhead.currentPathType == PathType.Lane &&
                    carAhead.currentLane == currentLane)
                {
                    float requiredGap = (CarLength + carAhead.CarLength) * 0.5f + MinimumCarBuffer;
                    if (gapAhead < requiredGap)
                    {
                        float correction = Mathf.Min(requiredGap - gapAhead, 2f * Time.deltaTime);
                        distanceAlongPath = Mathf.Max(0f, distanceAlongPath - correction);
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

        // Carry the distance travelled past the lane end onto the curve so
        // the transition doesn't cost a fraction of a frame's movement
        float overshoot = Mathf.Max(0f, distanceAlongPath - currentLane.Length);

        // Use the PRE-SELECTED turn, don't pick a new one
        currentTurnCurve = intendedTurn;
        currentPathType = PathType.TurnCurve;
        distanceAlongPath = Mathf.Min(overshoot, currentTurnCurve.ArcLength);

        // State remains Crossing - don't change it!
    }

    /// <summary>
    /// Transition from turn curve to destination lane (exiting intersection).
    /// Handles full state cleanup and checks for cars already on destination lane.
    /// </summary>
    private void TransitionToNextLane()
    {
        // Get destination lane
        Lane destinationLane = currentTurnCurve.ToLane;

        // Carry overshoot past the curve end onto the new lane
        float overshoot = Mathf.Max(0f, distanceAlongPath - currentTurnCurve.ArcLength);

        // Check for cars already on the destination lane and find safe starting position
        float safeStartDistance = FindSafeStartPositionOnLane(destinationLane);

        // Release any light commitment before intersection state is cleared
        SetLightDecision(LightDecision.None);

        // Transition to new lane
        currentLane = destinationLane;
        currentPathType = PathType.Lane;
        distanceAlongPath = Mathf.Max(safeStartDistance, Mathf.Min(overshoot, destinationLane.Length));

        // NOW clear intersection-related state
        currentState = CarState.OnLane;
        targetIntersection = null;
        intendedTurn = null;
        currentTurnCurve = null; // Clear reference
    }

    /// <summary>
    /// Find a safe starting position on a lane, accounting for any cars already on it.
    /// Cross-boundary following means we normally arrive already slowed, so
    /// starting at 0 and braking normally is safe; the pathological packed-lane
    /// case (which the box rule prevents) additionally clamps our speed.
    /// </summary>
    private float FindSafeStartPositionOnLane(Lane lane)
    {
        var allCars = simulationManager.GetAllCars();
        float closestCarDistance = float.MaxValue;
        CarController closestCar = null;

        foreach (var car in allCars)
        {
            if (car == this || car == null)
                continue;

            // Check if car is on the destination lane
            if (car.currentPathType == PathType.Lane && car.currentLane == lane)
            {
                if (car.distanceAlongPath < closestCarDistance)
                {
                    closestCarDistance = car.distanceAlongPath;
                    closestCar = car;
                }
            }
        }

        if (closestCar != null)
        {
            float requiredGap = (CarLength + closestCar.CarLength) * 0.5f + MinimumCarBuffer;
            if (closestCarDistance < requiredGap)
            {
                // Destination is packed to the lane start - hold at the mouth
                // and match the queue tail so we don't drive into it
                currentSpeed = Mathf.Min(currentSpeed, closestCar.CurrentSpeed);
            }
        }

        return 0f; // start at the beginning of the lane and brake normally
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
            // Map arc distance to the Bezier parameter through the arc-length
            // table; naive t = distance/ArcLength makes cars lurch through turns
            float t = currentTurnCurve.GetTForArcDistance(distanceAlongPath);
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
