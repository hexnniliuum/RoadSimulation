using UnityEngine;

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

    public enum PathType { Lane, TurnCurve }
    private PathType currentPathType = PathType.Lane;
    private Lane currentLane;
    private TurnCurve currentTurnCurve;

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

    // Latched once per approach: a car that can't brake in time commits to
    // clearing the intersection instead of flip-flopping every frame.
    private enum LightDecision { None, Stop, Go }
    private LightDecision lightDecision = LightDecision.None;

    // Set when a Stop decision was forced inside the comfortable stopping
    // distance; brakes harder than the comfort profile to still reach ~0 at the line.
    private bool forcedStop = false;

    private SimulationManager simulationManager;

    // Defaults; overwritten from RoadConfig in Initialize()
    private float INTERSECTION_DETECTION_DISTANCE = 25f;
    private float STOP_LINE_BUFFER = 0.5f;
    private float ROUNDABOUT_SPEED_LIMIT = 8f;

    // Near-horizon yield distance is INTERSECTION_DETECTION_DISTANCE itself (ties it
    // to the Approaching state so it can't drift if reconfigured); this constant only
    // extends the look-ahead to catch a fast car that is still farther out.
    private const float YIELD_TIME_HORIZON = 3f;

    public float CurrentSpeed => currentSpeed;
    public Lane CurrentLane => currentLane;
    public TurnCurve CurrentTurnCurve => currentTurnCurve;
    public CarState CurrentCarState => currentState;
    public float DistanceAlongPath => distanceAlongPath;
    public PathType CurrentPathType => currentPathType;
    // The turn this car pre-selected on approach (null until Approaching). Exposed so
    // the IntersectionManager can apply turn-aware right-of-way before the car stops.
    public TurnCurve IntendedTurn => intendedTurn;

    // True while committed to running a yellow and still approaching; lets
    // IntersectionManager's light gate skip forcing an instant stop at the line.
    public bool IsCommittedToRunLight =>
        lightDecision == LightDecision.Go && currentState == CarState.Approaching;

    // Planned (non-emergency) braking rate; headroom up to MaxBraking absorbs
    // tracking error and surprises.
    private float ComfortableBraking => MaxBraking * 0.7f;

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

        var config = manager.GetRoadGenerator()?.GetConfig();
        if (config != null)
        {
            INTERSECTION_DETECTION_DISTANCE = config.IntersectionDetectionDistance;
            STOP_LINE_BUFFER = config.StopLineBuffer;
            ROUNDABOUT_SPEED_LIMIT = config.RoundaboutSpeedLimit;
        }

        UpdatePositionAndRotation();
    }

    void Update()
    {
        // IntersectionManager isn't serialized, so it doesn't survive a domain
        // reload in play mode; stand still instead of throwing every frame.
        if (simulationManager == null || currentLane == null ||
            simulationManager.IntersectionManager == null)
            return;

        float deltaTime = Time.deltaTime;

        // Computed once and shared by the speed controller and the emergency stop check below.
        float gapToCarAhead;
        CarController carAhead = FindCarAhead(out gapToCarAhead);

        float targetSpeed = CalculateTargetSpeed(carAhead, gapToCarAhead);

        if (currentSpeed < targetSpeed)
        {
            currentSpeed = Mathf.Min(currentSpeed + MaxAcceleration * deltaTime, targetSpeed);
        }
        else if (currentSpeed > targetSpeed)
        {
            // Proportional brake: gentle for small corrections, full MaxBraking for genuine stops.
            float brakeRate = Mathf.Min(MaxBraking, (currentSpeed - targetSpeed) * 8f);
            currentSpeed = Mathf.Max(currentSpeed - brakeRate * deltaTime, targetSpeed);
        }

        float distanceDelta = currentSpeed * deltaTime;
        float newDistanceAlongPath = distanceAlongPath + distanceDelta;

        // Last-resort emergency brake; normal spacing is already handled by
        // CalculateTargetSpeed. Only fires while actually closing on the leader -
        // contact is impossible against a faster car, and slamming to zero behind
        // one that is pulling away (e.g. just merged ahead of us) is pure jolt.
        if (carAhead != null && currentSpeed > carAhead.CurrentSpeed)
        {
            float predictedGap = gapToCarAhead - distanceDelta
                                 - (CarLength + carAhead.CarLength) * 0.5f;
            if (predictedGap < MinimumCarBuffer * 0.3f)
            {
                BrakeDiagnostics.LogEmergencyStop(this, carAhead, gapToCarAhead);
                newDistanceAlongPath = distanceAlongPath;
                currentSpeed = 0f;
            }
        }

        distanceAlongPath = newDistanceAlongPath;

        if (currentPathType == PathType.Lane)
        {
            HandleLaneMovement();
        }
        else // PathType.TurnCurve
        {
            HandleIntersectionMovement();
        }

        UpdatePositionAndRotation();
    }

    /// <summary>
    /// Target speed is the min of independent constraints (cruise, car ahead,
    /// intersection); each constraint is smooth, which keeps braking continuous
    /// instead of causing sudden stops.
    /// </summary>
    private float CalculateTargetSpeed(CarController carAhead, float gapToCarAhead)
    {
        float limit = TargetSpeed;

        if (carAhead != null)
        {
            limit = Mathf.Min(limit, SafeFollowingSpeed(carAhead, gapToCarAhead));
        }

        limit = Mathf.Min(limit, IntersectionSpeedLimit());

        if (currentPathType == PathType.TurnCurve && currentTurnCurve is RoundaboutTurnCurve)
        {
            limit = Mathf.Min(limit, ROUNDABOUT_SPEED_LIMIT);
        }

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

        if (bumperGap < MinimumCarBuffer * 0.3f && currentSpeed >= carAhead.CurrentSpeed)
        {
            return 0f; // dangerously close and closing - stop immediately
        }

        float freeGap = Mathf.Max(0f, bumperGap - MinimumCarBuffer);
        float leaderSpeed = carAhead.CurrentSpeed;
        float headway = ComfortableBraking * FollowHeadway;

        return Mathf.Max(0f, -headway + Mathf.Sqrt(headway * headway +
                         leaderSpeed * leaderSpeed + 2f * ComfortableBraking * freeGap));
    }

    /// <summary>
    /// Speed limit from traffic lights, right-of-way/conflict gates, and the
    /// don't-block-the-box rule for the intersection at the end of this lane.
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
            return 0f;
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

        // Roundabout approach: decelerate toward ring speed at the yield line even
        // when entry is clear, so a granted car merges at circulating speed. The
        // decel budget is roughly half of comfortable braking: a grant can be
        // revoked mid-approach (circulating traffic braking invalidates the
        // reserved slot), and the remaining headroom is what still lets the car
        // stop at the line instead of arriving hot with nowhere to go.
        if (intersection.IsRoundabout)
        {
            float approachLimit = Mathf.Sqrt(ROUNDABOUT_SPEED_LIMIT * ROUNDABOUT_SPEED_LIMIT
                                  + 2f * (ComfortableBraking * 0.55f) * Mathf.Max(0f, distanceToStopLine - STOP_LINE_BUFFER));
            limit = Mathf.Min(limit, approachLimit);
        }

        // Right-of-way / conflict / downstream-space gate (applies on green too)
        if (currentState == CarState.Approaching && intendedTurn != null && targetIntersection != null)
        {
            if (!CanProceedIntoIntersection())
            {
                // Harmonic blend of MaxAcceleration and ComfortableBraking, not the pure
                // braking curve: a queued car restarting near the line would otherwise
                // accelerate past a safe arrival speed and trip the escape hatch below,
                // bypassing the yield gate.
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

        // Escape hatch: if the constraint appears inside our stopping distance, enter
        // anyway - an instant stop is worse than parking on the curve. Gated on speed so
        // a stopped/queued car can't use it to bypass yield rules the moment light turns green.
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

        // Roundabout approaches all decelerate to ring speed and yield only to
        // circulating traffic (handled in the manager's entry check).
        if (targetIntersection.IsRoundabout)
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
    /// Turn-type right-of-way (right-hand traffic): straight beats right beats left.
    /// A yielding movement gives way to a conflicting higher-priority car within
    /// INTERSECTION_DETECTION_DISTANCE (any speed) or arriving within YIELD_TIME_HORIZON.
    /// OnLane cars with no chosen turn yet are judged by their worst-case plausible
    /// movement; cars held on a red phase are ignored so we don't stall on our own green.
    /// </summary>
    private bool MustYieldByTurnPriority()
    {
        if (intendedTurn == null || targetIntersection == null)
            return false;

        // Turn-type priority is meaningless on a ring; circulating traffic wins instead.
        if (targetIntersection.IsRoundabout)
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

            float theirDistToLine = car.currentLane.Length - car.distanceAlongPath;
            float theirTimeToArrival = theirDistToLine / Mathf.Max(car.currentSpeed, 0.1f);
            if (theirDistToLine > INTERSECTION_DETECTION_DISTANCE && theirTimeToArrival > YIELD_TIME_HORIZON)
                continue; // genuinely far and not arriving soon - a large enough gap

            // A car stopped short of the line is queued behind its own head and can't be
            // next to enter; yielding to it anyway can deadlock two yielding turns that
            // each wait on a follower that never arrives.
            if (car.currentSpeed < 0.5f && theirDistToLine > 3f)
                continue;

            // OnLane car has no chosen turn yet; assume its worst-case plausible movement.
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
    /// True if <paramref name="car"/> has, or could plausibly have, a movement that
    /// outranks our priority and crosses our intended turn. OnLane cars have no turn
    /// yet, so every turn their lane offers is considered and the worst case wins.
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

        // A refused commitment (forced stop) is retried while yellow lasts: if the
        // conflicting crosser clears, running the light beats the impossible stop we were left with.
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
                else if (intendedTurn is RoundaboutTurnCurve approachRing &&
                         car.currentPathType == PathType.TurnCurve &&
                         car.currentTurnCurve is RoundaboutTurnCurve aheadRing &&
                         approachRing.SharesRingWith(aheadRing))
                {
                    // Ring traffic ahead of our future merge point, seen while still
                    // on the feeder lane so the constraint is already active - not
                    // discovered - at curve entry.
                    gap = RingFollowGap(approachRing, aheadRing, car);
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
                else if (currentTurnCurve is RoundaboutTurnCurve myRing &&
                         car.currentPathType == PathType.TurnCurve &&
                         car.currentTurnCurve is RoundaboutTurnCurve otherRing &&
                         myRing.SharesRingWith(otherRing))
                {
                    // Leader is circulating the same roundabout on a different turn path
                    gap = RingFollowGap(myRing, otherRing, car);
                }
                else if (currentTurnCurve is RoundaboutTurnCurve ourRing &&
                         car.currentPathType == PathType.Lane && car.currentLane != null &&
                         car.intendedTurn is RoundaboutTurnCurve mergerRing &&
                         ourRing.SharesRingWith(mergerRing))
                {
                    // Leader is approaching this roundabout on a feeder lane; project
                    // it into ring coordinates so a granted merger fades into view
                    // instead of popping up the frame it enters the curve.
                    gap = RingFollowGap(ourRing, mergerRing, car);
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
    /// Along-path gap to a car converging on, circulating, or leaving the same
    /// roundabout ring, measured through each car's progress toward and around
    /// the ring. Either car may still be on its feeder lane (its remaining lane
    /// distance then counts toward its entry). A leader already on the ring is followed
    /// by ring arc distance, with exit progress added so it fades out gradually.
    /// A leader that has not merged yet - however close - is followed as a
    /// virtual leader at its merge point, padded by how far short of merging it
    /// will still be when we pass; the padding shrinks smoothly to zero as our
    /// arrivals converge and equals the real gap the moment it merges, so the
    /// constraint fades in instead of flipping on at close range. Cars stopped
    /// short of their merge are yielding to the ring and are never followed -
    /// treating them as leaders freezes both cars face-to-face at the merge
    /// point. MaxValue when the other car is not a threat on our remaining path.
    /// </summary>
    private float RingFollowGap(RoundaboutTurnCurve myRing, RoundaboutTurnCurve otherRing, CarController other)
    {
        // Our own progress toward/around the ring; we may still be on the feeder
        // lane ourselves (myRing is then our intended turn, not our current path).
        float myEntryRemaining, myRemainingRing, myPos;
        if (currentPathType == PathType.TurnCurve)
        {
            if (myRing.ExitConnectorProgress(distanceAlongPath) > 0f)
                return float.MaxValue; // we've left the ring; destination-lane following covers the rest

            myEntryRemaining = myRing.EntryConnectorRemaining(distanceAlongPath);
            myRemainingRing = myRing.RemainingRingTravel(distanceAlongPath);
            myPos = myRing.RingPositionOf(distanceAlongPath);
        }
        else
        {
            myEntryRemaining = (currentLane.Length - distanceAlongPath) + myRing.EntryConnectorLength;
            myRemainingRing = myRing.RingArcLength;
            myPos = myRing.EntryRingDistance;
        }

        bool otherOnCurve = other.currentPathType == PathType.TurnCurve;
        float theirEntryRemaining, theirExitProgress, theirPos;
        if (otherOnCurve)
        {
            theirEntryRemaining = otherRing.EntryConnectorRemaining(other.distanceAlongPath);
            theirExitProgress = otherRing.ExitConnectorProgress(other.distanceAlongPath);
            theirPos = otherRing.RingPositionOf(other.distanceAlongPath);
        }
        else
        {
            // Still on a feeder lane; a slow car short of the line is being held
            // at the yield gate (a full entry connector away from the ring), but
            // one past the line (Crossing) enters regardless of speed.
            if (other.currentSpeed < 2f && other.currentState != CarState.Crossing)
                return float.MaxValue;

            theirEntryRemaining = (other.currentLane.Length - other.distanceAlongPath)
                                  + otherRing.EntryConnectorLength;
            theirExitProgress = 0f;
            theirPos = otherRing.EntryRingDistance;
        }

        float ringGap = RoundaboutTurnCurve.RingMod(theirPos - myPos, myRing.RingCircumference);

        if (ringGap > myRemainingRing + 2f)
        {
            // Behind us on the ring. Normally not our problem - except before our
            // own merge: circulating traffic reaching our entry point first has
            // right of way there, so follow its projected position past our entry.
            if (myEntryRemaining > 0f && otherOnCurve &&
                theirEntryRemaining <= 0f && theirExitProgress <= 0f)
            {
                float behindDist = myRing.RingCircumference - ringGap;
                if (otherRing.RemainingRingTravel(other.distanceAlongPath) + 1f >= behindDist)
                {
                    float theirTime = behindDist / Mathf.Max(other.currentSpeed, 0.5f);
                    float myTime = myEntryRemaining / Mathf.Max(currentSpeed, 2f);
                    if (theirTime <= myTime)
                        return myEntryRemaining + (myTime - theirTime) * other.currentSpeed;
                }
            }
            return float.MaxValue;
        }

        if (otherOnCurve && theirEntryRemaining <= 0f)
        {
            // Circulating or leaving the ring ahead of us
            return myEntryRemaining + ringGap + theirExitProgress;
        }

        // Converging on a merge point ahead of us but not on the ring yet.
        if (other.currentSpeed < 0.5f && theirEntryRemaining > 1f)
            return float.MaxValue; // stopped short of its merge: yielding, not converging

        float myDistToTheirMerge = myEntryRemaining + ringGap;
        if (myDistToTheirMerge <= 0f)
            return float.MaxValue;

        float myTimeToTheirMerge = myDistToTheirMerge / Mathf.Max(currentSpeed, 2f);
        float theirShortfallAtMyPass = Mathf.Max(0f,
            theirEntryRemaining - other.currentSpeed * myTimeToTheirMerge);
        return myDistToTheirMerge + theirShortfallAtMyPass;
    }

    private void HandleLaneMovement()
    {
        if (currentPathType != PathType.Lane)
        {
            Debug.LogError($"Car {CarID}: HandleLaneMovement called but pathType is {currentPathType}!");
            return;
        }

        float distanceToEnd = currentLane.Length - distanceAlongPath;

        // Transition: OnLane -> Approaching (skip for trivial intersections with only 2 roads)
        if (distanceToEnd < INTERSECTION_DETECTION_DISTANCE && currentState == CarState.OnLane)
        {
            if (currentLane.EndNode == null)
            {
                Debug.LogError($"Car {CarID}: Lane {currentLane.ID} has no EndNode!");
                return;
            }

            // Trivial intersections (only 2 roads) are straight-through continuations
            // with no collision risk, so intersection logic is skipped for them.
            if (!currentLane.EndNode.IsTrivialIntersection())
            {
                currentState = CarState.Approaching;
                targetIntersection = currentLane.EndNode;

                // Selected once; other cars' yield checks read IntendedTurn from here on.
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

                simulationManager.IntersectionManager.UpdateCarWaitingAt(this, currentLane, targetIntersection);
            }
        }

        // Transition: Approaching -> WaitingToEnter or Crossing
        if (distanceToEnd < STOP_LINE_BUFFER && currentState == CarState.Approaching)
        {
            if (CanProceedIntoIntersection())
            {
                currentState = CarState.Crossing;
                simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
                simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
                SetLightDecision(LightDecision.None);
            }
            else
            {
                currentState = CarState.WaitingToEnter;
                SetLightDecision(LightDecision.None);
            }
        }

        if (currentState == CarState.WaitingToEnter)
        {
            if (CanProceedIntoIntersection())
            {
                currentState = CarState.Crossing;
                simulationManager.IntersectionManager.RegisterEntry(this, targetIntersection, intendedTurn);
                simulationManager.IntersectionManager.RemoveCarWaitingAt(this, currentLane, targetIntersection);
                SetLightDecision(LightDecision.None);
            }
            else
            {
                BrakeDiagnostics.LogLineStop(this, targetIntersection);
                currentSpeed = 0f;

                // Resolve overlap gradually (rate-limited) instead of teleporting backwards.
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
                    distanceAlongPath = Mathf.Min(distanceAlongPath, currentLane.Length - 0.1f);
                }

                return; // skip the lane-end transition check below while stuck waiting
            }
        }

        // Transition to turn curve when at lane end
        if (distanceAlongPath >= currentLane.Length)
        {
            bool isTrivial = currentLane.EndNode != null && currentLane.EndNode.IsTrivialIntersection();

            if (isTrivial && currentState == CarState.OnLane)
            {
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

    private void HandleIntersectionMovement()
    {
        if (currentPathType != PathType.TurnCurve)
        {
            Debug.LogError($"Car {CarID}: HandleIntersectionMovement called but pathType is {currentPathType}!");
            return;
        }

        if (currentState != CarState.Crossing)
        {
            Debug.LogWarning($"Car {CarID}: On turn curve but state is {currentState}, correcting to Crossing");
            currentState = CarState.Crossing;
        }

        // Exit when turn complete
        if (distanceAlongPath >= currentTurnCurve.ArcLength)
        {
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
    /// Must be called with currentState already Crossing.
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

        // The approach limit targets ring speed at the yield line, but brake-tracking
        // lag can leave a few extra m/s; entering hot overtakes circulating traffic
        // that the merge projection assumed would stay ahead.
        if (currentTurnCurve is RoundaboutTurnCurve)
        {
            currentSpeed = Mathf.Min(currentSpeed, ROUNDABOUT_SPEED_LIMIT);
        }

        // State remains Crossing - don't change it!
    }

    private void TransitionToNextLane()
    {
        Lane destinationLane = currentTurnCurve.ToLane;
        float overshoot = Mathf.Max(0f, distanceAlongPath - currentTurnCurve.ArcLength);
        float safeStartDistance = FindSafeStartPositionOnLane(destinationLane);

        // Release any light commitment before intersection state is cleared.
        SetLightDecision(LightDecision.None);

        currentLane = destinationLane;
        currentPathType = PathType.Lane;
        distanceAlongPath = Mathf.Max(safeStartDistance, Mathf.Min(overshoot, destinationLane.Length));

        currentState = CarState.OnLane;
        targetIntersection = null;
        intendedTurn = null;
        currentTurnCurve = null;
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
                // Packed at the lane start: match the queue tail speed so we don't drive into it.
                currentSpeed = Mathf.Min(currentSpeed, closestCar.CurrentSpeed);
            }
        }

        // Always starts at the lane mouth; safety comes from the speed clamp above, not a distance offset.
        return 0f;
    }

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

        transform.position = position;

        if (direction.sqrMagnitude > 0.001f)
        {
            transform.rotation = Quaternion.LookRotation(direction, Vector3.up);
        }
    }

    public Vector3 GetPosition()
    {
        return transform.position;
    }
}
