# Traffic Simulation Technical Documentation

## 1. System Overview

This project implements a 3D traffic simulation in Unity, featuring procedural road network generation, agent-based vehicle dynamics, and autonomous intersection management. The system operates on a right-hand traffic model and includes bridge generation, dynamic traffic signal allocation, and telemetry export capabilities.

### Core Capabilities
*   **Procedural Generation**: Grid-based network with organic jitter, bridges, and randomized topology.
*   **Traffic Simulation**: Kinematic vehicle movement with physically consistent braking, safe-following, and queue management.
*   **Intersection Logic**: "Priority-to-the-right" rules, turn-conflict arbitration, yield and box rules, deadlock detection, and traffic light phasing.
*   **Data Export**: CSV telemetry export plus a Python braking-anomaly analyzer.

---

## 2. Architecture & Components

### Control Systems
*   **`SimulationManager.cs`**: Central orchestrator. Generates the network (rebuilding the `IntersectionManager` with it), spawns cars, and drives update loops. Clearing cars resets all per-car intersection state.
*   **`IntersectionManager.cs`**: Arbitrates intersection entry (lights, queue order, turn conflicts, priority), manages run-light commitments, tracks lane queues, and resolves deadlocks at unsignalized nodes.
*   **`StopLightController.cs`**: Handles state machines for signalized intersections (Green/Yellow/Red phases); pairs roughly opposite roads into shared phases.

### Generation Pipeline
*   **`RoadNetworkGenerator.cs`**: Executes the multi-pass procedural generation pipeline.
*   **`RoadGenerationConfig.cs`**: `ScriptableObject` containing all generation parameters.
*   **`RoadNetworkData.cs`**: Defines core data structures (`Node`, `Road`, `Lane`, `TurnCurve`, `BridgeInfo`).

### Agents & Visualization
*   **`CarController.cs`**: Individual vehicle logic — target-speed computation as the minimum of independent constraints, state machine (OnLane, Approaching, WaitingToEnter, Crossing), and path following.
*   **`RoadNetworkVisualizer.cs`**: Renders debug gizmos for the network graph, turn curves, and traffic states.
*   **`CSVLogger.cs`**: High-frequency position logging to disk.
*   **`SpeedVerificationTool.cs`**: Optional debug component for verifying lane arc lengths and vehicle speeds (bridges).

---

## 3. Installation & Setup

### Requirements
*   Unity Editor **6000.3.19f1** (Unity 6.3 LTS) — the project is pinned to this version; do not open with other editors unless you intend a one-way upgrade.
*   Universal Render Pipeline (URP 17.3, included via the package manifest).

### Opening the Project
1.  Clone the repository.
2.  In Unity Hub, choose **Add > Add project from disk**, select the cloned folder, and open it (the repository is a complete Unity project).
3.  Open `Assets/Scenes/SampleScene.unity` and press Play.

**Recommended editor setting**: *Preferences > General > Script Changes While Playing* → **Recompile After Finished Playing**. The road graph holds non-serializable object references and does not survive a mid-play domain reload.

### Scene Wiring Reference
Only needed when building a new scene — `SampleScene` ships fully configured:
*   A GameObject with `SimulationManager`, `RoadNetworkGenerator`, `RoadNetworkVisualizer`, and `CSVLogger` components, references assigned in the Inspector (the `IntersectionManager` is a plain class created at generation time, not a component).
*   A `RoadGenerationConfig` asset (**Create > Traffic Simulation > Road Generation Config**) assigned to the generator.
*   A car prefab (e.g. a cube scaled 2 x 1 x 4) with `CarController`, assigned to the `SimulationManager`'s "Car Prefab" field.

---

## 4. Configuration Parameters

### Road Generation (`RoadGenerationConfig`, asset: `Assets/DefaultRoadConfig.asset`)

| Parameter | Type | Description | Default |
| :--- | :--- | :--- | :--- |
| **Grid Width / Height** | `int` | Dimensions of the node grid. | 4 x 3 |
| **Segment Length (Min/Max)** | `float` | Random range for road lengths. | 30-50 |
| **Intersection Jitter** | `float` | Random node displacement in world units (0 = perfect grid). | 3 |
| **Lane Width** | `float` | Width per lane. | 3.5 |
| **Stop Line Distance** | `float` | Base setback from intersection center; scaled up automatically at sharp angles and clamped so short roads never invert. | 4 |
| **Turn Radius** | `float` | Control point distance for Bézier turn curves. | 5 |
| **Node / Road Removal %** | `float` | Percentage of non-essential nodes/roads removed for organic topology. | 15% / 10% |
| **Bridge Settings** | — | Enable flag, height (5), min ramp length (8), min distance from intersection (12), min crossing angle (35°). | Various |
| **Diagonal Connections** | `float` | % of shortcut roads to generate (creates bridge candidates). | 15% |
| **Random Seed** | `int` | Fixed seed for reproducible generation (0 = random). | 0 |
| **Traffic Light %** | `float` | % of eligible intersections (3+ roads) to signalize. | 100% |
| **Intersection Detection Distance** | `float` | Distance from lane end where cars begin intersection logic. | 25 |
| **Stop Line Buffer** | `float` | Distance before the stop line where cars hold. | 0.5 |
| **Deadlock Timeout** | `float` | Wait time before a stuck car is forced through (unsignalized intersections only). | 5.0s |

### Vehicle Parameters (`CarController`, on the car prefab)

| Parameter | Type | Description | Default |
| :--- | :--- | :--- | :--- |
| **Target Speed** | `float` | Cruising speed (units/sec). | 20 |
| **Max Acceleration** | `float` | Rate of speed increase (units/sec²). | 5 |
| **Max Braking** | `float` | Maximum deceleration (units/sec²). Planned braking uses 70% of this; the headroom absorbs surprises. | 10 |
| **Car Length** | `float` | Physical length used in gap calculations. | 4 |
| **Minimum Car Buffer** | `float` | Bumper-to-bumper gap maintained between stopped cars. | 1 |
| **Follow Headway** | `float` | Reaction-time headway behind moving cars (seconds); sets following distance at speed. | 0.6 |

---

## 5. Procedural Generation Pipeline

The `RoadNetworkGenerator` executes the following passes sequentially:

1.  **Grid Creation**: Generates `Node` objects on the XZ plane. Applies jitter if configured.
2.  **Topology Building**:
    *   Connects adjacent nodes; wrap-around and diagonal candidates are rejected if too long or if they pass within `MinDistanceFromIntersection` of an unrelated node.
    *   Adds diagonal shortcuts based on `DiagonalConnectionPercentage`.
    *   Validates connections: rejects angles < 20° and crossings that cannot be bridged.
3.  **Pruning**:
    *   Removes random nodes/roads based on configuration.
    *   **Connectivity Check**: Uses BFS to ensure the graph remains fully connected.
    *   **Dead-end Removal**: Iteratively removes nodes with degree < 2.
    *   **Sharp Angle Cleanup**: Removes roads creating unresolvable acute angles (< 20°).
4.  **Bridge Generation**:
    *   Identifies intersecting road segments after all removals.
    *   Validates crossing constraints (angle > 35°, ramp room, distance from nodes) — tried from both roads' perspectives; whichever can carry the bridge gets it. Crossings neither road can bridge are logged as warnings.
    *   Overlapping bridge intervals on the same road are merged into one span (elevations would otherwise stack).
    *   **Elevation**: C1-smooth cosine profile `h(p) = 0.5 (1 − cos 2πp) · H` — zero slope where ramps meet flat road.
5.  **Geometry Calculation**:
    *   **Stop Distances**: Scaled from the sharpest angle at each node: `d = (w/2) / tan(θ/2) · 1.15`, capped at the requirement for the sharpest permitted angle (20°); the two setbacks of a road are additionally scaled down to never exceed the road length.
    *   **Lane Objects**: Creates forward/backward lanes with lateral offsets.
    *   **Length Calculation**: Numerical integration for accurate 3D arc length on bridge lanes.
6.  **Turn Precomputation**:
    *   Generates quadratic Bézier curves for all valid lane connectors (U-turns excluded; turns sharper than 140° excluded, except a lane's least-sharp turn is always kept so no lane is left without an exit).
    *   Each curve builds an arc-length table so cars traverse it at uniform world speed.
    *   **Conflict Detection**: Pre-computes spatial overlaps between turn curves. Turns leaving from the same lane are never conflicts (queue order already serializes them); turns merging into the same lane are.

---

## 6. Simulation Logic

### 6.1 Vehicle State Machine
1.  **OnLane**: Standard traversal. Follows lane geometry/elevation.
2.  **Approaching**: Active within 25 units of a non-trivial intersection. Pre-selects a turn and enqueues on its lane.
3.  **WaitingToEnter**: Holding at the stop line; re-checks entry permission every frame.
4.  **Crossing**: Traversing the intersection via `TurnCurve`. Trivial (2-road) nodes are crossed without arbitration.

### 6.2 Speed Control
Each frame the target speed is the **minimum of independent constraints**, each smooth and physically trackable:

*   **Cruise**: `TargetSpeed`, reduced to 85-95% while approaching intersections.
*   **Safe following** (Krauss-style): fastest speed from which the car can settle behind the leader with `MinimumCarBuffer` spacing using comfortable braking (70% of max), including a `FollowHeadway` reaction-time term. The leader is found **across path boundaries** — same lane, turn curves leaving the lane, the lane beyond the car's own next turn, and (while crossing) the destination lane — so queues are seen through intersections and braking starts early.
*   **Stop-line braking**: `v = sqrt(2 a d)` toward the stop line when facing a red light, a yellow the car decided to stop for, a denied entry, or a queue.

Speed adjusts toward the target with acceleration capped at `MaxAcceleration` and deceleration proportional to the speed error (up to `MaxBraking`). A last-resort emergency stop prevents ever moving into the car ahead.

### 6.3 Traffic Lights
*   **Phasing**: Roads ~180° apart share a green phase; timing is fixed: Green (10s) → Yellow (3s) → all-red (2s).
*   **Yellow-light decision**: Made **once per approach** and latched. A car stops unless that would take more than ~85% of maximum braking; otherwise it requests a *run-light commitment* from the `IntersectionManager`, which grants it only if no conflicting car is crossing or already committed — so two conflicting approaches can never both run the same yellow. Committed cars pass the light gate and clear the intersection instead of being stopped dead at the line; refused cars brake hard (95% of max) and retry the commitment while the yellow lasts.
*   A committed car that ends up stopped anyway (e.g. behind a queue) downgrades to a normal stop and waits for green.

### 6.4 Intersection Entry Rules
Entry requires **all** of:
1.  **Light gate**: green, or a valid run-light commitment.
2.  **Queue order**: first in the lane's queue.
3.  **No path conflict** with active crossings or same-frame grants (using the precomputed turn-conflict map).
4.  **Priority-to-the-right** (unsignalized only): yields to a vehicle waiting at the stop line on the right — unless the requester physically can no longer stop, in which case it proceeds and the stationary car waits one more turn.
5.  **Don't block the box**: the destination lane must have room for the car to fully exit (cars still crossing toward that lane reserve space).
6.  **Yield rule**: entry is deferred while a conflicting car is bearing down on the intersection too fast to stop (the permissive-left pattern).

Rules 5-6 are waived for cars that physically cannot stop anymore (they enter and, if needed, wait on the turn curve) and for the deadlock breaker.

### 6.5 Deadlock Resolution (unsignalized intersections only)
Queue heads stuck at or crawling near the stop line are timed from the moment they actually stop. If two or more approaches are stuck past `DeadlockTimeout` (or one approach past twice that), the longest-waiting car is forced through, bypassing the arbitration rules. A breaker that fails to enter within the timeout expires and another car is chosen. Signalized intersections are never forced — the light cycle clears them.

---

## 7. Data Structures

### `Node`
Graph vertex: `Vector3 Position`, `List<Road> ConnectedRoads`. Nodes with exactly 2 roads are "trivial" pass-throughs.

### `Road`
Edge between two nodes: `ForwardLane`, `BackwardLane`, shared `List<BridgeInfo>`.

### `Lane`
One-directional traffic conduit: 3D arc `Length` (bridge-aware), `GetPositionAtDistance()`, `AvailableTurns`.

### `TurnCurve`
Quadratic Bézier lane connector with a cumulative arc-length table (`GetTForArcDistance`) for uniform-speed traversal.

### `BridgeInfo`
Elevation span on a road: `StartDistance`/`EndDistance`, `GetElevationAt()` (cosine profile).

---

## 8. Telemetry & Analysis

### CSV Export
The `CSVLogger` component exports vehicle positions at 10 Hz (configurable, drift-free cadence).
**Format**: `vehicle_id,positionX,positionY,positionZ,time`
**Location**: `PythonAnalysis/vehicle_positions.csv` (recreated each run; gitignored).

### Braking Analysis
`PythonAnalysis/brake_analysis.py` (Python 3) reconstructs per-vehicle speed and acceleration from the CSV and reports events exceeding the cars' physical braking capability (instant stops, teleports), filtering editor frame hitches:

```
python PythonAnalysis/brake_analysis.py PythonAnalysis/vehicle_positions.csv
```

Healthy runs show hard-braking events (decel > 25 u/s²) in the single digits per minute; extreme events (> 60) near zero.

---

## 9. Troubleshooting

| Issue | Probable Cause | Resolution |
| :--- | :--- | :--- |
| **Cars frozen + NullReferenceException spam** | Domain reload during play (script recompile) wiped runtime state. | Set *Script Changes While Playing* to "Recompile After Finished Playing"; restart play mode. |
| **"Serialization depth limit 10 exceeded" in console** | Known cosmetic issue: `Node`↔`Road` reference cycle in serialized classes. | Ignore; harmless. |
| **Gridlock** | Downstream jams looping across intersections. | Deadlock breaker resolves single nodes; reduce car count for heavy loops. |
| **Overlapping lanes / broken intersections** | Very high jitter relative to segment length. | Reduce `IntersectionJitter` or increase segment lengths; check console for generation warnings. |
| **Missing visuals** | Gizmos disabled. | Enable Gizmos in Scene/Game view; ensure `RoadNetworkVisualizer` is active. |

---

## 10. Known Issues & Limitations

### Identified Defects
*   **Topology Generation**: High jitter values can still produce overlapping roads and intersection generation errors (warnings are logged when safeguards fire).
*   **Right of Way**: Priority is sometimes overridden — deadlock breaking, run-light commitments, and the cannot-stop physics exemptions supersede normal right-of-way order by design, which can read as rule violations.
*   **Turn Geometry**: Turn arcs through an intersection can overlap; sharp turns may bulge outside the intersection footprint (the sharp-turn Bézier control point is allowed up to 3× the endpoint gap).

### Simulation Constraints
*   **Interaction**: Editor-only (Scene view + Inspector); no runtime UI.
*   **Perception Horizon**: Vehicles see one path segment ahead (through the next intersection), no further.
*   **Performance**: O(N²) car-to-car checks; optimized for <100 vehicles.
*   **Turn Restrictions**: U-turns are disabled to prevent flow loops.

### Signal Logic Limitations
*   **Fixed Timing**: Signal phases are time-based; no sensor actuation.
*   **Permissive Left Only**: Left turns on green yield via the runtime yield rule; there are no protected-left phases.
*   **Phase Optimization**: Phase assignment is heuristic (opposite-road pairing), not flow-optimized.

---

## 11. Future Improvements

*   **Signal Control**: Sensor-actuated traffic lights; protected left-turn phases.
*   **Traffic Rules**: Stop signs, yield signs, roundabout topology.
*   **Agents**: Pedestrian logic and emergency vehicle preemption.
*   **Navigation**: A* pathfinding with destination logic; dynamic lane changing.
