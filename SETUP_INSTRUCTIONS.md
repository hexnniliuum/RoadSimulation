# Traffic Simulation Technical Documentation

## 1. System Overview

This project implements a 3D traffic simulation in Unity, featuring procedural road network generation, agent-based vehicle dynamics, and autonomous intersection management. The system operates on a right-hand traffic model and includes bridge generation, dynamic traffic signal allocation, and telemetry export capabilities.

### Core Capabilities
*   **Procedural Generation**: Grid-based network with organic jitter, bridges, and randomized topology.
*   **Traffic Simulation**: Physics-based vehicle movement, collision avoidance, and queue management.
*   **Intersection Logic**: "Priority-to-the-right" rules, deadlock detection, and traffic light phasing.
*   **Data Export**: CSV telemetry export for external analysis.

---

## 2. Architecture & Components

### Control Systems
*   **`SimulationManager.cs`**: Central orchestrator. Initializes subsystems and manages the update loop.
*   **`IntersectionManager.cs`**: Manages intersection states, enforces priority rules, resolves deadlocks, and tracks lane queues.
*   **`StopLightController.cs`**: Handles state machines for signalized intersections (Green/Yellow/Red phases).

### Generation Pipeline
*   **`RoadNetworkGenerator.cs`**: Executes the multi-pass procedural generation pipeline.
*   **`RoadGenerationConfig.cs`**: `ScriptableObject` containing all generation parameters.
*   **`RoadNetworkData.cs`**: Defines core data structures (`Node`, `Road`, `Lane`, `TurnCurve`, `BridgeInfo`).

### Agents & Visualization
*   **`CarController.cs`**: Handles individual vehicle physics, pathfinding, and state (OnLane, Approaching, Waiting, Crossing).
*   **`RoadNetworkVisualizer.cs`**: Renders debug gizmos for the network graph, turn curves, and traffic states.
*   **`CSVLogger.cs`**: Handles high-frequency position logging to disk.

---

## 3. Installation & Setup

### Repository Initialization
1.  Initialize a default 3D Unity project.
2.  Pull the repository content directly into the project root.

### Requirements
*   Unity Editor 2021.3+
*   Standard 3D Render Pipeline (or URP with material upgrades)

### Scene Configuration
1.  **Script Placement**: Ensure all `.cs` files are located in `Assets/Scripts`.
2.  **Configuration Asset**: Create a `RoadGenerationConfig` asset via **Create > Traffic Simulation > Road Generation Config**.
3.  **Manager Setup**:
    *   Create an empty GameObject named `SimulationManager`.
    *   Attach `SimulationManager`, `RoadNetworkGenerator`, `RoadNetworkVisualizer`, `IntersectionManager`, and `CSVLogger` components.
    *   Assign references in the Inspector (drag `SimulationManager` object to self-reference fields).
    *   Assign the `RoadGenerationConfig` asset to the Generator.
4.  **Vehicle Prefab**:
    *   Create a Cube (Scale: X=2, Y=1, Z=4).
    *   Attach `CarController`.
    *   Create a Prefab and assign it to the `SimulationManager`'s "Car Prefab" field.

---

## 4. Configuration Parameters

### Road Generation (`RoadGenerationConfig`)

| Parameter | Type | Description | Default |
| :--- | :--- | :--- | :--- |
| **Grid Dimensions** | `int` | Width/Height of the node grid. | 4x4 |
| **Segment Length** | `float` | Random range for road lengths. | 50-100 |
| **Intersection Jitter** | `float` | Random node displacement (0.0-1.0). High values (>0.5) on small grids may cause overlap. | 0.3 |
| **Lane Width** | `float` | Width per lane. | 4.0 |
| **Turn Radius** | `float` | Control point distance for Bézier curves. | 8.0 |
| **Stop Line Distance** | `float` | Base distance from intersection center. Automatically scaled by intersection angle. | 5.0 |
| **Removal Percentages** | `float` | % of Nodes/Roads to remove for organic topology. | 0-30% |
| **Diagonal Connections** | `float` | % of shortcut roads to generate (creates bridge candidates). | 15% |
| **Bridge Constraints** | `float` | Min angle, min distance from node, ramp length. | Various |
| **Traffic Light %** | `float` | % of eligible intersections (3+ roads) to signalize. | 50% |
| **Deadlock Timeout** | `float` | Time before forcing entry during gridlock. | 3.0s |

### Vehicle Physics (`CarController`)

| Parameter | Type | Description | Default |
| :--- | :--- | :--- | :--- |
| **Target Speed** | `float` | Cruising speed (units/sec). | 20 |
| **Max Acceleration** | `float` | Rate of speed increase. | 5 |
| **Max Braking** | `float` | Rate of speed decrease. Used for kinematic stopping distance calc. | 10 |
| **Look Ahead** | `float` | Obstacle detection range. | 15 |
| **Safe Follow** | `float` | Minimum gap maintained behind leading vehicle. | 5 |

---

## 5. Procedural Generation Pipeline

The `RoadNetworkGenerator` executes the following passes sequentially:

1.  **Grid Creation**: Generates `Node` objects on XZ plane. Applies jitter if configured.
2.  **Topology Building**:
    *   Connects adjacent nodes.
    *   Adds diagonal shortcuts based on `DiagonalConnectionPercentage`.
    *   Validates connections: Rejects angles < 20° and overlapping segments.
3.  **Pruning**:
    *   Removes random nodes/roads based on configuration.
    *   **Connectivity Check**: Uses BFS to ensure graph remains fully connected.
    *   **Dead-end Removal**: Iteratively removes nodes with degree < 2.
    *   **Sharp Angle Cleanup**: Removes roads creating unresolvable acute angles (< 20°).
4.  **Bridge Generation**:
    *   Identifies intersecting road segments.
    *   Validates crossing constraints (angle > 35°, distance from nodes).
    *   Creates `BridgeInfo` on one road (elevation data), other road remains flat.
    *   **Elevation**: Parabolic curve $h(t) = 4t(1-t)H$.
5.  **Geometry Calculation**:
    *   **Stop Distances**: Calculated dynamically: $d = (w/2) / \tan(\theta/2)$.
    *   **Lane Objects**: Creates forward/backward lanes with offsets.
    *   **Length Calculation**: Uses numerical integration (100 samples) for accurate 3D arc length on bridges.
6.  **Turn Precomputation**:
    *   Generates Quadratic Bézier curves for all valid lane connectors.
    *   **Conflict Detection**: Pre-calculates spatial overlaps between turn curves for intersection safety.

---

## 6. Simulation Logic

### 6.1 Vehicle State Machine
Vehicles operate using the following states:
1.  **OnLane**: Standard traversal. Follows lane geometry/elevation.
2.  **Approaching**: Active within 25 units of intersection. Pre-selects turn direction.
3.  **WaitingToEnter**: Stopped at stop line. Queries `IntersectionManager` for entry permission.
4.  **Crossing**: Traversing the intersection via `TurnCurve`.

### 6.2 Movement & Physics
*   **Kinematic Braking**: Stopping distance calculated as $d = v^2 / 2a$.
*   **Queueing**: Maintains 1.0 unit buffer between vehicles. Implements "creep" behavior to close gaps.
*   **Collision Avoidance**: O(N²) check against vehicles on current path segment.

### 6.3 Intersection Management
The system employs a hierarchy of rules to resolve conflicts:

1.  **Traffic Lights** (If present):
    *   Overrides priority rules.
    *   **Phasing**: Groups non-adjacent lanes into phases.
    *   **Timing**: Green (10s) -> Yellow (3s) -> Red (2s).
2.  **Deadlock Resolution**:
    *   Monitors wait times. If all queue heads wait > `DeadlockTimeout`, the longest-waiting vehicle is forced through.
3.  **Priority-to-the-Right**:
    *   Standard yielding rule for unsignalized intersections.
    *   Uses cross product of approach vectors: `cross.y < -0.1` indicates right-side vehicle.
4.  **Path Conflict**:
    *   Prevents entry if intended turn path physically overlaps with an active crossing vehicle.

---

## 7. Data Structures

### `Node`
Represents a graph vertex.
*   `List<Road> ConnectedRoads`: Adjacency list.
*   `Vector3 Position`: World space coordinates.

### `Road`
Represents an edge between two Nodes.
*   `Lane ForwardLane`: Start -> End.
*   `Lane BackwardLane`: End -> Start.
*   `List<BridgeInfo> Bridges`: Shared elevation data.

### `Lane`
Traffic conduit.
*   `float RoadLength`: 3D arc length.
*   `GetPositionAtDistance(float d)`: Returns 3D coordinate including bridge elevation.

### `BridgeInfo`
Elevation data for crossings.
*   `float StartDist`, `EndDist`: Linear range of the bridge.
*   `GetElevationAt(float d)`: Returns Y-offset based on parabolic curve.

---

## 8. Telemetry & Analysis

### CSV Export
The `CSVLogger` component exports vehicle data at 10Hz (configurable).
**Format**: `vehicle_id, positionX, positionY, positionZ, time`
**Location**: Project root directory (`/vehicle_positions.csv`).

**Note**: This repository generates raw data only. No internal analysis tools are included; must parse the CSV/JSON output externally.

---

## 9. Troubleshooting

| Issue | Probable Cause | Resolution |
| :--- | :--- | :--- |
| **Cars teleporting/Jumping** | Invalid turn curves or network disconnects. | Check console for generation errors; reduce Jitter. |
| **Gridlock** | Deadlock timeout too high or logic failure. | Verify `DeadlockTimeout` is reasonable (3-5s). |
| **Overlapping Lanes** | Sharp angles not pruned. | Ensure `RoadGenerationConfig` has valid Stop Line Distance. |
| **Missing Visuals** | Gizmos disabled or incorrect View. | Enable Gizmos in Scene View; Ensure `RoadNetworkVisualizer` is active. |

---

## 10. Known Issues & Limitations

### Simulation Constraints
*   **Interaction**: System is currently interactions-only via Unity Scene view and Inspector.
*   **Perception Horizon**: Vehicles do not perceive traffic beyond the immediate next intersection.
*   **Performance**: Collision detection is O(N²), optimized for <100 vehicles. Requires spatial partitioning for scale.
*   **Turn Restrictions**: U-turns are explicitly disabled to prevent flow loops.

### Identified Defects
*   **Traffic Priority**: Priority logic for signalized intersections may exhibit suboptimal yielding behavior.
*   **Braking Instability**: Unexpected sudden braking events occur sporadically.
*   **Topology Failure**: High jitter values may cause intersection generation failure or lane overlaps.
*   **Bridge Geometry**: Bridges spanning multiple road segments do not follow a continuous smooth curve.

### Signal Logic Limitations
*   **Fixed Timing**: Signal phases are time-based; no sensor actuation.
*   **Implicit Yielding**: Left-turn-on-green yielding is handled implicitly via conflict detection rather than explicit phase logic.
*   **Phase Optimization**: Traffic light positioning and phase assignment are heuristic and not optimized for specific flow patterns.

---

## 11. Future Improvements

### Roadmap
*   **Signal Control**: Implementation of sensor-actuated traffic lights.
*   **Traffic Rules**: Addition of Stop signs, Yield signs, and Roundabout topology.
*   **Agents**: Pedestrian logic and Emergency vehicle preemption.
*   **Navigation**: A* pathfinding with destination logic; dynamic lane changing.