# Unity Procedural Traffic Simulation

A 3D traffic simulation framework implemented in Unity. This system features procedural road network generation, kinematic vehicle agents with physically consistent braking, and autonomous intersection management with support for signalized and unsignalized traffic control.

## System Overview

The simulation operates on a right-hand traffic model and generates road networks procedurally using a grid-based algorithm with organic jitter. It supports dynamic bridge generation, "priority-to-the-right" intersection rules, deadlock detection, and traffic light phasing.

### Core Capabilities

*   **Procedural Topology**: Automated generation of nodes and edges with configurable jitter, bridges, and randomized connectivity.
*   **Agent Dynamics**: Physics-consistent speed control — safe-following with reaction-time headway, smooth braking curves toward stop lines, and cross-segment lookahead so vehicles see traffic through intersections.
*   **Intersection Logic**: Autonomous conflict resolution using turn-path conflict maps, priority rules, yield-to-committed-traffic and don't-block-the-box rules, deadlock timeouts, and traffic signal state machines with latched stop/go decisions at yellow.
*   **Telemetry**: High-frequency CSV export of vehicle positions, with a bundled Python analysis script for detecting braking anomalies.

## Technical Architecture

### Control Systems
*   **`SimulationManager`**: Central orchestrator for subsystem initialization, car spawning, and update loops.
*   **`IntersectionManager`**: Manages node states, entry arbitration (lights, queues, conflicts, priority), run-light commitments, and deadlock breaking.
*   **`StopLightController`**: Implements state machines for signalized intersections (Green/Yellow/Red phases).

### Generation Pipeline
*   **`RoadNetworkGenerator`**: Executes multi-pass procedural generation (Grid -> Topology -> Pruning -> Bridges -> Geometry -> Turn precomputation).
*   **`RoadNetworkData`**: Defines graph structures (`Node`, `Road`, `Lane`, `TurnCurve`, `BridgeInfo`).

### Agents
*   **`CarController`**: Handles individual vehicle state (`OnLane`, `Approaching`, `WaitingToEnter`, `Crossing`), target-speed computation, and path following.

## Installation

### Prerequisites
*   Unity **6000.3.19f1** (Unity 6.3 LTS) — the project is pinned to this version; opening with a different editor triggers a one-way upgrade.
*   Universal Render Pipeline (URP 17.3).

### Setup
1.  Clone the repository.
2.  In Unity Hub, choose **Add > Add project from disk** and select the cloned folder, then open it.
3.  Open `Assets/Scenes/SampleScene.unity` and press Play.

See [SETUP_INSTRUCTIONS.md](SETUP_INSTRUCTIONS.md) for the full parameter reference and component wiring details.

## Configuration

Configuration is managed via the `RoadGenerationConfig` ScriptableObject (`Assets/DefaultRoadConfig.asset`).

| Parameter | Default | Description |
| :--- | :--- | :--- |
| **Grid Dimensions** | 4x3 | Width/height of the node grid. |
| **Segment Length** | 30-50 | Range for road segment lengths. |
| **Intersection Jitter** | 3 | Node displacement in world units. |
| **Traffic Light %** | 100% | Percentage of eligible intersections (3+ roads) to signalize. |
| **Deadlock Timeout** | 5.0s | Time threshold for forced entry during gridlock (unsignalized only). |
| **Random Seed** | 0 | Fixed seed for reproducible networks (0 = random). |

Vehicle parameters (target speed, braking, follow headway) live on the car prefab's `CarController`; car count lives on the `SimulationManager`.

## Telemetry

The system exports vehicle data to `PythonAnalysis/vehicle_positions.csv` at 10 Hz (one row per car per interval, file recreated each run).

**Format**:
```csv
vehicle_id, positionX, positionY, positionZ, time
```

`PythonAnalysis/brake_analysis.py` (Python 3) computes per-vehicle speed/acceleration from the log and reports braking events that exceed the cars' physical braking capability — useful for regression-testing driving behavior.

## Interaction & Usage

**Note**: The project is currently designed for interaction exclusively via the Unity Editor.
*   **Configuration**: Parameters are modified via the `RoadGenerationConfig` asset in the Inspector.
*   **Visualization**: Runtime monitoring is performed through the Unity Scene View (Gizmos) and Inspector debug data. There is no runtime UI or build-ready player interface.
*   **Editor tip**: Set *Preferences > General > Script Changes While Playing* to "Recompile After Finished Playing" — the road graph does not survive a mid-play domain reload.

## Known Issues

*   **Topology Generation**: A final width-aware repair pass removes sub-angle spikes, roads grazing an unrelated intersection, and un-bridgeable at-grade crossings, so these no longer reach the final graph. At extreme jitter (well above segment length) the repair may thin the network to keep it valid; reduce `IntersectionJitter` or raise segment lengths for denser maps.
*   **Right of Way**: Turning cars yield to conflicting higher-priority movements (straight > right > left) from any approach, and priority-to-the-right recognises a close approaching car, not only a fully stopped one. Priority can still be overridden by the deadlock breaker, run-light commitments, and the cannot-stop physics exemption (a genuine mover that can no longer stop is let through; a stopped or restarting car always holds) — these are intentional.
*   **Turn Geometry**: Opposite-approach left turns whose arcs would overlap are flattened at generation so both can proceed on the same green; the flattening breaks tangent continuity at the turn's entry/exit, so cars visibly snap their heading there. Pairs that cannot be separated (one turn exits onto the other's approach road) simply take turns. Sharp turns may still bulge slightly outside the intersection footprint.

## Limitations

*   **Collision Complexity**: O(N²) collision detection; optimized for <100 vehicles.
*   **Perception**: Agents see one path segment ahead (through the next intersection), but no further.
*   **Turn Restrictions**: U-turns are disabled to prevent flow loops.
*   **Signal Timing**: Traffic lights operate on fixed-time intervals without sensor actuation.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
