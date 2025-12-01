# Unity Procedural Traffic Simulation

A 3D traffic simulation framework implemented in Unity. This system features procedural road network generation, physics-based vehicle dynamics, and autonomous intersection management with support for signalized and unsignalized traffic control.

## System Overview

The simulation operates on a right-hand traffic model and generates road networks procedurally using a grid-based algorithm with organic jitter. It supports dynamic bridge generation, "priority-to-the-right" intersection rules, deadlock detection, and traffic light phasing.

### Core Capabilities

*   **Procedural Topology**: Automated generation of nodes and edges with configurable jitter, bridges, and randomized connectivity.
*   **Agent Dynamics**: Physics-based vehicle movement including kinematic braking, collision avoidance, and queue management.
*   **Intersection Logic**: Autonomous conflict resolution using priority rules, deadlock timeouts, and traffic signal state machines.
*   **Telemetry**: High-frequency CSV export of vehicle positions for external analysis.

## Technical Architecture

### Control Systems
*   **`SimulationManager`**: Central orchestrator for subsystem initialization and update loops.
*   **`IntersectionManager`**: Manages node states, priority enforcement, and queue tracking.
*   **`StopLightController`**: Implements state machines for signalized intersections (Green/Yellow/Red phases).

### Generation Pipeline
*   **`RoadNetworkGenerator`**: Executes multi-pass procedural generation (Grid -> Topology -> Pruning -> Bridges -> Geometry).
*   **`RoadNetworkData`**: Defines graph structures (`Node`, `Road`, `Lane`, `TurnCurve`, `BridgeInfo`).

### Agents
*   **`CarController`**: Handles individual vehicle state (`OnLane`, `Approaching`, `Waiting`, `Crossing`) and physics.

## Installation

### Prerequisites
*   Unity Editor 2021.3 or later.
*   Standard 3D Render Pipeline.

### Setup
1.  Clone the repository into the root of a standard 3D Unity project.
2.  Ensure all scripts are located in `Assets/Scripts`.
3.  Create a `RoadGenerationConfig` asset via **Create > Traffic Simulation > Road Generation Config**.
4.  Configure the `SimulationManager` GameObject with required components (`RoadNetworkGenerator`, `RoadNetworkVisualizer`, `IntersectionManager`, `CSVLogger`).
5.  Assign the `CarController` prefab to the simulation manager.

## Configuration

Configuration is managed via the `RoadGenerationConfig` ScriptableObject.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| **Grid Dimensions** | 4x4 | Width/Height of the node grid. |
| **Segment Length** | 50-100 | Range for road segment lengths. |
| **Intersection Jitter** | 0.3 | Node displacement factor (0.0-1.0). |
| **Traffic Light %** | 50% | Percentage of eligible intersections to signalize. |
| **Deadlock Timeout** | 3.0s | Time threshold for forced entry during gridlock. |

## Telemetry

The system exports vehicle data to `vehicle_positions.csv` in the project root.

**Format**:
```csv
vehicle_id, positionX, positionY, positionZ, time
```

**Note**: This repository generates raw data only. No internal analysis tools are included; must parse the CSV/JSON output externally.

## Interaction & Usage

**Note**: The project is currently designed for interaction exclusively via the Unity Editor.
*   **Configuration**: Parameters are modified via the `RoadGenerationConfig` asset in the Inspector.
*   **Visualization**: Runtime monitoring is performed through the Unity Scene View (Gizmos) and Inspector debug data. There is no runtime UI or build-ready player interface.

## Known Issues

*   **Braking Instability**: Sporadic sudden braking events may occur due to physics calculation edge cases.
*   **Priority Logic**: Signalized intersections may exhibit suboptimal yielding behavior under heavy load.
*   **Topology Generation**: High jitter values (>0.5) can result in lane overlaps or generation failures.
*   **Bridge Geometry**: Multi-segment bridges do not currently maintain a continuous smooth curve.

## Limitations

*   **Collision Complexity**: O(N²) collision detection; optimized for <100 vehicles.
*   **Perception**: Agents do not perceive traffic beyond the immediate intersection.
*   **Turn Restrictions**: U-turns are disabled to prevent flow loops.
*   **Signal Timing**: Traffic lights operate on fixed-time intervals without sensor actuation.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.