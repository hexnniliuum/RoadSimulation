# Road Network Analysis Tools

Python scripts for visualizing and analyzing road networks exported from Unity.

## Overview

This folder contains tools to:
- Load and visualize road networks exported from Unity (JSON format)
- Generate traffic density and speed heatmaps from vehicle simulation data (CSV format)
- Analyze network topology and traffic patterns

## Requirements

Install required Python packages:

```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` - Numerical operations
- `pandas` - Data handling (CSV loading)
- `matplotlib` - Plotting and visualization
- `scipy` - Gaussian smoothing for heatmaps

## File Structure

```
PythonAnalysis/
├── README.md                    # This file
├── requirements.txt             # Python package requirements
├── network_loader.py            # Load and parse JSON network data
├── visualization.py             # Network visualization functions
├── heatmap_generator.py         # Traffic heatmap generation
├── example_usage.py             # Example scripts and usage
├── road_network_export.json     # Exported from Unity (auto-created)
└── vehicle_positions.csv        # Vehicle data (you provide this)
```

## Quick Start

### Step 1: Export Road Network from Unity

1. Open your Unity project
2. Make sure a road network is generated (Play mode)
3. Select the `SimulationManager` GameObject in the Hierarchy
4. Find the `RoadNetworkGenerator` component in the Inspector
5. Right-click on the component
6. Select **"Export Road Network to JSON"**
7. This creates `PythonAnalysis/road_network_export.json`

### Step 2: Run Example Visualizations

```bash
python example_usage.py
```

This will generate:
- `network_2d.png` - 2D top-down view of the network
- `network_3d.png` - 3D view showing bridge elevations
- `network_combined.png` - Combined view with statistics

### Step 3: Generate Traffic Heatmaps (Optional)

To generate heatmaps, you need vehicle position data in CSV format.

**Expected CSV format:**
```csv
vehicle_id,positionX,positionY,positionZ,time
0,100.5,0.0,50.2,0.016
0,101.2,0.0,50.8,0.033
1,150.0,0.0,75.3,0.016
...
```

Once you have `vehicle_positions.csv`, run:

```bash
python example_usage.py
```

This will additionally generate:
- `traffic_density.png` - Heat map of vehicle density
- `traffic_speed.png` - Heat map of average speeds
- `traffic_heatmaps_combined.png` - Both heatmaps side-by-side
- `time_series/` folder - Frame-by-frame heatmaps

## Usage Examples

### Example 1: Basic Network Visualization

```python
from network_loader import RoadNetwork
from visualization import plot_network_2d
import matplotlib.pyplot as plt

# Load network
network = RoadNetwork('road_network_export.json')

# Visualize
fig, ax = plt.subplots(figsize=(14, 14))
plot_network_2d(network, ax=ax, show_labels=True, show_bridges=True)
plt.savefig('my_network.png', dpi=300)
plt.show()
```

### Example 2: Query Network Data

```python
from network_loader import RoadNetwork

network = RoadNetwork('road_network_export.json')

# Get all node IDs
node_ids = network.get_all_node_ids()

# Get specific node position
pos = network.get_node_position(node_id=0)  # Returns (x, z)

# Get lane path with bridge elevations
lane_path = network.get_lane_path(lane_id=5, num_points=100)
# Returns numpy array of shape (100, 3) with [x, z, y] coordinates

# Get network bounds
bounds = network.get_network_bounds()
print(f"X range: {bounds['x_min']} to {bounds['x_max']}")
print(f"Z range: {bounds['z_min']} to {bounds['z_max']}")
```

### Example 3: Generate Traffic Heatmaps

```python
from network_loader import RoadNetwork
from heatmap_generator import TrafficHeatMap
import matplotlib.pyplot as plt

# Load network and vehicle data
network = RoadNetwork('road_network_export.json')
heatmap = TrafficHeatMap(network)
heatmap.load_vehicle_data('vehicle_positions.csv')

# Generate density heatmap
fig, ax = plt.subplots(figsize=(14, 14))
heatmap.plot_density_heatmap(ax=ax, overlay_network=True)
plt.savefig('density.png', dpi=300)
plt.show()

# Generate speed heatmap
fig, ax = plt.subplots(figsize=(14, 14))
heatmap.plot_speed_heatmap(ax=ax, overlay_network=True)
plt.savefig('speed.png', dpi=300)
plt.show()
```

### Example 4: Time Window Analysis

```python
# Analyze specific time period
heatmap.plot_density_heatmap(
    time_window=(0, 30),  # First 30 seconds
    overlay_network=True
)
plt.savefig('density_0_30s.png', dpi=300)
plt.show()
```

### Example 5: Generate Animation Frames

```python
# Generate sequence of heatmaps over time
heatmap.generate_time_series_plots(
    output_dir='./animation_frames',
    num_frames=60  # 60 frames
)

# These can be combined into a video using ffmpeg:
# ffmpeg -framerate 10 -i animation_frames/heatmap_frame_%03d.png -c:v libx264 -pix_fmt yuv420p traffic_animation.mp4
```

## API Reference

### RoadNetwork Class

**Methods:**
- `get_node_position(node_id)` - Get 2D (x, z) position
- `get_node_position_3d(node_id)` - Get 3D (x, y, z) position
- `get_lane_path(lane_id, num_points)` - Get points along lane with bridges
- `get_turn_curve_path(turn_id, num_points)` - Get Bézier curve points
- `get_all_lane_ids()` - Get list of all lane IDs
- `get_all_node_ids()` - Get list of all node IDs
- `get_roads_at_node(node_id)` - Get roads connected to a node
- `get_network_bounds()` - Get bounding box
- `find_lane_by_nodes(start_id, end_id)` - Find lane between two nodes

### Visualization Functions

**Functions:**
- `plot_network_2d(network, ax, show_labels, show_arrows, show_bridges)` - 2D plot
- `plot_network_3d(network, ax, show_labels)` - 3D plot with elevation
- `plot_turn_curves(network, ax, color_by_type)` - Plot turn curves
- `plot_combined_view(network, figsize)` - Multi-panel view with stats

### TrafficHeatMap Class

**Methods:**
- `load_vehicle_data(csv_path)` - Load vehicle positions from CSV
- `calculate_vehicle_speeds()` - Calculate speeds from position data
- `generate_density_heatmap(time_window, grid_size, sigma)` - Get density grid
- `generate_speed_heatmap(time_window, grid_size, sigma)` - Get speed grid
- `plot_density_heatmap(time_window, ax, overlay_network)` - Plot density
- `plot_speed_heatmap(time_window, ax, overlay_network)` - Plot speeds
- `plot_combined_heatmaps(time_window, figsize)` - Plot both side-by-side
- `generate_time_series_plots(output_dir, num_frames)` - Generate animation frames

## Data Formats

### Network JSON Structure

Exported from Unity, contains:
- `RandomSeed` - Generation seed
- `Nodes` - List of intersection nodes with positions
- `Roads` - List of road segments connecting nodes
- `Lanes` - List of directional lanes (2 per road)
- `Bridges` - List of bridge elevations where roads cross
- `TurnCurves` - List of Bézier curves for turns at intersections

### Vehicle CSV Format

You provide this from your simulation:

```csv
vehicle_id,positionX,positionY,positionZ,time
0,100.5,0.0,50.2,0.016
0,101.2,0.0,50.8,0.033
```

**Columns:**
- `vehicle_id` - Unique vehicle identifier (int)
- `positionX` - X coordinate in Unity (float)
- `positionY` - Y coordinate in Unity (elevation, float)
- `positionZ` - Z coordinate in Unity (float)
- `time` - Simulation time in seconds (float)

**Tips:**
- Log vehicle positions every frame or every N frames
- Include all vehicles in the simulation
- Sort by time for better performance
- Speeds are auto-calculated from consecutive positions

## Customization

### Adjusting Heatmap Parameters

```python
# Density heatmap with finer grid and more smoothing
heatmap.generate_density_heatmap(
    time_window=None,
    grid_size=2.0,    # Smaller = finer detail
    sigma=3.0         # Larger = smoother
)

# Speed heatmap with coarser grid
heatmap.generate_speed_heatmap(
    grid_size=10.0,   # Larger = coarser, faster
    sigma=1.0         # Less smoothing
)
```

### Custom Color Maps

```python
# Use different colormap
im = ax.imshow(H.T, cmap='viridis')  # Options: hot, viridis, plasma, jet, etc.
```

## Troubleshooting

### "FileNotFoundError: road_network_export.json"
- Make sure you've exported the network from Unity first
- Check that the file is in the `PythonAnalysis/` folder
- Verify the path in your Python script

### "ValueError: Missing required columns"
- Check your CSV has the correct column names
- Column names are case-sensitive
- Use commas as delimiters (not tabs or semicolons)

### Heatmaps are empty/blank
- Check that vehicle_data contains positions within network bounds
- Try increasing `grid_size` parameter
- Verify time_window contains data

### Plots don't match Unity Scene view
- Unity uses Y-up coordinate system
- Plots show XZ plane (bird's eye view)
- Bridges show elevation on Y axis in 3D plots

## Future Enhancements

Potential additions:
- Real-time streaming from Unity (WebSocket)
- Congestion analysis and bottleneck detection
- Path analysis and route optimization
- Traffic light timing analysis
- Vehicle trajectory visualization
- Statistical reports (PDF export)

## License

Part of the Unity Traffic Simulation project.

## Contact

For issues or questions about the Python analysis tools, refer to the main project documentation.

