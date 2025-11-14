"""
Example Usage of Road Network Visualization and Heatmap Generation

This script demonstrates how to:
1. Load an exported road network from Unity
2. Visualize the network in various ways
3. Load vehicle position data (CSV)
4. Generate traffic heatmaps
"""

import matplotlib.pyplot as plt
from network_loader import RoadNetwork
from visualization import plot_network_2d, plot_network_3d, plot_combined_view
from heatmap_generator import TrafficHeatMap


def example_network_visualization():
    """
    Example 1: Load and visualize road network
    """
    print("="*60)
    print("EXAMPLE 1: Network Visualization")
    print("="*60)
    
    # Load the exported network
    network = RoadNetwork('road_network_export.json')
    
    # Option A: Simple 2D view
    print("\nGenerating 2D network plot...")
    fig, ax = plt.subplots(figsize=(14, 14))
    plot_network_2d(network, ax=ax, show_labels=True, show_arrows=True, show_bridges=True)
    plt.savefig('network_2d.png', dpi=300, bbox_inches='tight')
    print("Saved: network_2d.png")
    
    # Option B: 3D view with bridges
    print("\nGenerating 3D network plot...")
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    plot_network_3d(network, ax=ax, show_labels=False)
    plt.savefig('network_3d.png', dpi=300, bbox_inches='tight')
    print("Saved: network_3d.png")
    
    # Option C: Combined view with statistics
    print("\nGenerating combined view...")
    fig, axes = plot_combined_view(network, figsize=(18, 12))
    plt.savefig('network_combined.png', dpi=300, bbox_inches='tight')
    print("Saved: network_combined.png")
    
    plt.show()
    print("\nVisualization complete!")


def example_network_queries():
    """
    Example 2: Query network data programmatically
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: Network Queries")
    print("="*60)
    
    network = RoadNetwork('road_network_export.json')
    
    # Get network bounds
    bounds = network.get_network_bounds()
    print(f"\nNetwork bounds:")
    print(f"  X: {bounds['x_min']:.1f} to {bounds['x_max']:.1f}")
    print(f"  Z: {bounds['z_min']:.1f} to {bounds['z_max']:.1f}")
    
    # Get a specific node
    node_id = 0
    node = network.nodes[node_id]
    print(f"\nNode {node_id}:")
    print(f"  Position: ({node['X']:.1f}, {node['Y']:.1f}, {node['Z']:.1f})")
    print(f"  Connected roads: {node['ConnectedRoadIDs']}")
    
    # Get roads at that node
    roads = network.get_roads_at_node(node_id)
    print(f"  Number of roads: {len(roads)}")
    
    # Get a lane path
    lane_id = 0
    lane_path = network.get_lane_path(lane_id, num_points=20)
    print(f"\nLane {lane_id} path (first 3 points):")
    for i in range(min(3, len(lane_path))):
        print(f"  Point {i}: ({lane_path[i,0]:.1f}, {lane_path[i,1]:.1f}, {lane_path[i,2]:.1f})")


def example_heatmap_generation():
    """
    Example 3: Generate traffic heatmaps from vehicle data
    
    NOTE: This requires vehicle position data CSV file.
    Expected format: vehicle_id, positionX, positionY, positionZ, time
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: Traffic Heatmap Generation")
    print("="*60)
    
    # Load network
    network = RoadNetwork('road_network_export.json')
    
    # Create heatmap generator
    heatmap = TrafficHeatMap(network)
    
    # Try to load vehicle data
    try:
        heatmap.load_vehicle_data('vehicle_positions.csv')
        
        # Generate and plot density heatmap
        print("\nGenerating density heatmap...")
        fig, ax = plt.subplots(figsize=(14, 14))
        heatmap.plot_density_heatmap(ax=ax, overlay_network=True)
        plt.savefig('traffic_density.png', dpi=300, bbox_inches='tight')
        print("Saved: traffic_density.png")
        
        # Generate and plot speed heatmap
        print("\nGenerating speed heatmap...")
        fig, ax = plt.subplots(figsize=(14, 14))
        heatmap.plot_speed_heatmap(ax=ax, overlay_network=True)
        plt.savefig('traffic_speed.png', dpi=300, bbox_inches='tight')
        print("Saved: traffic_speed.png")
        
        # Combined heatmaps
        print("\nGenerating combined heatmaps...")
        fig, axes = heatmap.plot_combined_heatmaps(figsize=(18, 8))
        plt.savefig('traffic_heatmaps_combined.png', dpi=300, bbox_inches='tight')
        print("Saved: traffic_heatmaps_combined.png")
        
        # Time-series heatmaps (for animation)
        print("\nGenerating time-series heatmaps...")
        heatmap.generate_time_series_plots(output_dir='./time_series', num_frames=10)
        print("Saved time-series frames to ./time_series/")
        
        plt.show()
        print("\nHeatmap generation complete!")
        
    except FileNotFoundError:
        print("\nWARNING: vehicle_positions.csv not found!")
        print("To generate heatmaps, you need to:")
        print("1. Run traffic simulation in Unity")
        print("2. Export vehicle position data to CSV")
        print("3. Place CSV file in this directory")
        print("\nExpected CSV format:")
        print("  vehicle_id, positionX, positionY, positionZ, time")


def example_time_window_analysis():
    """
    Example 4: Analyze specific time periods
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: Time Window Analysis")
    print("="*60)
    
    network = RoadNetwork('road_network_export.json')
    heatmap = TrafficHeatMap(network)
    
    try:
        heatmap.load_vehicle_data('vehicle_positions.csv')
        
        # Analyze first 30 seconds
        print("\nAnalyzing time window: 0-30 seconds...")
        fig, axes = heatmap.plot_combined_heatmaps(
            time_window=(0, 30),
            figsize=(18, 8)
        )
        plt.savefig('heatmap_0_30s.png', dpi=300, bbox_inches='tight')
        print("Saved: heatmap_0_30s.png")
        
        # Analyze 30-60 seconds
        print("\nAnalyzing time window: 30-60 seconds...")
        fig, axes = heatmap.plot_combined_heatmaps(
            time_window=(30, 60),
            figsize=(18, 8)
        )
        plt.savefig('heatmap_30_60s.png', dpi=300, bbox_inches='tight')
        print("Saved: heatmap_30_60s.png")
        
        plt.show()
        
    except FileNotFoundError:
        print("\nWARNING: vehicle_positions.csv not found!")


if __name__ == "__main__":
    print("\n")
    print("╔" + "="*58 + "╗")
    print("║" + " "*8 + "Road Network Visualization Examples" + " "*14 + "║")
    print("╚" + "="*58 + "╝")
    print()
    
    # Run examples
    try:
        # Example 1: Visualize the network
        example_network_visualization()
        
        # Example 2: Query network data
        example_network_queries()
        
        # Example 3: Generate heatmaps (if vehicle data available)
        example_heatmap_generation()
        
        # Example 4: Time window analysis (if vehicle data available)
        # example_time_window_analysis()
        
    except FileNotFoundError as e:
        print(f"\nERROR: Could not find required file: {e}")
        print("\nMake sure:")
        print("1. Road network has been exported from Unity")
        print("2. File 'road_network_export.json' exists in this directory")
        print("3. For heatmaps: 'vehicle_positions.csv' exists")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "="*60)
    print("Done!")
    print("="*60 + "\n")

