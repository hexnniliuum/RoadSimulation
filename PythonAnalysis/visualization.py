"""
Road Network Visualization
Provides plotting functions for road networks exported from Unity.
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D


def plot_network_2d(network, ax=None, show_labels=True, show_arrows=True, show_bridges=True):
    """
    Plot the road network in 2D (top-down view on XZ plane).
    
    Args:
        network: RoadNetwork object
        ax: Matplotlib axis (creates new if None)
        show_labels: Whether to show node IDs
        show_arrows: Whether to show lane direction arrows
        show_bridges: Whether to highlight bridge segments
        
    Returns:
        Matplotlib axis
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(14, 14))
    
    # Plot roads (gray lines between nodes)
    for road in network.roads.values():
        start_pos = network.get_node_position(road['StartNodeID'])
        end_pos = network.get_node_position(road['EndNodeID'])
        ax.plot([start_pos[0], end_pos[0]], 
               [start_pos[1], end_pos[1]], 
               'gray', linewidth=3, alpha=0.4, zorder=1)
    
    # Plot lanes with direction and bridges
    for lane in network.lanes.values():
        path = network.get_lane_path(lane['ID'], num_points=50)
        
        # Check if lane has bridges
        has_bridge = lane['BridgeIndices'] and len(lane['BridgeIndices']) > 0
        
        if has_bridge and show_bridges:
            # Plot lane in segments, coloring bridges differently
            for i in range(len(path) - 1):
                # Check if this segment is on a bridge
                mid_t = (i + 0.5) / (len(path) - 1)
                mid_dist = mid_t * lane['Length']
                
                # Convert to road distance
                if lane['IsReversed']:
                    road_dist = lane['RoadLength'] - lane['StopLineDistanceAtStart'] - mid_dist
                else:
                    road_dist = lane['StopLineDistanceAtStart'] + mid_dist
                
                # Check if on any bridge
                on_bridge = False
                for bridge_idx in lane['BridgeIndices']:
                    bridge = network.bridges[bridge_idx]
                    if bridge['StartDistance'] <= road_dist <= bridge['EndDistance']:
                        on_bridge = True
                        break
                
                color = 'magenta' if on_bridge else 'cyan'
                linewidth = 2 if on_bridge else 1.5
                ax.plot(path[i:i+2, 0], path[i:i+2, 1], 
                       color=color, linewidth=linewidth, alpha=0.8, zorder=2)
        else:
            # Simple lane plot
            ax.plot(path[:, 0], path[:, 1], 'cyan', linewidth=1.5, alpha=0.8, zorder=2)
        
        # Draw direction arrow at midpoint
        if show_arrows:
            mid_idx = len(path) // 2
            if mid_idx < len(path) - 1:
                dx = path[mid_idx + 1, 0] - path[mid_idx, 0]
                dz = path[mid_idx + 1, 1] - path[mid_idx, 1]
                ax.arrow(path[mid_idx, 0], path[mid_idx, 1], 
                        dx * 3, dz * 3,
                        head_width=2, head_length=1.5, 
                        fc='cyan', ec='cyan', alpha=0.7, zorder=3)
    
    # Plot nodes
    for node in network.nodes.values():
        ax.plot(node['X'], node['Z'], 'yo', markersize=12, 
               markeredgecolor='black', markeredgewidth=1.5, zorder=4)
        
        if show_labels:
            ax.text(node['X'], node['Z'], str(node['ID']), 
                   ha='center', va='center', fontsize=9, 
                   fontweight='bold', zorder=5)
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (Unity)', fontsize=12)
    ax.set_ylabel('Z (Unity)', fontsize=12)
    ax.set_title(f'Road Network (Seed: {network.data["RandomSeed"]})', fontsize=14)
    
    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], color='gray', linewidth=3, label='Roads'),
        plt.Line2D([0], [0], color='cyan', linewidth=1.5, label='Lanes'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', 
                  markersize=10, markeredgecolor='black', label='Nodes')
    ]
    
    if show_bridges:
        legend_elements.append(
            plt.Line2D([0], [0], color='magenta', linewidth=2, label='Bridges')
        )
    
    ax.legend(handles=legend_elements, loc='upper right')
    
    return ax


def plot_network_3d(network, ax=None, show_labels=False):
    """
    Plot the road network in 3D showing bridge elevations.
    
    Args:
        network: RoadNetwork object
        ax: Matplotlib 3D axis (creates new if None)
        show_labels: Whether to show node IDs
        
    Returns:
        Matplotlib 3D axis
    """
    if ax is None:
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
    
    # Plot lanes with elevation
    for lane in network.lanes.values():
        path = network.get_lane_path(lane['ID'], num_points=100)
        
        # Color by bridge
        has_bridge = lane['BridgeIndices'] and len(lane['BridgeIndices']) > 0
        color = 'magenta' if has_bridge else 'cyan'
        linewidth = 2 if has_bridge else 1
        
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 
               color=color, linewidth=linewidth, alpha=0.7)
    
    # Plot nodes
    for node in network.nodes.values():
        ax.scatter(node['X'], node['Z'], node['Y'], 
                  c='yellow', s=100, edgecolor='black', linewidth=1.5)
        
        if show_labels:
            ax.text(node['X'], node['Z'], node['Y'], str(node['ID']))
    
    ax.set_xlabel('X (Unity)')
    ax.set_ylabel('Z (Unity)')
    ax.set_zlabel('Y (Elevation)')
    ax.set_title(f'Road Network 3D View (Seed: {network.data["RandomSeed"]})')
    
    return ax


def plot_turn_curves(network, ax=None, color_by_type=True):
    """
    Plot turn curves at intersections.
    
    Args:
        network: RoadNetwork object
        ax: Matplotlib axis (creates new if None)
        color_by_type: Whether to color by turn type (Straight/Left/Right)
        
    Returns:
        Matplotlib axis
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(14, 14))
    
    # Color mapping for turn types
    turn_colors = {
        'Straight': 'green',
        'Left': 'blue',
        'Right': 'red'
    }
    
    # Plot turn curves
    for turn in network.turns.values():
        path = network.get_turn_curve_path(turn['ID'], num_points=30)
        
        color = turn_colors.get(turn['TurnType'], 'gray') if color_by_type else 'orange'
        
        ax.plot(path[:, 0], path[:, 1], color=color, 
               linewidth=2, alpha=0.6)
        
        # Mark start and end
        ax.plot(path[0, 0], path[0, 1], 'o', color=color, markersize=4)
        ax.plot(path[-1, 0], path[-1, 1], 's', color=color, markersize=4)
    
    # Plot nodes
    for node in network.nodes.values():
        ax.plot(node['X'], node['Z'], 'yo', markersize=10, 
               markeredgecolor='black', markeredgewidth=1)
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (Unity)')
    ax.set_ylabel('Z (Unity)')
    ax.set_title('Turn Curves')
    
    # Legend
    if color_by_type:
        legend_elements = [
            plt.Line2D([0], [0], color='green', linewidth=2, label='Straight'),
            plt.Line2D([0], [0], color='blue', linewidth=2, label='Left Turn'),
            plt.Line2D([0], [0], color='red', linewidth=2, label='Right Turn')
        ]
        ax.legend(handles=legend_elements)
    
    return ax


def plot_combined_view(network, figsize=(18, 12)):
    """
    Create a combined view with multiple subplots.
    
    Args:
        network: RoadNetwork object
        figsize: Figure size tuple
        
    Returns:
        Figure and axes
    """
    fig = plt.figure(figsize=figsize)
    
    # 2D network view
    ax1 = fig.add_subplot(221)
    plot_network_2d(network, ax=ax1, show_labels=True, show_arrows=True)
    ax1.set_title('Network Layout (2D)')
    
    # 3D view
    ax2 = fig.add_subplot(222, projection='3d')
    plot_network_3d(network, ax=ax2)
    ax2.set_title('Network with Bridges (3D)')
    
    # Turn curves
    ax3 = fig.add_subplot(223)
    plot_turn_curves(network, ax=ax3)
    ax3.set_title('Turn Curves')
    
    # Network statistics
    ax4 = fig.add_subplot(224)
    ax4.axis('off')
    
    # Calculate statistics
    total_road_length = sum(road['Length'] for road in network.roads.values())
    avg_road_length = total_road_length / len(network.roads) if network.roads else 0
    
    num_bridges = len(network.bridges)
    
    turn_type_counts = {}
    for turn in network.turns.values():
        turn_type = turn['TurnType']
        turn_type_counts[turn_type] = turn_type_counts.get(turn_type, 0) + 1
    
    stats_text = f"""
    NETWORK STATISTICS
    ═══════════════════════════
    Random Seed: {network.data['RandomSeed']}
    
    Nodes: {len(network.nodes)}
    Roads: {len(network.roads)}
    Lanes: {len(network.lanes)}
    
    Total Road Length: {total_road_length:.1f} units
    Avg Road Length: {avg_road_length:.1f} units
    
    Bridges: {num_bridges}
    
    Turn Curves: {len(network.turns)}
      - Straight: {turn_type_counts.get('Straight', 0)}
      - Left: {turn_type_counts.get('Left', 0)}
      - Right: {turn_type_counts.get('Right', 0)}
    """
    
    ax4.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
            verticalalignment='center')
    
    plt.tight_layout()
    
    return fig, (ax1, ax2, ax3, ax4)

