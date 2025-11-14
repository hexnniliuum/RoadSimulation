"""
Traffic Heatmap Generator
Generates traffic density and speed heatmaps from vehicle position data.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.ndimage import gaussian_filter


class TrafficHeatMap:
    """
    Generates and visualizes traffic heatmaps from vehicle position data.
    Expected CSV format: vehicle_id, positionX, positionY, positionZ, time
    """
    
    def __init__(self, network):
        """
        Initialize heatmap generator.
        
        Args:
            network: RoadNetwork object
        """
        self.network = network
        self.vehicle_data = None
    
    def load_vehicle_data(self, csv_path):
        """
        Load vehicle position data from CSV file.
        
        Args:
            csv_path: Path to CSV file with columns: 
                     vehicle_id, positionX, positionY, positionZ, time
        """
        self.vehicle_data = pd.read_csv(csv_path)
        
        # Validate columns
        required_cols = ['vehicle_id', 'positionX', 'positionY', 'positionZ', 'time']
        missing_cols = [col for col in required_cols if col not in self.vehicle_data.columns]
        
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        print(f"Loaded vehicle data: {len(self.vehicle_data)} records")
        print(f"  Unique vehicles: {self.vehicle_data['vehicle_id'].nunique()}")
        print(f"  Time range: {self.vehicle_data['time'].min():.2f} - {self.vehicle_data['time'].max():.2f}")
    
    def generate_density_heatmap(self, time_window=None, grid_size=5.0, sigma=2.0):
        """
        Generate 2D traffic density heatmap.
        
        Args:
            time_window: Tuple (start_time, end_time) or None for all time
            grid_size: Size of grid cells in Unity units
            sigma: Gaussian smoothing parameter (higher = smoother)
            
        Returns:
            Tuple (H, xedges, zedges) where H is the 2D histogram
        """
        if self.vehicle_data is None:
            raise ValueError("No vehicle data loaded. Call load_vehicle_data() first.")
        
        # Filter by time window
        data = self.vehicle_data
        if time_window:
            data = data[(data['time'] >= time_window[0]) & 
                       (data['time'] <= time_window[1])]
        
        if len(data) == 0:
            print("Warning: No data in specified time window")
            return None, None, None
        
        # Get network bounds for grid
        bounds = self.network.get_network_bounds()
        
        # Add padding
        padding = 10.0
        x_min = bounds['x_min'] - padding
        x_max = bounds['x_max'] + padding
        z_min = bounds['z_min'] - padding
        z_max = bounds['z_max'] + padding
        
        # Create bins
        x_bins = np.arange(x_min, x_max + grid_size, grid_size)
        z_bins = np.arange(z_min, z_max + grid_size, grid_size)
        
        # Create 2D histogram
        H, xedges, zedges = np.histogram2d(
            data['positionX'], 
            data['positionZ'],
            bins=[x_bins, z_bins]
        )
        
        # Apply Gaussian smoothing
        H_smooth = gaussian_filter(H, sigma=sigma)
        
        return H_smooth, xedges, zedges
    
    def calculate_vehicle_speeds(self):
        """
        Calculate vehicle speeds from position data.
        Adds 'speed' column to vehicle_data DataFrame.
        """
        if self.vehicle_data is None:
            raise ValueError("No vehicle data loaded.")
        
        # Sort by vehicle and time
        df = self.vehicle_data.sort_values(['vehicle_id', 'time'])
        
        # Calculate speeds (distance / time between consecutive points)
        speeds = []
        
        for vehicle_id in df['vehicle_id'].unique():
            vehicle_data = df[df['vehicle_id'] == vehicle_id]
            
            for i in range(len(vehicle_data)):
                if i == 0:
                    speeds.append(0.0)  # First point has no previous speed
                else:
                    prev = vehicle_data.iloc[i-1]
                    curr = vehicle_data.iloc[i]
                    
                    # Calculate distance
                    dx = curr['positionX'] - prev['positionX']
                    dy = curr['positionY'] - prev['positionY']
                    dz = curr['positionZ'] - prev['positionZ']
                    distance = np.sqrt(dx**2 + dy**2 + dz**2)
                    
                    # Calculate time delta
                    dt = curr['time'] - prev['time']
                    
                    # Calculate speed (avoid division by zero)
                    speed = distance / dt if dt > 0 else 0.0
                    speeds.append(speed)
        
        df['speed'] = speeds
        self.vehicle_data = df
        
        print(f"Calculated speeds - Mean: {df['speed'].mean():.2f}, Max: {df['speed'].max():.2f}")
    
    def generate_speed_heatmap(self, time_window=None, grid_size=5.0, sigma=2.0):
        """
        Generate average speed heatmap.
        
        Args:
            time_window: Tuple (start_time, end_time) or None for all time
            grid_size: Size of grid cells
            sigma: Gaussian smoothing parameter
            
        Returns:
            Tuple (speed_grid, xedges, zedges)
        """
        if 'speed' not in self.vehicle_data.columns:
            self.calculate_vehicle_speeds()
        
        # Filter by time window
        data = self.vehicle_data
        if time_window:
            data = data[(data['time'] >= time_window[0]) & 
                       (data['time'] <= time_window[1])]
        
        # Get network bounds
        bounds = self.network.get_network_bounds()
        padding = 10.0
        
        x_min = bounds['x_min'] - padding
        x_max = bounds['x_max'] + padding
        z_min = bounds['z_min'] - padding
        z_max = bounds['z_max'] + padding
        
        # Create bins
        x_bins = np.arange(x_min, x_max + grid_size, grid_size)
        z_bins = np.arange(z_min, z_max + grid_size, grid_size)
        
        # Create grids for sum and count
        speed_sum, _, _ = np.histogram2d(
            data['positionX'], 
            data['positionZ'],
            bins=[x_bins, z_bins],
            weights=data['speed']
        )
        
        count, xedges, zedges = np.histogram2d(
            data['positionX'], 
            data['positionZ'],
            bins=[x_bins, z_bins]
        )
        
        # Calculate average (avoid division by zero)
        with np.errstate(divide='ignore', invalid='ignore'):
            speed_grid = np.where(count > 0, speed_sum / count, 0)
        
        # Apply Gaussian smoothing
        speed_grid_smooth = gaussian_filter(speed_grid, sigma=sigma)
        
        return speed_grid_smooth, xedges, zedges
    
    def plot_density_heatmap(self, time_window=None, ax=None, overlay_network=True):
        """
        Plot traffic density heatmap.
        
        Args:
            time_window: Tuple (start_time, end_time) or None for all time
            ax: Matplotlib axis (creates new if None)
            overlay_network: Whether to overlay road network
            
        Returns:
            Matplotlib axis
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 14))
        
        # Generate heatmap
        H, xedges, zedges = self.generate_density_heatmap(time_window)
        
        if H is None:
            print("No data to plot")
            return ax
        
        # Plot heatmap
        im = ax.imshow(H.T, origin='lower', cmap='hot', 
                      extent=[xedges[0], xedges[-1], zedges[0], zedges[-1]],
                      alpha=0.7, interpolation='bilinear')
        
        # Overlay road network
        if overlay_network:
            # Plot roads as thin lines
            for road in self.network.roads.values():
                start_pos = self.network.get_node_position(road['StartNodeID'])
                end_pos = self.network.get_node_position(road['EndNodeID'])
                ax.plot([start_pos[0], end_pos[0]], 
                       [start_pos[1], end_pos[1]], 
                       'cyan', linewidth=1, alpha=0.5)
            
            # Plot nodes
            for node in self.network.nodes.values():
                ax.plot(node['X'], node['Z'], 'co', markersize=6)
        
        ax.set_aspect('equal')
        ax.set_xlabel('X (Unity)')
        ax.set_ylabel('Z (Unity)')
        
        time_str = f" ({time_window[0]:.1f}s - {time_window[1]:.1f}s)" if time_window else ""
        ax.set_title(f'Traffic Density Heatmap{time_str}')
        
        plt.colorbar(im, ax=ax, label='Vehicle Count')
        
        return ax
    
    def plot_speed_heatmap(self, time_window=None, ax=None, overlay_network=True):
        """
        Plot average speed heatmap.
        
        Args:
            time_window: Tuple (start_time, end_time) or None for all time
            ax: Matplotlib axis (creates new if None)
            overlay_network: Whether to overlay road network
            
        Returns:
            Matplotlib axis
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 14))
        
        # Generate speed heatmap
        speed_grid, xedges, zedges = self.generate_speed_heatmap(time_window)
        
        # Plot heatmap (blue = slow, green = medium, red = fast)
        im = ax.imshow(speed_grid.T, origin='lower', cmap='RdYlGn', 
                      extent=[xedges[0], xedges[-1], zedges[0], zedges[-1]],
                      alpha=0.7, interpolation='bilinear')
        
        # Overlay road network
        if overlay_network:
            for road in self.network.roads.values():
                start_pos = self.network.get_node_position(road['StartNodeID'])
                end_pos = self.network.get_node_position(road['EndNodeID'])
                ax.plot([start_pos[0], end_pos[0]], 
                       [start_pos[1], end_pos[1]], 
                       'black', linewidth=1, alpha=0.3)
            
            for node in self.network.nodes.values():
                ax.plot(node['X'], node['Z'], 'ko', markersize=6)
        
        ax.set_aspect('equal')
        ax.set_xlabel('X (Unity)')
        ax.set_ylabel('Z (Unity)')
        
        time_str = f" ({time_window[0]:.1f}s - {time_window[1]:.1f}s)" if time_window else ""
        ax.set_title(f'Average Speed Heatmap{time_str}')
        
        plt.colorbar(im, ax=ax, label='Speed (units/s)')
        
        return ax
    
    def plot_combined_heatmaps(self, time_window=None, figsize=(18, 8)):
        """
        Plot density and speed heatmaps side by side.
        
        Args:
            time_window: Tuple (start_time, end_time) or None for all time
            figsize: Figure size tuple
            
        Returns:
            Figure and axes
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
        
        self.plot_density_heatmap(time_window, ax=ax1)
        self.plot_speed_heatmap(time_window, ax=ax2)
        
        plt.tight_layout()
        
        return fig, (ax1, ax2)
    
    def generate_time_series_plots(self, output_dir='./heatmaps', num_frames=10):
        """
        Generate a series of heatmaps over time (for animation or analysis).
        
        Args:
            output_dir: Directory to save image files
            num_frames: Number of time slices to generate
        """
        import os
        
        if self.vehicle_data is None:
            raise ValueError("No vehicle data loaded.")
        
        os.makedirs(output_dir, exist_ok=True)
        
        t_min = self.vehicle_data['time'].min()
        t_max = self.vehicle_data['time'].max()
        time_slices = np.linspace(t_min, t_max, num_frames + 1)
        
        for i in range(num_frames):
            time_window = (time_slices[i], time_slices[i+1])
            
            fig, (ax1, ax2) = self.plot_combined_heatmaps(time_window, figsize=(18, 8))
            
            filename = f'{output_dir}/heatmap_frame_{i:03d}.png'
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close(fig)
            
            print(f"Saved frame {i+1}/{num_frames}: {filename}")

