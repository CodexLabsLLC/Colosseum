"""

drone_map_level.py 

drone_map_level.py (Redesigned with Non-Blocking Coverage Visualization)
===============================================================================

Behavior:
- Fly in a heading for MoveDuration, then pick a new random safe heading.
- Build a 3D voxel occupancy grid from LIDAR.
- Collision avoidance uses BOTH:
  (1) live LIDAR forward distance
  (2) voxel-grid raycast clearance along current heading
- If predicted clearance <= ObstacleDistance (+ margin), switch to a new safe heading.
- Control loop sends motion commands at a fixed rate to prevent AirSim watchdog hover.
- Real-time 2D coverage visualization showing exploration progress (runs in separate thread).

Key constraints:
- ObstacleDistance=1.0m with Speed=5.0m/s is extremely tight. This script slows down
  as clearance shrinks to make that constraint attainable.

FIX: Visualization now runs in a separate thread to prevent matplotlib from blocking
the control loop, which was causing the drone to get stuck against walls.

Requirements:
- numpy, matplotlib
- AirSim PythonClient available on sys.path

Usage:
- Configure settings.json as you already do
- Run: python drone_map_environment.py
"""

import os
import sys
import time
import json
import math
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# Update this if needed
sys.path.insert(0, r'd:\repos\Colosseum\PythonClient')
import airsim  # noqa: E402


# ----------------------------
# Settings / Configuration
# ----------------------------

DEFAULT_SETTINGS = {
    "Resolution": 0.5,
    "FlightHeight": 3.0,
    "Speed": 5.0,
    "MoveDuration": 150.0,
    "ScanRate": 0.1,
    "ObstacleDistance": 1.0,
    "MapXRange": [-150, 150],
    "MapYRange": [-150, 150],
    "MapZFilter": [-25, 5],
    "Duration": 300
}

DEBUG_LEVEL = 2  # 0=minimal, 1=status, 2=detailed
VIZ_RATE = 0.5   # Visualization update rate in seconds


def debug_log(level: int, msg: str):
    if level <= DEBUG_LEVEL:
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")


def load_settings() -> dict:
    settings = DEFAULT_SETTINGS.copy()
    settings_path = os.path.join(os.path.expanduser("~"), "Documents", "AirSim", "settings.json")
    if os.path.exists(settings_path):
        try:
            with open(settings_path, "r") as f:
                all_settings = json.load(f)
            mapper = all_settings.get("DroneMapper", {})
            for k in DEFAULT_SETTINGS:
                if k in mapper:
                    settings[k] = mapper[k]
            print(f"Loaded DroneMapper settings from {settings_path}")
        except Exception as e:
            print(f"Error reading settings.json: {e}")
            print("Using default settings")
    else:
        print(f"settings.json not found at {settings_path}")
        print("Using default settings")
    return settings


def wrap_pi(a: float) -> float:
    """Wrap radians to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ----------------------------
# 2D Coverage Tracker
# ----------------------------

class CoverageTracker:
    """
    Tracks 2D exploration coverage at a specific Z slice (FlightHeight).
    Uses first_seen timestamps to show exploration progress.
    Double-buffered for thread-safe visualization.
    """

    def __init__(
        self,
        resolution: float,
        x_range: Tuple[float, float],
        y_range: Tuple[float, float],
        z_slice: float,
        z_tolerance: float = 2.0,
        max_age_display: float = 120.0
    ):
        self.res = float(resolution)
        self.inv = 1.0 / self.res

        self.x_min, self.x_max = float(x_range[0]), float(x_range[1])
        self.y_min, self.y_max = float(y_range[0]), float(y_range[1])
        self.z_slice = float(z_slice)
        self.z_tol = float(z_tolerance)
        self.max_age = float(max_age_display)

        self.nx = int(math.ceil((self.x_max - self.x_min) * self.inv))
        self.ny = int(math.ceil((self.y_max - self.y_min) * self.inv))

        # scan_duration[ix, iy] = total seconds this cell has been observed
        self.scan_duration = np.zeros((self.nx, self.ny), dtype=np.float32)

        # Double-buffered coverage image for thread-safe visualization
        self._img_front = np.full((self.nx, self.ny), np.nan, dtype=np.float32)
        self._img_back = np.full((self.nx, self.ny), np.nan, dtype=np.float32)
        self._img_lock = threading.Lock()

        # Statistics
        self.cells_explored = 0
        self.total_cells = self.nx * self.ny

        debug_log(1, f"CoverageTracker: {self.nx} x {self.ny} @ z={self.z_slice:.1f}m (±{self.z_tol:.1f}m)")

    def update(self, points_world: np.ndarray, dt: float):
        """
        Update scan_duration for points near the Z slice.
        Accumulates 'dt' for every cell hits by LIDAR this frame.
        """
        if points_world is None or points_world.size == 0:
            return

        x = points_world[:, 0]
        y = points_world[:, 1]
        z = points_world[:, 2]

        # Filter to Z slice and XY bounds
        mask = (
            (np.abs(z - self.z_slice) <= self.z_tol) &
            (x >= self.x_min) & (x < self.x_max) &
            (y >= self.y_min) & (y < self.y_max)
        )
        pts = points_world[mask]
        if pts.size == 0:
            return

        # Convert to grid indices
        ix = np.floor((pts[:, 0] - self.x_min) * self.inv).astype(np.int32)
        iy = np.floor((pts[:, 1] - self.y_min) * self.inv).astype(np.int32)

        # Clip to valid range
        ix = np.clip(ix, 0, self.nx - 1)
        iy = np.clip(iy, 0, self.ny - 1)

        # Flatten for efficient update
        lin = ix * self.ny + iy
        if lin.size > 0:
            # Use unique to avoid duplicate updates per frame for the same cell
            unique_lin = np.unique(lin)
            
            # Flatten array to update
            flat_duration = self.scan_duration.ravel()
            flat_duration[unique_lin] += dt
            
            # Count explored cells (non-zero duration)
            self.cells_explored = int(np.sum(self.scan_duration > 0))

    def render_image(self, elapsed_time: float):
        """
        Render the coverage image to the back buffer.
        Color = total scan duration.
        """
        # Create a copy to work with
        duration_map = self.scan_duration.copy()
        
        # Mark unseen cells as NaN for transparency/black background
        duration_map[duration_map == 0] = np.nan

        # Clamp for visualization consistency
        duration_map = np.clip(duration_map, 0, self.max_age)

        # Write to back buffer
        np.copyto(self._img_back, duration_map)

        # Swap buffers
        with self._img_lock:
            self._img_front, self._img_back = self._img_back, self._img_front

    def get_image(self) -> np.ndarray:
        """
        Get a copy of the front buffer for visualization.
        Thread-safe, O(1) reference + copy.
        """
        with self._img_lock:
            return self._img_front.copy()

    def get_stats(self) -> dict:
        pct = 100.0 * self.cells_explored / self.total_cells if self.total_cells > 0 else 0.0
        return {
            "cells_explored": self.cells_explored,
            "total_cells": self.total_cells,
            "coverage_pct": pct
        }


# ----------------------------
# Visualization Manager (Non-Blocking)
# ----------------------------

@dataclass
class VizUpdate:
    """Data structure for visualization updates."""
    drone_x: float
    drone_y: float
    drone_yaw: float
    elapsed: float


class CoverageVisualizer:
    """
    Matplotlib-based real-time visualization of coverage map.
    
    FIX: All matplotlib operations now run in a dedicated thread to prevent
    blocking the main control loop. The main loop only queues lightweight
    update data, and the viz thread handles all rendering.
    """

    def __init__(
        self,
        coverage: CoverageTracker,
        x_range: Tuple[float, float],
        y_range: Tuple[float, float],
        update_rate: float = 0.5
    ):
        self.coverage = coverage
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.update_rate = update_rate
        self.last_update = 0.0

        # Thread synchronization
        self._update_queue = queue.Queue(maxsize=2)  # Small queue, drop old frames
        self._running = True
        self._viz_thread = None
        self._initialized = threading.Event()

        # Matplotlib objects (created in viz thread)
        self.fig = None
        self.ax = None
        self.im = None
        self.drone_marker = None
        self.drone_heading = None
        self.stats_text = None
        self.cmap = None

    def start(self):
        """Start the visualization thread. Call this after construction."""
        self._viz_thread = threading.Thread(target=self._viz_loop, daemon=True, name="VizThread")
        self._viz_thread.start()
        # Wait for matplotlib initialization to complete
        self._initialized.wait(timeout=5.0)
        debug_log(1, "Visualization thread started")

    def _create_figure(self):
        """Create matplotlib figure. Must be called from viz thread."""
        # Create custom colormap: Black (unseen) -> Green -> Yellow -> Red (highly scanned)
        colors = [
            (0.0, 0.8, 0.0),   # Green (low duration)
            (1.0, 1.0, 0.0),   # Yellow (medium duration)
            (1.0, 0.0, 0.0),   # Red (high duration)
            (0.5, 0.0, 0.0),   # Dark Red (very high duration)
        ]
        self.cmap = LinearSegmentedColormap.from_list('scan_duration', colors, N=256)
        self.cmap.set_bad(color='black')  # NaN (unseen) = black

        # Setup figure
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Drone Coverage Map')

        # Initial empty image
        empty = np.full((self.coverage.nx, self.coverage.ny), np.nan, dtype=np.float32)
        self.im = self.ax.imshow(
            empty.T,
            origin='lower',
            extent=[self.x_min, self.x_max, self.y_min, self.y_max],
            cmap=self.cmap,
            vmin=0,
            vmax=self.coverage.max_age,
            aspect='equal'
        )

        # Colorbar
        cbar = self.fig.colorbar(self.im, ax=self.ax, shrink=0.8)
        cbar.set_label('Scan Duration (seconds)')

        # Drone position marker
        self.drone_marker, = self.ax.plot([], [], 'r^', markersize=12, markeredgecolor='white', markeredgewidth=2)
        self.drone_heading, = self.ax.plot([], [], 'r-', linewidth=2)

        # Labels
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Exploration Coverage Map')
        self.ax.grid(True, alpha=0.3)

        # Stats text
        self.stats_text = self.ax.text(
            0.02, 0.98, '',
            transform=self.ax.transAxes,
            verticalalignment='top',
            fontsize=10,
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7),
            color='white'
        )

        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _viz_loop(self):
        """
        Background thread that handles all matplotlib updates.
        This keeps the main control loop responsive.
        """
        # Create figure in this thread (matplotlib is not thread-safe for creation)
        try:
            self._create_figure()
        except Exception as e:
            debug_log(0, f"[VIZ] Failed to create figure: {e}")
            self._initialized.set()
            return

        self._initialized.set()

        while self._running:
            try:
                # Wait for update data with timeout (allows clean shutdown)
                try:
                    update = self._update_queue.get(timeout=0.1)
                except queue.Empty:
                    # No update pending, just pump events to keep window responsive
                    if self.fig:
                        self.fig.canvas.flush_events()
                    continue

                # Process the update
                self._render_frame(update)

            except Exception as e:
                debug_log(1, f"[VIZ] Render error: {e}")

        # Cleanup
        if self.fig:
            plt.close(self.fig)

    def _render_frame(self, update: VizUpdate):
        """Render a single frame. Called from viz thread only."""
        if not self.fig:
            return

        # Get pre-rendered coverage image from tracker
        img = self.coverage.get_image()

        # Update image data (transpose for correct orientation)
        self.im.set_data(img.T)

        # Update drone position marker
        self.drone_marker.set_data([update.drone_x], [update.drone_y])

        # Heading line (3m ahead)
        hx = update.drone_x + 3.0 * math.cos(update.drone_yaw)
        hy = update.drone_y + 3.0 * math.sin(update.drone_yaw)
        self.drone_heading.set_data([update.drone_x, hx], [update.drone_y, hy])

        # Update stats
        stats = self.coverage.get_stats()
        self.stats_text.set_text(
            f"Elapsed: {update.elapsed:.1f}s\n"
            f"Coverage: {stats['coverage_pct']:.1f}%\n"
            f"Cells: {stats['cells_explored']:,} / {stats['total_cells']:,}"
        )

        # Redraw (these blocking calls now happen in viz thread, not control loop)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def should_update(self, now: float) -> bool:
        """O(1) check if visualization should be updated."""
        return (now - self.last_update) >= self.update_rate

    def update(self, drone_x: float, drone_y: float, drone_yaw: float, elapsed: float):
        """
        Queue a visualization update. NON-BLOCKING.
        
        This is the key fix: instead of doing matplotlib operations here,
        we just queue the data and return immediately. The viz thread
        handles the actual rendering.
        """
        self.last_update = time.time()

        update = VizUpdate(
            drone_x=drone_x,
            drone_y=drone_y,
            drone_yaw=drone_yaw,
            elapsed=elapsed
        )

        # Non-blocking put: if queue is full, drop the oldest frame
        try:
            # Try to clear old frames if queue is getting full
            while self._update_queue.qsize() >= 2:
                try:
                    self._update_queue.get_nowait()
                except queue.Empty:
                    break
            self._update_queue.put_nowait(update)
        except queue.Full:
            pass  # Drop frame rather than block

    def close(self):
        """Shutdown the visualization thread cleanly."""
        self._running = False
        if self._viz_thread and self._viz_thread.is_alive():
            self._viz_thread.join(timeout=2.0)
        debug_log(1, "Visualization thread stopped")


# ----------------------------
# 3D Voxel Occupancy Grid
# ----------------------------

class VoxelGrid3D:
    """
    3D voxel grid storing hit counts.
    Occupied if hits >= OCCUPIED_HITS.
    """

    def __init__(self, resolution: float,
                 x_range: Tuple[float, float],
                 y_range: Tuple[float, float],
                 z_range: Tuple[float, float]):
        self.res = float(resolution)
        self.inv = 1.0 / self.res

        self.x_min, self.x_max = float(x_range[0]), float(x_range[1])
        self.y_min, self.y_max = float(y_range[0]), float(y_range[1])
        self.z_min, self.z_max = float(z_range[0]), float(z_range[1])

        # Use ceil so max boundary is representable
        self.nx = int(math.ceil((self.x_max - self.x_min) * self.inv))
        self.ny = int(math.ceil((self.y_max - self.y_min) * self.inv))
        self.nz = int(math.ceil((self.z_max - self.z_min) * self.inv))

        # uint16 is enough for hit counts and halves memory vs int32
        self.hits = np.zeros((self.nx, self.ny, self.nz), dtype=np.uint16)

        self._lock = threading.Lock()

        debug_log(1, f"VoxelGrid: {self.nx} x {self.ny} x {self.nz} @ {self.res}m")
        debug_log(1, f"  X[{self.x_min}, {self.x_max}] Y[{self.y_min}, {self.y_max}] Z[{self.z_min}, {self.z_max}]")

    def world_to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        ix = int(math.floor((x - self.x_min) * self.inv))
        iy = int(math.floor((y - self.y_min) * self.inv))
        iz = int(math.floor((z - self.z_min) * self.inv))
        return ix, iy, iz

    def in_bounds(self, ix: int, iy: int, iz: int) -> bool:
        return (0 <= ix < self.nx) and (0 <= iy < self.ny) and (0 <= iz < self.nz)

    def update_hits(self, points_world: np.ndarray):
        """
        Vectorized endpoint hit insertion.
        points_world: Nx3 in world/NED.
        """
        if points_world is None or points_world.size == 0:
            return

        # Filter within map bounds first (world)
        x = points_world[:, 0]
        y = points_world[:, 1]
        z = points_world[:, 2]
        m = (
            (x >= self.x_min) & (x < self.x_max) &
            (y >= self.y_min) & (y < self.y_max) &
            (z >= self.z_min) & (z < self.z_max)
        )
        pts = points_world[m]
        if pts.size == 0:
            return

        # Convert to indices
        ix = np.floor((pts[:, 0] - self.x_min) * self.inv).astype(np.int32)
        iy = np.floor((pts[:, 1] - self.y_min) * self.inv).astype(np.int32)
        iz = np.floor((pts[:, 2] - self.z_min) * self.inv).astype(np.int32)

        # Safe clip (should already be in bounds)
        ix = np.clip(ix, 0, self.nx - 1)
        iy = np.clip(iy, 0, self.ny - 1)
        iz = np.clip(iz, 0, self.nz - 1)

        # Increment hits (cap to avoid overflow)
        with self._lock:
            # Flatten indexing for speed
            flat = self.hits.ravel()
            lin = (ix * (self.ny * self.nz)) + (iy * self.nz) + iz
            np.add.at(flat, lin, 1)
            # Cap
            np.minimum(flat, np.uint16(65530), out=flat)

    def distance_to_boundary(self, x: float, y: float, yaw: float, max_dist: float) -> float:
        """
        Compute distance to map boundary in the given heading direction.
        Returns max_dist if boundary is not reached within that distance.
        Uses a fence margin to keep the drone inside the operational area.
        """
        fence = 1.0  # Stay this far from the edge
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        
        # Calculate parametric distances to each boundary
        # t represents how far along the heading direction until we hit fence
        t_x = float('inf')
        t_y = float('inf')
        
        # X boundary check
        if dx > 0.001:
            # Moving toward x_max
            t_x = (self.x_max - fence - x) / dx
        elif dx < -0.001:
            # Moving toward x_min
            t_x = (self.x_min + fence - x) / dx
        
        # Y boundary check
        if dy > 0.001:
            # Moving toward y_max
            t_y = (self.y_max - fence - y) / dy
        elif dy < -0.001:
            # Moving toward y_min
            t_y = (self.y_min + fence - y) / dy
        
        # Take the closest boundary hit
        t = min(t_x, t_y)
        
        if t < 0:
            # Already past the fence in this direction - return small positive
            # to indicate we need to move away
            return 0.1
        
        return min(t, max_dist)

    def raycast_clearance(
        self,
        pos: Tuple[float, float, float],
        yaw: float,
        max_dist: float,
        corridor_radius: float,
        z_radius: float,
        occupied_hits: int,
        step: Optional[float] = None
    ) -> float:
        """
        Return distance to first occupied voxel or boundary along heading, else max_dist.
        Samples a small corridor (lateral + vertical offsets) to model drone volume.

        This is used for collision prediction using the voxel map.
        
        FIX: No longer returns 0.0 immediately when at boundary. Instead, computes
        the actual clearance in the given direction, allowing escape routes away
        from the boundary to be found.
        """
        x0, y0, z0 = pos
        fence = 1.0

        if step is None:
            step = max(0.25, self.res * 0.5)

        # First, compute distance to boundary in this direction
        boundary_dist = self.distance_to_boundary(x0, y0, yaw, max_dist)

        # forward direction in world (NED)
        dx = math.cos(yaw)
        dy = math.sin(yaw)

        # perpendicular (right)
        rx = math.cos(yaw + math.pi / 2)
        ry = math.sin(yaw + math.pi / 2)

        lateral_offsets = [-corridor_radius, 0.0, corridor_radius]
        z_offsets = [-z_radius, 0.0, z_radius]

        n_steps = int(max_dist / step)
        # Start a little ahead to avoid self/noise
        start = 0.5
        
        voxel_dist = max_dist  # Default: no voxel obstruction

        with self._lock:
            for i in range(n_steps):
                d = start + i * step
                if d > max_dist:
                    break
                
                # Stop early if we've already found a closer boundary
                if d >= boundary_dist:
                    break

                cx = x0 + d * dx
                cy = y0 + d * dy

                for lo in lateral_offsets:
                    ox = cx + lo * rx
                    oy = cy + lo * ry
                    for zo in z_offsets:
                        oz = z0 + zo
                        ix, iy, iz = self.world_to_grid(ox, oy, oz)
                        if self.in_bounds(ix, iy, iz):
                            if self.hits[ix, iy, iz] >= occupied_hits:
                                voxel_dist = d
                                break
                    if voxel_dist < max_dist:
                        break
                if voxel_dist < max_dist:
                    break

        # Return the minimum of voxel obstruction and boundary distance
        return min(voxel_dist, boundary_dist)

    def stats(self, occupied_hits: int) -> dict:
        with self._lock:
            occ = int(np.sum(self.hits >= occupied_hits))
            total_hits = int(np.sum(self.hits))
        return {"occupied_voxels": occ, "total_hits": total_hits}



# ----------------------------
# LIDAR Thread (read + map update + sector clearance + coverage)
# ----------------------------

@dataclass
class LidarSummary:
    timestamp: float = 0.0
    valid: bool = False
    # Vehicle pose at time of scan
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    # Obstacle info from this scan
    front_dist: float = 999.0
    sector_min: Optional[np.ndarray] = None  # shape (n_sectors,)
    n_points: int = 0


class LidarWorker(threading.Thread):
    """
    Continuously reads LIDAR + state and updates voxel grid.
    Also computes:
    - front distance in a narrow corridor
    - per-sector minimum ranges for escape direction selection
    - 2D coverage map updates
    """

    def __init__(
        self,
        grid: VoxelGrid3D,
        coverage: CoverageTracker,
        scan_rate: float,
        detection_range: float,
        n_sectors: int,
        z_tol: float,
        front_half_width: float,
        viz_rate: float = 0.5,
        sensor_name: str = "LidarSensor1",
    ):
        super().__init__(daemon=True)
        self.grid = grid
        self.coverage = coverage
        self.scan_rate = float(scan_rate)
        self.detection_range = float(detection_range)
        self.n_sectors = int(n_sectors)
        self.z_tol = float(z_tol)
        self.front_half_width = float(front_half_width)
        self.viz_rate = float(viz_rate)
        self.sensor_name = sensor_name

        self._lock = threading.Lock()
        self._latest = LidarSummary(sector_min=np.full(self.n_sectors, np.inf, dtype=np.float32))
        self._running = False

        self.start_time = 0.0
        self.last_viz_render = 0.0

        self.client = None  # created in run

    def get_latest(self) -> LidarSummary:
        with self._lock:
            # Shallow copy is fine; sector_min is immutable enough for read
            s = self._latest
            out = LidarSummary(
                timestamp=s.timestamp, valid=s.valid,
                x=s.x, y=s.y, z=s.z, yaw=s.yaw,
                front_dist=s.front_dist,
                sector_min=None if s.sector_min is None else s.sector_min.copy(),
                n_points=s.n_points
            )
        return out

    def stop(self):
        self._running = False

    def run(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self._running = True
        self.start_time = time.time()
        self.last_viz_render = self.start_time
        next_t = time.time()

        while self._running:
            now = time.time()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
                continue
            next_t = now + self.scan_rate

            try:
                state = self.client.getMultirotorState()
                p = state.kinematics_estimated.position
                o = state.kinematics_estimated.orientation
                _, _, yaw = airsim.to_eularian_angles(o)
                x, y, z = p.x_val, p.y_val, p.z_val

                data = self.client.getLidarData(self.sensor_name)
                pc = data.point_cloud
                if pc is None or len(pc) < 3:
                    with self._lock:
                        self._latest.timestamp = now
                        self._latest.valid = False
                        self._latest.n_points = 0
                    continue

                pts = np.array(pc, dtype=np.float32).reshape(-1, 3)
                n_pts = int(pts.shape[0])

                elapsed = now - self.start_time

                # Update voxel grid (endpoint hits)
                self.grid.update_hits(pts)

                # Update coverage tracker (accumulate duration)
                # Pass scan_rate as the time delta for this frame
                self.coverage.update(pts, self.scan_rate)

                # Periodically render coverage image for visualization
                if (now - self.last_viz_render) >= self.viz_rate:
                    self.coverage.render_image(elapsed)
                    self.last_viz_render = now

                # Compute clearance summaries in vehicle body frame
                dx = pts[:, 0] - x
                dy = pts[:, 1] - y
                dz = pts[:, 2] - z

                # Only consider near-altitude band for obstacle avoidance
                m = np.abs(dz) <= self.z_tol
                dx = dx[m]
                dy = dy[m]
                dz = dz[m]  # kept for completeness
                if dx.size == 0:
                    front_dist = 999.0
                    sector_min = np.full(self.n_sectors, self.detection_range, dtype=np.float32)
                else:
                    cy = math.cos(-yaw)
                    sy = math.sin(-yaw)
                    xb = dx * cy - dy * sy  # forward
                    yb = dx * sy + dy * cy  # right
                    r = np.sqrt(xb * xb + yb * yb)

                    # Ignore self/noise close returns
                    valid = r >= 0.7
                    xb = xb[valid]
                    yb = yb[valid]
                    r = r[valid]

                    # Front distance in a narrow corridor
                    front_mask = (xb > 0.3) & (xb <= self.detection_range) & (np.abs(yb) <= self.front_half_width)
                    if np.any(front_mask):
                        front_dist = float(np.min(xb[front_mask]))
                    else:
                        front_dist = 999.0

                    # Sector minimum ranges
                    bearing = np.arctan2(yb, xb)  # [-pi, pi], 0=forward
                    edges = np.linspace(-math.pi, math.pi, self.n_sectors + 1, dtype=np.float32)
                    idx = np.searchsorted(edges, bearing, side="right") - 1
                    idx = np.clip(idx, 0, self.n_sectors - 1)

                    sector_min = np.full(self.n_sectors, np.inf, dtype=np.float32)
                    np.minimum.at(sector_min, idx.astype(np.int32), r.astype(np.float32))

                    # If a sector has no returns, treat as open to detection_range
                    sector_min[~np.isfinite(sector_min)] = self.detection_range
                    # Clip
                    sector_min = np.minimum(sector_min, self.detection_range).astype(np.float32)

                with self._lock:
                    self._latest = LidarSummary(
                        timestamp=now,
                        valid=True,
                        x=x, y=y, z=z, yaw=yaw,
                        front_dist=front_dist,
                        sector_min=sector_min,
                        n_points=n_pts
                    )

            except Exception as e:
                debug_log(1, f"[LIDAR] Error: {e}")
                with self._lock:
                    self._latest.timestamp = time.time()
                    self._latest.valid = False
                    self._latest.n_points = 0


# ----------------------------
# Exploration Controller
# ----------------------------

class ExplorerState:
    MOVE = "MOVE"
    BACKUP = "BACKUP"
    TURN = "TURN"


class DroneExplorer:
    def __init__(self, settings: dict):
        self.res = float(settings["Resolution"])
        self.flight_z = -abs(float(settings["FlightHeight"]))  # NED (down is +); altitude is negative
        self.flight_height = abs(float(settings["FlightHeight"]))
        self.speed = float(settings["Speed"])
        self.move_duration = float(settings["MoveDuration"])
        self.scan_rate = float(settings["ScanRate"])
        self.obstacle_dist = float(settings["ObstacleDistance"])
        self.duration = float(settings["Duration"])

        self.x_range = tuple(settings["MapXRange"])
        self.y_range = tuple(settings["MapYRange"])
        self.z_range = tuple(settings["MapZFilter"])

        self.grid = VoxelGrid3D(
            resolution=self.res,
            x_range=self.x_range,
            y_range=self.y_range,
            z_range=self.z_range
        )

        # Coverage tracker for visualization (slice at flight height)
        # In NED, flight_z is negative, so the slice is at -flight_z in world Z
        self.coverage = CoverageTracker(
            resolution=self.res,
            x_range=self.x_range,
            y_range=self.y_range,
            z_slice=self.flight_z,  # NED coordinates
            z_tolerance=2.0,
            max_age_display=120.0
        )

        # Visualization (created in connect())
        self.visualizer = None

        # Collision checking model (rough drone volume)
        self.corridor_radius = 0.6  # meters lateral radius sampled in raycast
        self.z_radius = 0.6         # meters vertical sampling
        self.OCCUPIED_HITS = 8      # how many hits to consider "solid obstacle" in voxel map

        # Control cadence
        self.ctrl_hz = 20.0
        self.ctrl_dt = 1.0 / self.ctrl_hz
        self.cmd_duration = 0.25  # seconds: tolerate short stalls without watchdog

        # Turning control
        self.yaw_kp = 2.5          # deg/s per deg error (bounded)
        self.max_yaw_rate = 90.0   # deg/s
        self.yaw_tol_deg = 5.0

        # Detection: must be >= speed * reaction_time + margin
        reaction_time = 0.6
        self.detection_range = max(5.0, self.speed * reaction_time + 2.0, self.obstacle_dist * 3.0)

        # Preemptive threshold: switch direction before reaching obstacle_dist
        self.safety_margin = max(0.3, self.res)  # meters

        # Backup behavior (only if too close)
        self.backup_speed = 2.0
        self.backup_time = 0.5

        # LIDAR sectoring used to pick new headings
        self.n_sectors = 36  # 10 deg bins
        self.lidar_z_tol = 2.0
        self.front_half_width = 0.8

        self.client = None
        self.lidar_worker = None

        # Smoothed velocity commands to avoid "jerk"
        self.vx_s = 0.0
        self.vy_s = 0.0
        self.vel_smooth_alpha = 0.6  # closer to 1 => more smoothing

        # Heading control
        self.heading_yaw = 0.0       # radians
        self.target_yaw = 0.0        # radians

        # State
        self.state = ExplorerState.MOVE
        self.segment_end = 0.0
        self.backup_end = 0.0
        self.turn_timeout = 0.0

        # Counters
        self.avoid_count = 0
        self.scan_count = 0

    def connect(self):
        debug_log(1, "Connecting...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        debug_log(1, "Taking off...")
        self.client.takeoffAsync().join()
        time.sleep(0.3)

        debug_log(1, f"Climb to {abs(self.flight_z):.1f}m...")
        self.client.moveToZAsync(self.flight_z, 3.0).join()
        time.sleep(0.3)

        # Start LIDAR worker (separate client, read-only)
        self.lidar_worker = LidarWorker(
            grid=self.grid,
            coverage=self.coverage,
            scan_rate=self.scan_rate,
            detection_range=self.detection_range,
            n_sectors=self.n_sectors,
            z_tol=self.lidar_z_tol,
            front_half_width=self.front_half_width,
            viz_rate=VIZ_RATE
        )
        self.lidar_worker.start_time = time.time()  # Sync start time
        self.lidar_worker.start()

        # Initialize visualization in separate thread (non-blocking)
        self.visualizer = CoverageVisualizer(
            coverage=self.coverage,
            x_range=self.x_range,
            y_range=self.y_range,
            update_rate=VIZ_RATE
        )
        self.visualizer.start()  # Starts the viz thread

        # Initialize heading randomly
        x, y, z, yaw = self.get_state()
        self.heading_yaw = yaw
        self.pick_new_heading(yaw, prefer_random=True)

        now = time.time()
        self.segment_end = now + self.move_duration

        debug_log(1, f"Ready. Speed={self.speed:.1f}m/s MoveDuration={self.move_duration:.1f}s "
                     f"ObstacleDistance={self.obstacle_dist:.1f}m DetectRange={self.detection_range:.1f}m")

    def get_state(self) -> Tuple[float, float, float, float]:
        s = self.client.getMultirotorState()
        p = s.kinematics_estimated.position
        o = s.kinematics_estimated.orientation
        _, _, yaw = airsim.to_eularian_angles(o)
        return p.x_val, p.y_val, p.z_val, yaw

    def command_velocity_yaw(self, vx: float, vy: float, yaw_mode: airsim.YawMode):
        # No join: keep the control loop real-time
        self.client.moveByVelocityZAsync(vx, vy, self.flight_z, duration=self.cmd_duration, yaw_mode=yaw_mode)

    def pick_new_heading(self, current_yaw: float, prefer_random: bool = False):
        """
        Choose a new heading:
        - primary: pick best sector using LIDAR sector mins + map raycast
        - fallback: random heading
        
        FIX: Added center-bias when near boundaries to help escape edges.
        """
        x, y, z, yaw = self.get_state()
        lid = self.lidar_worker.get_latest() if self.lidar_worker else LidarSummary(valid=False)

        # Compute center-bias parameters for boundary escape
        center_x = (self.grid.x_min + self.grid.x_max) / 2.0
        center_y = (self.grid.y_min + self.grid.y_max) / 2.0
        to_center_angle = math.atan2(center_y - y, center_x - x)
        
        # Check if we're in the boundary danger zone (within 5m of any edge)
        boundary_margin = 5.0
        in_boundary_zone = (
            x <= self.grid.x_min + boundary_margin or 
            x >= self.grid.x_max - boundary_margin or
            y <= self.grid.y_min + boundary_margin or 
            y >= self.grid.y_max - boundary_margin
        )
        
        if in_boundary_zone:
            debug_log(2, f"Near boundary at ({x:.1f},{y:.1f}) - applying center bias toward {math.degrees(to_center_angle):.1f}°")

        if (not prefer_random) and lid.valid and (lid.sector_min is not None):
            edges = np.linspace(-math.pi, math.pi, self.n_sectors + 1, dtype=np.float32)
            centers = (edges[:-1] + edges[1:]) * 0.5  # relative angles

            best_score = -1e9
            best_abs = None

            # Require at least this much space to consider a direction
            min_clear = max(self.obstacle_dist + 1.0, 2.0)

            for i in range(self.n_sectors):
                rel = float(centers[i])
                abs_y = wrap_pi(current_yaw + rel)

                lidar_clear = float(lid.sector_min[i])

                # Quick reject: LIDAR already says it's tight
                if lidar_clear < min_clear:
                    continue

                map_clear = self.grid.raycast_clearance(
                    pos=(x, y, z),
                    yaw=abs_y,
                    max_dist=self.detection_range,
                    corridor_radius=self.corridor_radius,
                    z_radius=self.z_radius,
                    occupied_hits=self.OCCUPIED_HITS,
                )

                clearance = min(lidar_clear, map_clear)

                if clearance < min_clear:
                    continue

                # Penalize tiny changes so it doesn't jitter between nearby headings
                turn = abs(wrap_pi(abs_y - current_yaw))
                score = clearance - 0.8 * turn  # turn penalty in radians
                
                # FIX: Add center-bias when near boundary
                if in_boundary_zone:
                    # Bonus for heading toward center (max ~3.0 when pointing directly at center)
                    angle_diff = abs(wrap_pi(abs_y - to_center_angle))
                    center_bonus = 3.0 * (1.0 - angle_diff / math.pi)
                    score += center_bonus

                if score > best_score:
                    best_score = score
                    best_abs = abs_y

            if best_abs is not None:
                self.target_yaw = best_abs
                return

        # Fallback: when near boundary, prefer heading toward center
        if in_boundary_zone:
            # Random perturbation around the center direction (+/- 45°)
            delta = np.random.uniform(-math.radians(45), math.radians(45))
            self.target_yaw = wrap_pi(to_center_angle + delta)
            debug_log(2, f"Fallback: heading toward center with delta {math.degrees(delta):.1f}°")
        else:
            # Normal fallback: random heading, but avoid extremely small turns
            delta = np.random.uniform(math.radians(60), math.radians(160))
            if np.random.rand() < 0.5:
                delta *= -1.0
            self.target_yaw = wrap_pi(current_yaw + delta)


    def compute_clearance(self, x: float, y: float, z: float, yaw: float) -> Tuple[float, float, float]:
        """
        Returns (front_lidar_dist, map_dist, combined_dist)
        """
        lid = self.lidar_worker.get_latest() if self.lidar_worker else LidarSummary(valid=False)
        front = float(lid.front_dist) if (lid.valid) else 999.0

        map_d = self.grid.raycast_clearance(
            pos=(x, y, z),
            yaw=yaw,
            max_dist=self.detection_range,
            corridor_radius=self.corridor_radius,
            z_radius=self.z_radius,
            occupied_hits=self.OCCUPIED_HITS,
        )

        return front, map_d, min(front, map_d)

    def run(self):
        self.connect()
        t0 = time.time()
        last_status = t0

        try:
            while True:
                now = time.time()
                elapsed = now - t0
                if elapsed >= self.duration:
                    break

                loop_start = now

                x, y, z, yaw = self.get_state()

                # Decide clearance along current heading (not necessarily current yaw if we are mid-turn)
                # - In MOVE, we fly along heading_yaw
                # - In TURN, we rotate toward target_yaw
                # - In BACKUP, we keep current yaw but reverse velocity
                if self.state == ExplorerState.MOVE:
                    check_yaw = self.heading_yaw
                else:
                    check_yaw = yaw

                front, map_d, dist = self.compute_clearance(x, y, z, check_yaw)

                # Preemptive avoidance trigger
                trigger_dist = self.obstacle_dist + self.safety_margin
                too_close = dist <= trigger_dist

                # Periodic status
                if (now - last_status) >= 2.0:
                    st = self.grid.stats(self.OCCUPIED_HITS)
                    cov = self.coverage.get_stats()
                    debug_log(
                        1,
                        f"t={elapsed:5.1f}s state={self.state} pos=({x:7.1f},{y:7.1f},{z:5.1f}) "
                        f"yaw={math.degrees(yaw):6.1f}° head={math.degrees(self.heading_yaw):6.1f}° "
                        f"lidar={front:4.1f}m map={map_d:4.1f}m min={dist:4.1f}m "
                        f"avoid={self.avoid_count} vox={st['occupied_voxels']:,} cov={cov['coverage_pct']:.1f}%"
                    )
                    last_status = now

                # Visualization update - NOW NON-BLOCKING
                # This just queues data for the viz thread, returns immediately
                if self.visualizer and self.visualizer.should_update(now):
                    self.visualizer.update(x, y, yaw, elapsed)

                # State transitions
                if self.state == ExplorerState.MOVE:
                    # Switch direction on MoveDuration
                    if now >= self.segment_end:
                        self.pick_new_heading(self.heading_yaw, prefer_random=False)
                        self.state = ExplorerState.TURN
                        self.turn_timeout = now + 5.0  # hard limit
                        debug_log(2, f"MoveDuration reached -> TURN to {math.degrees(self.target_yaw):.1f}°")

                    # Or switch direction on obstacle distance threshold
                    elif too_close:
                        self.avoid_count += 1
                        debug_log(2, f"Obstacle trigger at {dist:.2f}m (lidar={front:.2f}, map={map_d:.2f}) -> AVOID")

                        # If very close, do brief backup first to create turn space
                        if dist < max(0.6, 0.6 * self.obstacle_dist + 0.2):
                            self.state = ExplorerState.BACKUP
                            self.backup_end = now + self.backup_time
                        else:
                            self.pick_new_heading(self.heading_yaw, prefer_random=False)
                            self.state = ExplorerState.TURN
                            self.turn_timeout = now + 5.0

                elif self.state == ExplorerState.BACKUP:
                    if now >= self.backup_end:
                        self.pick_new_heading(yaw, prefer_random=False)
                        self.state = ExplorerState.TURN
                        self.turn_timeout = now + 5.0
                        debug_log(2, f"Backup complete -> TURN to {math.degrees(self.target_yaw):.1f}°")

                elif self.state == ExplorerState.TURN:
                    err = wrap_pi(self.target_yaw - yaw)
                    err_deg = math.degrees(err)

                    if abs(err_deg) <= self.yaw_tol_deg:
                        # Commit new heading and resume MOVE
                        self.heading_yaw = self.target_yaw
                        self.segment_end = now + self.move_duration
                        self.state = ExplorerState.MOVE
                        debug_log(2, f"Turn complete -> MOVE heading={math.degrees(self.heading_yaw):.1f}°")

                    elif now >= self.turn_timeout:
                        # Give up and move anyway (prevents deadlock)
                        self.heading_yaw = self.target_yaw
                        self.segment_end = now + self.move_duration
                        self.state = ExplorerState.MOVE
                        debug_log(1, f"Turn timeout -> MOVE heading={math.degrees(self.heading_yaw):.1f}°")

                # Command generation (single source of truth)
                if self.state == ExplorerState.MOVE:
                    # Slow down near obstacles so ObstacleDistance is achievable
                    slow_band = max(2.0, self.detection_range * 0.4)
                    scale = clamp((dist - self.obstacle_dist) / slow_band, 0.2, 1.0)
                    spd = self.speed * scale

                    vx = spd * math.cos(self.heading_yaw)
                    vy = spd * math.sin(self.heading_yaw)

                    # Smooth velocity to remove jerk
                    a = self.vel_smooth_alpha
                    self.vx_s = a * self.vx_s + (1 - a) * vx
                    self.vy_s = a * self.vy_s + (1 - a) * vy

                    # Yaw to face heading (absolute yaw in degrees)
                    yaw_deg = (math.degrees(self.heading_yaw) + 360.0) % 360.0
                    self.command_velocity_yaw(
                        self.vx_s, self.vy_s,
                        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw_deg)
                    )

                elif self.state == ExplorerState.BACKUP:
                    # Reverse along current yaw
                    vx = -self.backup_speed * math.cos(yaw)
                    vy = -self.backup_speed * math.sin(yaw)
                    self.vx_s = 0.0
                    self.vy_s = 0.0
                    yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
                    self.command_velocity_yaw(
                        vx, vy,
                        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw_deg)
                    )

                elif self.state == ExplorerState.TURN:
                    # Turn in place using yaw rate, hold position (vx=vy=0)
                    err = wrap_pi(self.target_yaw - yaw)
                    err_deg = math.degrees(err)
                    yaw_rate = clamp(self.yaw_kp * err_deg, -self.max_yaw_rate, self.max_yaw_rate)
                    self.vx_s = 0.0
                    self.vy_s = 0.0
                    self.command_velocity_yaw(
                        0.0, 0.0,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
                    )

                # Maintain loop cadence
                dt = time.time() - loop_start
                sleep_t = self.ctrl_dt - dt
                if sleep_t > 0:
                    time.sleep(sleep_t)

        finally:
            self.shutdown()

    def shutdown(self):
        debug_log(1, "Shutting down...")

        if self.lidar_worker:
            self.lidar_worker.stop()

        if self.visualizer:
            self.visualizer.close()

        # Save map
        np.save("lidar_map.npy", self.grid.hits)
        meta = {
            "resolution": self.grid.res,
            "x_range": [self.grid.x_min, self.grid.x_max],
            "y_range": [self.grid.y_min, self.grid.y_max],
            "z_range": [self.grid.z_min, self.grid.z_max],
            "shape": [self.grid.nx, self.grid.ny, self.grid.nz],
            "occupied_hits": self.OCCUPIED_HITS
        }
        with open("lidar_map_metadata.json", "w") as f:
            json.dump(meta, f, indent=2)

        # Save coverage data
        np.save("coverage_first_seen.npy", self.coverage.first_seen)
        cov_meta = {
            "resolution": self.coverage.res,
            "x_range": [self.coverage.x_min, self.coverage.x_max],
            "y_range": [self.coverage.y_min, self.coverage.y_max],
            "z_slice": self.coverage.z_slice,
            "shape": [self.coverage.nx, self.coverage.ny],
            "cells_explored": self.coverage.cells_explored,
            "total_cells": self.coverage.total_cells
        }
        with open("coverage_metadata.json", "w") as f:
            json.dump(cov_meta, f, indent=2)

        # Land
        try:
            self.client.hoverAsync()
            time.sleep(0.2)
            self.client.landAsync().join()
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:
            pass

        debug_log(1, "Done.")


def main():
    print("=" * 60)
    print("DRONE LIDAR VOXEL MAPPER (with Non-Blocking Visualization)")
    print("=" * 60)

    settings = load_settings()

    print("\nConfiguration:")
    for k in ["Resolution", "FlightHeight", "Speed", "MoveDuration", "ScanRate",
              "ObstacleDistance", "MapXRange", "MapYRange", "MapZFilter", "Duration"]:
        print(f"  {k}: {settings[k]}")

    explorer = DroneExplorer(settings)
    explorer.run()


if __name__ == "__main__":
    main()
