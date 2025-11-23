#!/usr/bin/env python3
"""
Costmap Processor - Extracts and analyzes costmap data for decision explanation.

Week 3: Core analysis component for understanding navigation environment.

Key concepts:
- OccupancyGrid: 2D array where 0=free, 100=unknown, 253=inscribed, 254=lethal
- Grid coordinates vs. world coordinates conversion
- Efficient obstacle detection within radius
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from nav_msgs.msg import OccupancyGrid


class CostmapProcessor:
    """
    Processes Nav2 costmaps to extract obstacle and navigation information.
    """

    # Costmap value meanings (Nav2 defaults)
    FREE_SPACE = 0
    UNKNOWN = -1  # or 255 depending on config
    INSCRIBED_OBSTACLE = 253
    LETHAL_OBSTACLE = 254

    def __init__(self):
        """Initialize costmap processor."""
        self.local_costmap: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[OccupancyGrid] = None

    def update_costmap(self, costmap: OccupancyGrid, costmap_type: str = 'local'):
        """
        Store a costmap update.

        Args:
            costmap: OccupancyGrid message
            costmap_type: 'local' or 'global'
        """
        if costmap_type == 'local':
            self.local_costmap = costmap
        else:
            self.global_costmap = costmap

    def get_cost_at_position(
        self,
        x: float,
        y: float,
        costmap_type: str = 'local'
    ) -> Optional[int]:
        """
        Get cost value at world position.

        Args:
            x, y: World coordinates (meters)
            costmap_type: Which costmap to query

        Returns:
            Cost value (0-255) or None if position invalid
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return None

        grid_x, grid_y = self._world_to_grid(x, y, costmap)

        if grid_x is None:
            return None

        info = costmap.info
        idx = grid_y * info.width + grid_x

        if 0 <= idx < len(costmap.data):
            return costmap.data[idx]

        return None

    def find_obstacles_in_radius(
        self,
        center_x: float,
        center_y: float,
        radius: float = 1.0,
        costmap_type: str = 'local'
    ) -> List[Dict[str, Any]]:
        """
        Find all obstacles within radius of a point.

        Args:
            center_x, center_y: Center point in world coordinates
            radius: Search radius in meters
            costmap_type: Which costmap to search

        Returns:
            List of obstacle info dictionaries
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return []

        info = costmap.info
        grid_radius = int(radius / info.resolution) + 1

        center_gx, center_gy = self._world_to_grid(center_x, center_y, costmap)
        if center_gx is None:
            return []

        obstacles = []

        # Search grid cells in square, filter by radius
        for dy in range(-grid_radius, grid_radius + 1):
            for dx in range(-grid_radius, grid_radius + 1):
                # Skip if outside circular radius
                if dx*dx + dy*dy > grid_radius*grid_radius:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                # Bounds check
                if not (0 <= gx < info.width and 0 <= gy < info.height):
                    continue

                idx = gy * info.width + gx
                cost = costmap.data[idx]

                if cost >= self.INSCRIBED_OBSTACLE:
                    world_x, world_y = self._grid_to_world(gx, gy, costmap)
                    distance = ((world_x - center_x)**2 + (world_y - center_y)**2)**0.5

                    obstacles.append({
                        'x': world_x,
                        'y': world_y,
                        'cost': cost,
                        'is_lethal': cost == self.LETHAL_OBSTACLE,
                        'distance': distance
                    })

        return obstacles

    def analyze_path_clearance(
        self,
        path_poses: List,
        clearance_radius: float = 0.5,
        costmap_type: str = 'local'
    ) -> Dict[str, Any]:
        """
        Analyze clearance along a planned path.

        Args:
            path_poses: List of PoseStamped from Path message
            clearance_radius: Radius to check around each waypoint
            costmap_type: Which costmap to use

        Returns:
            Clearance analysis dictionary
        """
        if not path_poses:
            return {'clear': True, 'obstacles': [], 'min_clearance': float('inf')}

        all_obstacles = []
        min_clearance = float('inf')

        # Sample path at intervals (not every waypoint for efficiency)
        sample_step = max(1, len(path_poses) // 20)  # Max 20 samples

        for i in range(0, len(path_poses), sample_step):
            pose = path_poses[i]
            x = pose.pose.position.x
            y = pose.pose.position.y

            obstacles = self.find_obstacles_in_radius(
                x, y, clearance_radius, costmap_type
            )

            for obs in obstacles:
                if obs['distance'] < min_clearance:
                    min_clearance = obs['distance']

                obs['path_index'] = i
                all_obstacles.append(obs)

        return {
            'clear': len(all_obstacles) == 0,
            'obstacle_count': len(all_obstacles),
            'obstacles': all_obstacles[:10],  # Limit to first 10
            'min_clearance': min_clearance if min_clearance != float('inf') else None
        }

    def get_region_statistics(
        self,
        center_x: float,
        center_y: float,
        radius: float = 2.0,
        costmap_type: str = 'local'
    ) -> Dict[str, Any]:
        """
        Get statistics about a region around a point.

        Args:
            center_x, center_y: Center of region
            radius: Region radius in meters
            costmap_type: Which costmap to analyze

        Returns:
            Statistics dictionary
        """
        costmap = self.local_costmap if costmap_type == 'local' else self.global_costmap

        if costmap is None:
            return {}

        info = costmap.info
        grid_radius = int(radius / info.resolution) + 1

        center_gx, center_gy = self._world_to_grid(center_x, center_y, costmap)
        if center_gx is None:
            return {}

        costs = []

        for dy in range(-grid_radius, grid_radius + 1):
            for dx in range(-grid_radius, grid_radius + 1):
                if dx*dx + dy*dy > grid_radius*grid_radius:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                if 0 <= gx < info.width and 0 <= gy < info.height:
                    idx = gy * info.width + gx
                    costs.append(costmap.data[idx])

        if not costs:
            return {}

        costs = np.array(costs)

        return {
            'mean_cost': float(np.mean(costs)),
            'max_cost': int(np.max(costs)),
            'free_percent': float(np.sum(costs == self.FREE_SPACE) / len(costs) * 100),
            'obstacle_percent': float(np.sum(costs >= self.INSCRIBED_OBSTACLE) / len(costs) * 100),
            'sample_count': len(costs)
        }

    def _world_to_grid(
        self,
        x: float,
        y: float,
        costmap: OccupancyGrid
    ) -> Tuple[Optional[int], Optional[int]]:
        """Convert world coordinates to grid coordinates."""
        info = costmap.info

        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            return grid_x, grid_y

        return None, None

    def _grid_to_world(
        self,
        grid_x: int,
        grid_y: int,
        costmap: OccupancyGrid
    ) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        info = costmap.info

        world_x = grid_x * info.resolution + info.origin.position.x
        world_y = grid_y * info.resolution + info.origin.position.y

        return world_x, world_y

    @property
    def has_local_costmap(self) -> bool:
        """Check if local costmap is available."""
        return self.local_costmap is not None

    @property
    def has_global_costmap(self) -> bool:
        """Check if global costmap is available."""
        return self.global_costmap is not None
