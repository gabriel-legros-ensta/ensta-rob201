"""
Planner class
Implementation of A*
"""

import numpy as np
import heapq

from occupancy_grid import OccupancyGrid


class Planner:
    """Simple occupancy grid Planner"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid


        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def get_neighbors(self, current_cell):
        """
        Get the 8 neighbors of a cell in the occupancy grid
        current_cell : [x, y] nparray or list, current cell in world coordinates
        """
        x, y = current_cell
        neighbors = []

        # 8 directions (dx, dy)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            ( 0, -1),          ( 0, 1),
            ( 1, -1), ( 1, 0), ( 1, 1)
        ]

        for dx, dy in directions:
            neighbor = [x + dx, y + dy]
            neighbors.append(neighbor)

        return neighbors
        
    def heuristic(self, cell_1, cell_2):
        """
        Calculate the Euclidean distance between two cells.
        cell_1, cell_2 : [x, y] list or np.ndarray
        """
        cell_1 = np.array(cell_1)
        cell_2 = np.array(cell_2)
        return np.linalg.norm(cell_1 - cell_2)


    def plan(self, start, goal):
        """
        Compute a path using A*, recompute plan if start or goal change
        start : [x, y, theta] nparray, start pose in world coordinates (theta unused)
        goal : [x, y, theta] nparray, goal pose in world coordinates (theta unused)
        """
        # TODO for TP5

        def reconstruct_path(came_from, current):
            total_path = [current]
            while tuple(current) in came_from:
                current = came_from[tuple(current)]
                total_path.insert(0, current)  # prepend
            return total_path

        def A_Star(self, start, goal):
            sx, sy = self.grid.conv_world_to_map(start[0], start[1])
            gx, gy = self.grid.conv_world_to_map(goal[0], goal[1])
            start = [sx, sy]
            goal = [gx, gy]

            to_visit = {tuple(start)}
            prev = {}
            cost_from_start = {}
            est_total_cost = {}

            cost_from_start[tuple(start)] = 0
            est_total_cost[tuple(start)] = self.heuristic(np.array(start), np.array(goal))

            while to_visit:
                node = min(to_visit, key=lambda k: est_total_cost[k])
                if node == tuple(goal):
                    return reconstruct_path(prev, node)

                to_visit.remove(node)
                adj = self.get_neighbors(np.array(node))
                for n in adj:
                    n_t = tuple(n)
                    temp_g = cost_from_start[node] + self.heuristic(np.array(node), n)
                    if temp_g < cost_from_start[n_t]:
                        prev[n_t] = node
                        cost_from_start[n_t] = temp_g
                        est_total_cost[n_t] = temp_g + self.heuristic(n, np.array(goal))
                        if n_t not in to_visit:
                            to_visit.add(n_t)
            return []
        
        path = A_Star(start, goal)
        for i in range(len(path)):
            x, y = path[i]
            path[i] = self.grid.conv_map_to_world(x, y)
        return path

    def explore_frontiers(self):
        """ Frontier based exploration """
        goal = np.array([0, 0, 0])  # frontier to reach for exploration
        return goal
