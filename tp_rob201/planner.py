"""
Planner class
Implementation of A*
"""

import numpy as np
import heapq

from occupancy_grid import OccupancyGrid


class Planner:
    """Planificateur simple basé sur une grille d'occupation"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid
        # Origine du repère odométrique dans la carte
        self.odom_pose_ref = np.array([0, 0, 0])

    def get_neighbors(self, current_cell):
        """
        Retourne les 8 voisins d'une cellule dans la grille d'occupation.
        current_cell : [x, y] nparray ou liste, cellule courante en coordonnées carte
        """
        x, y = current_cell
        neighbors = []

        # 8 directions possibles autour de la cellule courante
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
        Calcule la distance euclidienne entre deux cellules.
        cell_1, cell_2 : [x, y] liste ou np.ndarray
        """
        cell_1 = np.array(cell_1)
        cell_2 = np.array(cell_2)
        return np.linalg.norm(cell_1 - cell_2)

    def A_Star(self, start, goal):
        """
        Algorithme A* pour trouver le plus court chemin entre start et goal.
        start, goal : [x, y] coordonnées en monde réel
        """
        # Conversion des coordonnées monde en indices de la carte
        sx, sy = self.grid.conv_world_to_map(start[0], start[1])
        gx, gy = self.grid.conv_world_to_map(goal[0], goal[1])
        start = [sx, sy]
        goal = [gx, gy]

        to_visit = {tuple(start)}  # Ensemble des noeuds à explorer
        prev = {}                  # Dictionnaire pour reconstruire le chemin
        cost_from_start = {}       # Coût du chemin depuis le départ
        est_total_cost = {}        # Coût estimé total (g + h)

        cost_from_start[tuple(start)] = 0
        est_total_cost[tuple(start)] = self.heuristic(np.array(start), np.array(goal))

        while to_visit:
            # Sélection du noeud avec le coût estimé le plus faible
            node = min(to_visit, key=lambda k: est_total_cost[k])
            if node == tuple(goal):
                # Chemin trouvé, on le reconstruit
                return self.reconstruct_path(prev, node)

            to_visit.remove(node)
            adj = self.get_neighbors(np.array(node))
            for n in adj:
                n_t = tuple(n)
                temp_g = cost_from_start[node] + self.heuristic(np.array(node), n)
                if temp_g < cost_from_start.get(n_t, np.inf):
                    prev[n_t] = node
                    cost_from_start[n_t] = temp_g
                    est_total_cost[n_t] = temp_g + self.heuristic(n, np.array(goal))
                    if n_t not in to_visit:
                        to_visit.add(n_t)
        return []
    
    def reconstruct_path(self, came_from, current):
        """
        Reconstruit le chemin à partir du dictionnaire des précédents.
        """
        total_path = [current]
        while tuple(current) in came_from:
            current = came_from[tuple(current)]
            total_path.insert(0, current)  # Ajoute au début du chemin
        return total_path

    def plan(self, start, goal):
        """
        Calcule un chemin avec A*, recalcule si start ou goal change.
        start : [x, y, theta] nparray, position de départ (theta non utilisé)
        goal : [x, y, theta] nparray, position d'arrivée (theta non utilisé)
        """
        # Appel de l'algorithme A*
        path = self.A_Star(start, goal)
        # Conversion des indices de carte en coordonnées monde
        for i in range(len(path)):
            x, y = path[i]
            path[i] = self.grid.conv_map_to_world(x, y)
        return path

    def explore_frontiers(self):
        """
        Exploration basée sur les frontières (frontier-based exploration).
        Retourne un objectif à atteindre pour explorer la carte.
        """
        goal = np.array([0, 0, 0])  # À remplacer par une vraie détection de frontière
        return goal
