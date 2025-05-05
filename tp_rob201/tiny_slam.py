""" A simple robotics navigation code including SLAM, exploration, planning"""

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):  # pose = position robot repère absolu
        """
        Computes the sum of log probabilities of laser end points in the map
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, position of the robot to evaluate, in world coordinates
        """
        # TODO for TP4

        sensor_values = np.array(lidar.get_sensor_values())
        ray_angles = np.array(lidar.get_ray_angles())
        max_range = lidar.max_range                   # max_range = x mètres
        valid = sensor_values < max_range
        sensor = sensor_values[valid]
        angles = ray_angles[valid]

        x = pose[0] + sensor * np.cos(angles + pose[2])
        y = pose[1] + sensor * np.sin(angles + pose[2])

        # x_map, y_map = self.grid.conv_world_to_map(x, y)
        # mask = ((x_map >= 0) & (x_map < self.grid.x_max_map) & (y_map >= 0) & (y_map < self.grid.y_max_map)) 
        # x_map = x_map[mask]
        # y_map = y_map[mask]

        # score = np.sum(self.grid.occupancy_map[y_map, x_map])
        score = 0
        for i in range(len(x)):

            x_m, y_m = self.grid.conv_world_to_map(x[i], y[i])
  
            if 0 <= x_m < 500 and 0 <= y_m < 500:
            # Si dans les limites, on ajoute la valeur correspondante dans occupancy_map au score
                score += self.grid.occupancy_map[x_m, y_m]

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        # TODO for TP4

        xO,yO,thetaO = odom_pose
        xOref,yOref,thetaOref = odom_pose_ref
        corrected_x = xOref + xO * np.cos(thetaOref) - yO * np.sin(thetaOref)
        corrected_y = yOref + xO * np.sin(thetaOref) + yO * np.cos(thetaOref)
        corrected_theta = thetaOref + thetaO
        corrected_pose = np.array([corrected_x, corrected_y, corrected_theta])
        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        score = self._score(lidar, self.get_corrected_pose(raw_odom_pose, np.array(self.odom_pose_ref)))
        while(...):
            offset = [np.random.normal(loc=0.0, scale=1) for _ in range(2)]

        best_score = 0

        return best_score

    def update_map(self, lidar, pose): # pose : odométrie
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        # TODO for TP3

        sensor_values = np.array(lidar.get_sensor_values())
        ray_angles = np.array(lidar.get_ray_angles())

        x = pose[0] + (sensor_values-5) * np.cos(ray_angles + pose[2])
        y = pose[1] + (sensor_values-5) * np.sin(ray_angles + pose[2])
        # x = pose[0] - sensor_values * np.cos(ray_angles + pose[2]) pour les add_value ?
        for i in range(len(x)):
             self.grid.add_value_along_line(pose[0], pose[1], x[i], y[i], -1)
            # self.grid.add_value_along_line(x[i]-2, y[i]-2, x[i]-1, y[i]-1, 0.5)
            # self.grid.add_value_along_line(x[i]+1, y[i]+1, x[i]+2, y[i]+2, 0.5)
            # self.grid.add_value_along_line(x[i]+3, y[i]+3, x[i]+4, y[i]+4, -1)
        x = pose[0] + (sensor_values-5) * np.cos(ray_angles + pose[2])
        y = pose[1] + (sensor_values-5) * np.sin(ray_angles + pose[2])

        self.grid.add_map_points(x, y, 2)
    

        self.grid.occupancy_map = np.clip(self.grid.occupancy_map, -40, 40)

        self.grid.display_cv(pose, goal=None, traj=None)




