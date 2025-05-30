"""
Robot controller definition
Complete controller including SLAM, planning, path following
"""
import numpy as np

from place_bot.entities.robot_abstract import RobotAbstract
from place_bot.entities.odometer import OdometerParams
from place_bot.entities.lidar import LidarParams

from tiny_slam import TinySlam

from control import potential_field_control, reactive_obst_avoid
from occupancy_grid import OccupancyGrid
from planner import Planner


# Definition of our robot controller
class MyRobotSlam(RobotAbstract):
    """A robot controller including SLAM, path planning and path following"""

    def __init__(self,
                 lidar_params: LidarParams = LidarParams(),
                 odometer_params: OdometerParams = OdometerParams()):
        # Passing parameter to parent class
        super().__init__(should_display_lidar=False,
                         lidar_params=lidar_params,
                         odometer_params=odometer_params)

        # step counter to deal with init and display
        self.counter = 0

        # Init SLAM object
        # Here we cheat to get an occupancy grid size that's not too large, by using the
        # robot's starting position and the maximum map size that we shouldn't know.
        size_area = (1400, 1000)
        robot_position = (-400, 0) # 439.0, 195
        self.occupancy_grid = OccupancyGrid(x_min=-(size_area[0] / 2 + robot_position[0]),
                                            x_max=size_area[0] / 2 - robot_position[0],
                                            y_min=-(size_area[1] / 2 + robot_position[1]),
                                            y_max=size_area[1] / 2 - robot_position[1],
                                            resolution=2)
        # self.occupancy_grid = OccupancyGrid(x_min=-(size_area[0] / 2 + robot_position[0]),
        #                                     x_max=size_area[0] / 2 - robot_position[0],
        #                                     y_min=-(size_area[1] / 2 + robot_position[1]),
        #                                     y_max=size_area[1] / 2 - robot_position[1],
        #                                     resolution=2)


        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        # storage for pose after localization
        self.corrected_pose = np.array([0, 0, 0])

    def control(self):
        """
        Main control function executed at each time step
        """
        return self.control_tp1()

    def control_tp1(self):
        """
        Control function for TP1
        Control funtion with minimal random motion
        """
        pose = self.odometer_values()
        goal = [800, 70, 0]
        command = potential_field_control(self.lidar(), pose, goal)
        self.counter +=1
        if self.counter < 30 :
            corrected_pose = self.tiny_slam.get_corrected_pose(pose)
            command = potential_field_control(self.lidar(), pose, goal)
            self.tiny_slam.update_map(self.lidar(), corrected_pose)
        else:
            best_score = self.tiny_slam.localise(self.lidar(), pose)
            print(best_score)
            if best_score > 20: #40
                corrected_pose = self.tiny_slam.get_corrected_pose(pose)
                command = potential_field_control(self.lidar(), pose, goal)
                self.tiny_slam.update_map(self.lidar(), corrected_pose)

                traj = self.planner.plan(corrected_pose, goal)
                traj = np.array(traj)
                self.occupancy_grid.display_cv(corrected_pose, goal, traj)  
        return command

    def control_tp2(self):
        pose = self.odometer_values()
        goal = [800, 100, 0] # _ 100 _
    
        # Compute new command speed to perform obstacle avoidance
        command = potential_field_control(self.lidar(), pose, goal)
        return command
    
    def control_tp3(self):
        """
        Control function for TP2
        Main control function with full SLAM, random exploration and path planning
        """
        pose = self.odometer_values()
        corrected_pose = self.tiny_slam.get_corrected_pose(pose)
        goal = [800, 70, 0] # _ 100 _

        # Compute new command speed to perform obstacle avoidance
        command = potential_field_control(self.lidar(), pose, goal)
        self.tiny_slam.update_map(self.lidar(), corrected_pose)

        return command



