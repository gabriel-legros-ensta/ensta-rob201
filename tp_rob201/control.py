""" A set of robotics control functions """

import random
import numpy as np

historique_rotation = ["gauche"] 
def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """

    laser_dist = lidar.get_sensor_values() # laser_dist = [d0, d1,.., d360] 180° = devant le robot
    if min(laser_dist[180-50:180+50+1])<60:
        if(len(historique_rotation)>0 and historique_rotation[-1]=="left"):
            rotation_speed = 0.5
            historique_rotation.append("right")
        else:
            rotation_speed = -0.5
            historique_rotation.append("left")

        speed = 0.3
        command = {"forward": speed, "rotation": rotation_speed}
        if len(historique_rotation) > 5:
            historique_rotation.pop(0)

    else:
        speed = 0.5
        rotation_speed = 0
        command = {"forward": speed, "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    command = {"forward": 0,
               "rotation": 0}

    return command
