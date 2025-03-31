""" A set of robotics control functions """

import random
import numpy as np

def reactive_obst_avoid(lidar):
    """
    Amélioration de l'évitement d'obstacles en fonction de l'angle de l'obstacle.
    lidar : objet Place-Bot fournissant les données lidar
    """
    laser_dist = lidar.get_sensor_values()  # laser_dist = [d0, d1, ..., d360] avec 180° devant le robot

    # On définit une plage pour analyser le secteur avant (par exemple de 160° à 200°)
    front_sector = laser_dist[160:201]  # 201 car l'index 200 doit être inclus
    min_distance = min(front_sector)

    # Si un obstacle est détecté à moins de 60 unités dans le secteur
    if min_distance < 60:
        # Calculer la moyenne des distances pour la partie gauche et droite du secteur
        left_avg = sum(laser_dist[160:180]) / 20  # de 160° à 179°
        right_avg = sum(laser_dist[180:200]) / 20   # de 180° à 199°

        # Choisir la direction opposée au côté où l'obstacle est le plus proche
        if left_avg > right_avg:
            rotation_speed = -0.5  # tourner vers la gauche (négatif)
        else:
            rotation_speed = 0.5   # tourner vers la droite

        speed = 0.1  # avancer lentement lors de l'évitement
    else:
        speed = 0.3
        rotation_speed = 0  # pas de rotation si aucun obstacle proche

    return {"forward": speed, "rotation": rotation_speed}

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
