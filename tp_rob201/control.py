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

    # Pose actuelle et cible
    q = np.array(current_pose[:2])  # Position actuelle [x, y] odometrie
    qgoal = np.array(goal_pose[:2])  # Position cible [x, y]

    # Calcul de la distance et du vecteur vers l'objectif
    distance_to_goal = np.linalg.norm(qgoal - q)
    stop_threshold = 9
    Kgoal = 0.3
    #Kquad = Kgoal / stop_threshold  # pour que ce soit continu
    if distance_to_goal < stop_threshold:
        #attraction = Kquad*(qgoal-q)
        return {"forward": 0.0, "rotation": 0.0}
        
    else:
        attraction = Kgoal*(qgoal-q)/distance_to_goal


    sensor_values = lidar.get_sensor_values()
    ray_angles = lidar.get_ray_angles()

    # Distance minimale
    min_dist = np.min(sensor_values)
    min_index = np.argmin(sensor_values)
    angle = ray_angles[min_index]

    # Si trop loin, pas de force répulsive
    d_safe =100
    if min_dist < d_safe and min_dist > 0.01:  # éviter les zéros/NaN
        # Position obstacle dans le repère local du robot
        # obs_local = np.array([min_dist * np.cos(angle), min_dist * np.sin(angle)]) position dans repere local ru robot, on veut odometrie
        x_obs_odom = current_pose[0] + min_dist*np.cos(current_pose[2] + angle)
        y_obs_odom = current_pose[1] + min_dist*np.sin(current_pose[2] + angle)
        obs_local = np.array([x_obs_odom,y_obs_odom])

        # Calcul du gradient répulsif
        Kobs = 1000
        diff = -(q - obs_local)  # on pousse le robot à l'opposé de l'obstacle
        factor = Kobs * (1/min_dist - 1/d_safe) / (min_dist**3)
        repulsion = factor * diff
    else:
        repulsion = np.array([0.0, 0.0])
    
    total_gradient =  attraction + repulsion
      # Calcul de la vitesse linéaire
    theta = current_pose[2]
    desired_angle = np.arctan2(total_gradient[1], total_gradient[0])
    angle_error = desired_angle - theta
    v = np.clip(np.linalg.norm(total_gradient), -1.0, 1.0)
    w = np.clip(angle_error, -1.0, 1.0)


    return {"forward": v, "rotation": w}



