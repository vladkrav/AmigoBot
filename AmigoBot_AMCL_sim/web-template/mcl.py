from GUI import GUI
from HAL import HAL
import numpy as np
import matplotlib.image as mpimg
import math
from scipy import signal
# from scipy import stats
# from multiprocess import Pool
import config
# from functools import partial
# from environment import Environment
import copy
# Enter sequential code!


# ROBOT_DIAMETER = 5
# no_particles = 10
# no_random_particles = 5000
# type_localization = True
# scene = 'scene-1'
# map_name = 'scenes/%s.png' % 'scene-1'
# mapa = mpimg.imread(map_name)[::-1, :, 0]
# show_particles = True
# mark = np.ones((config.ROBOT_DIAMETER, config.ROBOT_DIAMETER))
# convolve_mark = (signal.convolve2d(1-mapa[:, :], mark, mode='same', boundary='fill', fillvalue=0)) / np.sum(mark)
# convolve_mark_overlay = np.copy(mapa)
# threshold = 1/np.sum(mark)
# convolve_mark_overlay[convolve_mark > threshold]
# map_with_safe_boundary = np.copy(mapa)
# map_with_safe_boundary[convolve_mark > threshold]
# traversable_area = np.stack(np.nonzero(1 - (map_with_safe_boundary.T < 1)), axis=1)
# #Se generan las partículas aleatorias
# particles_xy_indices = np.random.choice(traversable_area.shape[0], size=no_particles, replace=True)
# particles_xy = traversable_area[particles_xy_indices]
# particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)
# particles = np.hstack([particles_xy, particles_theta])
# # GUI.showParticles(np.array(particles).tolist())

GUI.showEstimatedPose((4/0.03, 8/0.03, 0))
aux_pos_x = HAL.getPose3d().x / 0.03
aux_pos_y =  HAL.getPose3d().y / 0.03
aux_pos_yaw = HAL.getPose3d().yaw
noise = True
distance_differences = []
angle_differences = []
# no_sensors = config.SYSTEM_NO_SENSORS
# radar_thetas = (np.arange(0, no_sensors) - no_sensors // 2)*(np.pi/no_sensors)
# env = Environment(scene, 20)

#Inicializacion de MCL
#Se indica la posicion inicial del robot
robot_pose_x = 4 / 0.03
robot_pose_y =  8 / 0.03
robot_pose_yaw = 0
# Ground truth
gt_x = HAL.getPose3d().x / 0.03
gt_y =  HAL.getPose3d().y / 0.03
gt_yaw = HAL.getPose3d().yaw / 0.03
#Se indica el ruido de la odometria
odom_noise1 = 1.0
odom_noise2 = 0.5
odom_noise3 = 0.5
odom_noise4 = 2.0
#Se indica la varianza de las medidas
measurement_variance = 0.1 * 0.1
measurement_resolution = 0.1
z_hit = 0.9
z_rand = 0.1
if(z_hit + z_rand != 1.0):
    console.print("Sum of z_hit and z_rand must be one.")
resample_threshold = 0.5
#Numero de particulas
particle_num = 100
particles = []
landmarks = [(4.65/0.03, 11.3/0.03), (9.6/0.03, 3/0.03), (15.3/0.03, 9.5/0.03),
            (17.7/0.03, 11.3/0.03), (16/0.03, 16.9/0.03), (20.2/0.03, 14.5/0.03),
            (12.8/0.03, 19.3/0.03), (10./0.03, 14.5/0.03), (7.10/0.03, 12/0.03),
            (9.5/0.03, 6.4/0.03), (10.3/0.03, 10.5/0.03), (13.5/0.03, 8.8/0.03)]

plot_size_x = 5.0
plot_size_y = 5.0
PI = 3.14159265359
PI2 = 6.28318530718
#Añadiendo landmarks
# landmarks.append([2.0, 2.0])
var_x = 100
var_y = 100
var_yaw = 2.0 * math.pi / 180.0
# Se generan las primeras partículas aleatoriamente alrededor del punto incial del robot
wo = 1.0 / float(particle_num)
for i in range(particle_num):
    x = np.random.normal(robot_pose_x, var_x)
    y = np.random.normal(robot_pose_y, var_y)
    yaw = np.random.normal(robot_pose_yaw, var_yaw)
    while yaw < -PI:
        yaw += PI2
    while yaw > PI:
        yaw -= PI2
    particles.append((x, y, yaw, wo))
particles = np.array(particles).tolist()
# console.print(particles)
GUI.showParticles(particles)
measurements = []
measurement_range_variance = 0.3 * 0.3 / 0.03
max_measurement_range = 8.0
measurement_angle_variance = 0.01 * 0.01 / 0.03
random_measurement_rate = 0.05 / 0.03
while True:
    # Enter iterative code!

    #Se obtiene la distancia recorrida desde la posicion inicial
    delta_dist = math.sqrt((HAL.getPose3d().x / 0.03 - aux_pos_x)**2 + (( HAL.getPose3d().y / 0.03) - aux_pos_y)**2)
    delta_yaw = abs(aux_pos_yaw - HAL.getPose3d().yaw)

    if(delta_dist > 1 or delta_yaw > math.pi/6):
        # Se obiene la posicion del robot ground truth
        x = gt_x + delta_dist * math.cos(gt_yaw)
        y = gt_y + delta_dist * math.sin(gt_yaw)
        yaw = gt_yaw + delta_yaw
        gt_x = x
        gt_y = y
        while yaw < -PI:
            gt_yaw += PI2
        while yaw > PI:
            gt_yaw -= PI2
        # Etapa de observacion, se mide la diferencia que hay entre la posicion del robot y la marca del entorno
        for i in range(len(landmarks)):
            dx = landmarks[i][0] - gt_x
            dy = landmarks[i][1] - gt_y
            dl = np.random.normal(math.sqrt(dx * dx + dy * dy), measurement_range_variance)
            if dl <= max_measurement_range:
                dyaw = np.random.normal(math.atan2(dy, dx) - gt_yaw, measurement_angle_variance)
                while dyaw < -PI:
                    dyaw += PI2
                while yaw > PI:
                    dyaw -= PI2
                # simulate random range measurement
                if np.random.random() < random_measurement_rate:
                    dl = np.random.random() * max_measurement_range
                measurements.append([dl, dyaw])

        # Se obtienen las medidas del sensor laser
        # En esta parte se actualizan las particulas
        delta_dist2 = delta_dist * delta_dist
        delta_yaw2 = delta_yaw * delta_yaw
        for i in range(particle_num):
            del_dist = np.random.normal(delta_dist, odom_noise1 * delta_dist2 + odom_noise2 * delta_yaw2)
            del_yaw = np.random.normal(delta_yaw, odom_noise3 * delta_dist2 + odom_noise4 * delta_yaw2)
            x = particles[i][0] + del_dist * math.cos(particles[i][2])
            y = particles[i][1] + del_dist * math.sin(particles[i][2])
            console.print("El valor de particles y:")
            console.print(particles[i][1])
            console.print("El valor de y:")
            console.print(y)
            yaw_ = particles[i][2] + del_yaw
            while yaw < -PI:
                yaw += PI2
            while yaw > PI:
                yaw -= PI2
            particles[i][0] = x
            particles[i][1] = y
            particles[i][2] = yaw
            new_particles = np.array(particles).tolist()
            # console.print(new_particles)
            GUI.showParticles(new_particles)
            #Ahora se calculan los pesos de las particulas
            total_weight = 0.0
            norm_coef = 1.0 / (math.sqrt(2.0 * PI * measurement_variance))
            for i in range(particle_num):
                total_log_prob = 0.0
                for j in range(len(measurements)):
                    myaw = particles[i][2] + measurements[j][1]
                    # myaw = particles[i][2] + measurements[j]
                    mx = measurements[j][0] * math.cos(myaw) + particles[i][0]
                    my = measurements[j][0] * math.sin(myaw) + particles[i][1]
                    # mx = measurements[j] * math.cos(myaw) + particles[i][0]
                    # my = measurements[j] * math.sin(myaw) + particles[i][1]
                    min_dl = 0.0
                    for k in range(len(landmarks)):
                        dx = landmarks[k][0] - mx
                        dy = landmarks[k][1] - my
                        dl = math.sqrt(dx * dx + dy * dy)
                        if k == 0:
                            min_dl = dl
                        elif min_dl > dl:
                            min_dl = dl
                    prob = z_hit * norm_coef * math.exp(-0.5 * (min_dl * min_dl) / (2.0 * measurement_variance)) + z_rand * 10e-6
                    prob *= measurement_resolution
                    if prob > 1.0:
                        prob = 1.0
                    total_log_prob += math.log(prob)
                prob = math.exp(total_log_prob)
                weight = particles[i][3] * prob
                particles[i][3] = weight
                total_weight += weight
            effective_sample_size = 0.0
            #Ahora se estima la posicion del robot
            x = 0.0
            y = 0.0
            yaw = 0.0
            tmp_yaw = copy.copy(robot_pose_yaw)
            for i in range(particle_num):
                x += particles[i][0] * particles[i][3]
                y += particles[i][1] * particles[i][3]
                dyaw = tmp_yaw - particles[i][2]
                while dyaw < -PI:
                    dyaw += PI2
                while dyaw > PI:
                    dyaw -= PI2
                yaw += dyaw * particles[i][3]
            yaw_ = tmp_yaw - yaw
            robot_pose_x = x
            robot_pose_y = y
            while yaw_ < -PI:
                yaw_ += PI2
            while yaw_ > PI:
                yaw_ -= PI2
            robot_pose_yaw = yaw_
            GUI.showEstimatedPose((robot_pose_x, robot_pose_y, robot_pose_yaw))
            #Ahora se realiza un remuestreo de las particulas
            if (effective_sample_size > float(particle_num) * resample_threshold):
                console.print("Algo esta mal.")
            tmp_particles = copy.copy(particles)
            wo = 1.0 / float(particle_num)
            board = []
            board.append(particles[0][3])
            for i in range(1, particle_num):
                board.append(board[i - 1] + particles[i][3])
            for i in range(particle_num):
                darts = np.random.random()
                for j in range(particle_num):
                    if (darts < board[j]):
                        particles[i] = copy.copy(tmp_particles[j])
                        particles[i][3] = wo
                        break
            # console.print(particles)

            #Actualizacion de la posicion del robot
            aux_pos_x = HAL.getPose3d().x / 0.03
            aux_pos_y = HAL.getPose3d().y / 0.03
            aux_pos_yaw = HAL.getPose3d().yaw
