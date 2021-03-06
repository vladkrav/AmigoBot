from GUI import GUI
from HAL import HAL
import numpy as np
import matplotlib.image as mpimg
import math
from scipy import signal
from scipy.stats import norm
import config
# Enter sequential code!
num_particles = 100
particles = []
observed_particles = []
array_area = []
weights = []
robot_x = 0
robot_y = 0
robot_yaw = 0
localization = False
# Posicion inicial del robot
robot_pose_x = HAL.getPose3d().x
robot_pose_y =  HAL.getPose3d().y
robot_pose_yaw = HAL.getPose3d().yaw
# Incertidumbre de la posicion incial
var_x = 1
var_y = 1
var_yaw = 2.0 * math.pi / 180.0
# Marcas del entorno
env_landmarks = [(4.65, 11.3), (9.6, 3), (15.3, 9.5),
            (17.7, 11.3), (16, 16.9), (20.2, 14.5),
            (12.8, 19.3), (10, 14.5), (7.10, 12),
            (9.5, 6.4), (10.3, 10.5), (13.5, 8.8)]
# Etapa de generacion de las particulas
# Se obtiene el espacio libre del mapa, para despues generar las particulas
scene = 'scene-1'
map_name = 'scenes/%s.png' % 'scene-1'
mapa = mpimg.imread(map_name)[::-1, :, 0]
mark = np.ones((config.ROBOT_DIAMETER, config.ROBOT_DIAMETER))
convolve_mark = (signal.convolve2d(1-mapa[:, :], mark, mode='same', boundary='fill', fillvalue=0)) / np.sum(mark)
convolve_mark_overlay = np.copy(mapa)
threshold = 1/np.sum(mark)
convolve_mark_overlay[convolve_mark > threshold]
map_with_safe_boundary = np.copy(mapa)
map_with_safe_boundary[convolve_mark > threshold]
traversable_area = np.stack(np.nonzero(1 - (map_with_safe_boundary.T < 1)), axis=1)
# Segun el tipo de localizacion elegido, se distribuyen las particulas
if(localization == True):
    # Peso de las particulas se distribuye uniformemente entre todas las particulas
    wo = 1.0 / float(num_particles)
    for i in range(num_particles):
        x = int(np.random.normal(robot_pose_x, var_x) / 0.03)
        y = int(np.random.normal(robot_pose_y, var_y) / 0.03)
        yaw = np.random.normal(robot_pose_yaw, var_yaw)
        particles.append((x, y, yaw, wo))
    particles = np.array(particles).tolist()
else:
    # Se generan las particulas aleatorias por todo el mapa
    particles_xy_indices = np.random.choice(traversable_area.shape[0], size=num_particles, replace=True)
    particles_xy = traversable_area[particles_xy_indices]
    particles_theta = np.random.uniform(0.0, 2*np.pi, (num_particles, 1)) % (2*np.pi)
    particles = np.hstack([np.multiply(particles_xy, 0.03), particles_theta]).tolist()
# Mostrar por el GUI las particulas generadas. Recordar transformar las medidas en metros en medidas en pixeles en .js
GUI.showParticles(particles)
while True:
    # Enter iterative code!
    distance_control = math.sqrt((HAL.getPose3d().x - robot_pose_x)**2 + (HAL.getPose3d().y- robot_pose_y)**2)
    distance_yaw_control = abs(robot_pose_x - HAL.getPose3d().yaw)
    if(distance_control >= 1):
        # console.print("Entra")
        # Etapa de observacion
        # Se obtiene en que landmark se encuentra el robot
        min_distance = 0
        aux_distance = 1000
        for landmark in env_landmarks:
            # console.print("Entra2")
            distance = math.sqrt((HAL.getPose3d().x - landmark[0]) **2 + (HAL.getPose3d().y - landmark[1])**2)
            if (distance < aux_distance):
                min_distance = distance
                aux_distance = min_distance
                current_landmark = landmark
            else:
                pass
        # Se mide la diferencia que hay entre las particulas y la marca del entorno

        for particle in particles:
            x = particle[0]
            y = particle[1]
            #Para cada particula, mirar la distancia hacia el current landmark
            observed_dist = math.sqrt((x - current_landmark[0]) **2 + (y - current_landmark[1])**2)
            observed_particles.append((observed_dist))
        # prob = norm.pdf(observed_particles, np.array(particles).shape[0], 10)
        prob = norm.pdf(observed_particles, np.mean(observed_particles), 10)
        console.print(prob)
        for i in range(np.array(particles).shape[0]):
            console.print(i)
            particles[i][3] = prob[i]
        # normalizers = np.sum(prob[:-1])
        # weights = prob[-1] / normalizers

        # Ahora que se tienen los pesos
        # Se realiza el resampling
        console.print("No llega")
        particle_resampling_indicies = np.random.choice(np.array(particles).shape[0], np.array(particles).shape[0], replace=True, p=prob)
        console.print("No llega")
        particle_resampling = traversable_area[particle_resampling_indicies]
        console.print("No llega")
        GUI.showParticles(particle_resampling.tolist())

        # Se aplica el modelo de movimiento a las particulas
        move_yaw = abs(HAL.getPose3d().yaw - robot_pose_yaw)
        for particle in particles:
            # Actualizar el giro para la siguiente iteracion
            robot_pose_yaw = HAL.getPose3d().yaw
            if(0 < HAL.getPose3d().yaw < math.pi/2):
                alpha = HAL.getPose3d().yaw
            elif(math.pi/2 < HAL.getPose3d().yaw < math.pi):
                alpha = math.pi - HAL.getPose3d().yaw
            elif(math.pi < HAL.getPose3d().yaw < 3*math.pi/2):
                alpha = HAL.getPose3d().yaw - math.pi
            elif(HAL.getPose3d().yaw > 3*math.pi/2):
                alpha = 2*math.pi - HAL.getPose3d().yaw

            # Se calcula la distancia recorrida por el robot
            distance = math.sqrt((HAL.getPose3d().x - robot_pose_x)**2 + (HAL.getPose3d().y - robot_pose_y)**2)
            # Movimiento en x
            move_x = math.cos(alpha) * distance
            # Movimiento en y
            move_y = math.sin(alpha) * distance
            # Se aplica el movimiento calculado a las particulas
            particle[i][0] = particle[i][0] + move_x
            particle[i][1] = particle[i][1] + move_y
            particle[i][2] = particle[i][2] + move_yaw
        # Se estima la posicion del robot
        for i in range(num_particles):
            robot_x += particles[i][0] * particles[i][3]
            robot_y += particles[i][1] * particles[i][3]
            robot_yaw += particles[i][2] * particles[i][3]
        GUI.showEstimatedPose((robot_x, robot_y, robot_yaw))

        # Actualizar el giro para la siguiente iteracion
        robot_pose_yaw = HAL.getPose3d().yaw
        # Actualizar la x para la siguiente iteracion
        robot_pose_x = HAL.getPose3d().x
        # Actualizar la y para la siguiente iteracion
        robot_pose_y = HAL.getPose3d().y