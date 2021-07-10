from GUI import GUI
from HAL import HAL
import numpy as np
import matplotlib.image as mpimg
import math
from scipy import signal
from scipy.stats import norm
import config
import copy
# Enter sequential code!
num_particles = 100
particles = []
observed_particles = []
weights = []
robot_x = 0
robot_y = 0
robot_yaw = 0
# Posicion inicial del robot
robot_pose_x = HAL.getPose3d().x
robot_pose_y =  HAL.getPose3d().y
robot_pose_yaw = HAL.getPose3d().yaw

# Incertidumbre de la posicion incial
var_x = 1
var_y = 1
var_yaw = 2.0 * math.pi / 180.0

# Marcas del entorno
env_landmarks = [(4.65/0.03, 11.3/0.03), (9.6/0.03, 3/0.03), (15.3/0.03, 9.5/0.03),
            (17.7/0.03, 11.3/0.03), (16/0.03, 16.9/0.03), (20.2/0.03, 14.5/0.03),
            (12.8/0.03, 19.3/0.03), (10./0.03, 14.5/0.03), (7.10/0.03, 12/0.03),
            (9.5/0.03, 6.4/0.03), (10.3/0.03, 10.5/0.03), (13.5/0.03, 8.8/0.03)]

# Etapa de generacion de las particulas
# Peso de las particulas se distribuye uniformemente entre todas las particulas 
wo = 1.0 / float(num_particles)
for i in range(num_particles):
    x = np.random.normal(robot_pose_x, var_x)
    y = np.random.normal(robot_pose_y, var_y)
    yaw = np.random.normal(robot_pose_yaw, var_yaw)

    particles.append((x, y, yaw, wo))
particles = np.array(particles).tolist()

# Mostrar por el GUI las particulas generadas. Recordar transformar las medidas en metros en medidas en pixeles en .js
GUI.showParticles(particles)

# Etapa de observacion
# Se obtiene en que landmark se encuentra el robot
min_distance = 0
aux_distance = 1000
for landmark in env_landmarks:
    distance = math.sqrt((HAL.getPose3d().x - landmark[0]) **2 + (HAL.getPose3d().y - landmark[1])**2)
    if (distance < aux_distance):
        min_distance = distance
        aux_distance = min_distance
        current_landmark = landmark
    else:
        pass
# Se mide la diferencia que hay entre las particulas y la marca del entorno

for particle in num_particles:
    x = particle[0]
    y = particle[1]
    #Para cada particula, mirar la distancia hacia el current landmark
    observed_dist = math.sqrt((x - current_landmark[0]) **2 + (y - current_landmark[1])**2)
    observed_particles.append((observed_dist))
prob = norm.pdf(observed_particles, particles, 10)
print(prob)
for i in range(particles.shape[0]):
    particles[i][3] = prob[i]
# normalizers = np.sum(prob[:-1])
# weights = prob[-1] / normalizers

# Ahora que se tienen los pesos
# Se realiza el resampling
particle_resampling_indicies = np.random.choice(particles.shape[0], particles.shape[0], replace=True, p=prob)
particle_resampling = particles[particle_resampling_indicies]
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