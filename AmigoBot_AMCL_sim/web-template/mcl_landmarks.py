from GUI import GUI
from HAL import HAL
import numpy as np
import matplotlib.image as mpimg
import math
from scipy import signal
import config
import copy
# Enter sequential code!
num_particles = 100
# env_landmarks = 

#Etapa de observacion
weights = []
for particle in num_particles:
    x = particle[0]
    y = particle[1]
    yaw = particle[2]

    #Para cada punto, mirar la distancia hacia el landmark
    observed_dist = math.sqrt((x - ) **2 + (start[1] - end[1])**2)