from GUI import GUI
from HAL import HAL
import numpy as np
import matplotlib.image as mpimg
import math
from scipy import signal
from scipy import stats
from multiprocess import Pool
import config
from functools import partial
from environment import Environment
# Enter sequential code!


ROBOT_DIAMETER = 5
no_particles = 10
no_random_particles = 5000
type_localization = True
scene = 'scene-1'
map_name = 'scenes/%s.png' % 'scene-1'
mapa = mpimg.imread(map_name)[::-1, :, 0]
show_particles = True
mark = np.ones((config.ROBOT_DIAMETER, config.ROBOT_DIAMETER))
convolve_mark = (signal.convolve2d(1-mapa[:, :], mark, mode='same', boundary='fill', fillvalue=0)) / np.sum(mark)
convolve_mark_overlay = np.copy(mapa)
threshold = 1/np.sum(mark)
convolve_mark_overlay[convolve_mark > threshold]
map_with_safe_boundary = np.copy(mapa)
map_with_safe_boundary[convolve_mark > threshold]
traversable_area = np.stack(np.nonzero(1 - (map_with_safe_boundary.T < 1)), axis=1)
#Se generan las partículas aleatorias
particles_xy_indices = np.random.choice(traversable_area.shape[0], size=no_particles, replace=True)
particles_xy = traversable_area[particles_xy_indices]
particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)
particles = np.hstack([particles_xy, particles_theta])
GUI.showParticles(np.array(particles).tolist())
aux_pos_x = HAL.getPose3d().x
aux_pos_y = HAL.getPose3d().y
aux_pos_yaw = HAL.getPose3d().yaw
noise = True
distance_differences = []
angle_differences = []
no_sensors = config.SYSTEM_NO_SENSORS
radar_thetas = (np.arange(0, no_sensors) - no_sensors // 2)*(np.pi/no_sensors)
env = Environment(scene, 20)
while True:
    # Enter iterative code!
    distance = math.sqrt((HAL.getPose3d().x - aux_pos_x)**2 + (HAL.getPose3d().y - aux_pos_y)**2)
    #console.print(distance)
    distance_yaw = abs(aux_pos_yaw - HAL.getPose3d().yaw)
    if(distance >= 1 or distance_yaw >= math.pi/6):
        #Actualización de la posición
        aux_pos_x = HAL.getPose3d().x
        aux_pos_y = HAL.getPose3d().y
        aux_pos_yaw = HAL.getPose3d().yaw
        robot_pos = ((aux_pos_x, aux_pos_y, aux_pos_yaw))
        particles_xy_indices = np.random.choice(traversable_area.shape[0], size=no_particles, replace=True)
        particles_xy = traversable_area[particles_xy_indices]
        particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)
        particles = np.hstack([particles_xy, particles_theta])
        new_particles = np.array(particles).tolist()
        GUI.showParticles(new_particles)
        control = (HAL.getPose3d().x - aux_pos_x, HAL.getPose3d().y - aux_pos_y, HAL.getPose3d().yaw - aux_pos_yaw)
        if(noise == True):
            noise_free_measurements = HAL.getLaserData().values
            noisy_measurements = noise_free_measurements + np.random.normal(0, config.RADAR_NOISE_STD, len(noise_free_measurements))
            #console.print("Hace correctamente el ruido")
        if(show_particles == True):
            new_state, new_v = np.zeros(particles.shape), np.zeros((particles.shape[0], 2))
            #console.print("Hace correctamente el El show")
            for i in range(particles.shape[0]):
                robot_thetha = particles[i][2] + control[2]
                control = np.array(control)
                theta_control = np.arctan2(control[1], control[0])
                diff_theta = robot_thetha - theta_control

                c, s = np.cos(diff_theta), np.sin(diff_theta)
                rot = np.array(((c, -s), (s, c)))
                vcontrol = rot.dot(control[:2])
                nx = particles[i][0] + vcontrol[0]
                ny = particles[i][1] + vcontrol[1]

                if noise:
                    nx = nx + np.random.normal(0, config.SYSTEM_MOTION_NOISE[0])
                    ny = ny + np.random.normal(0, config.SYSTEM_MOTION_NOISE[1])

                v = (nx - particles[i][0], ny - particles[i][1])

                ntheta = np.arctan2(v[1], v[0])

                new_state_p = (
                    nx,
                    ny,
                    ntheta
                )
                #particle_positions and particle_velocities
                new_state[i] = new_state_p
                new_v[i] = v
            mm = partial(env.measurement_model, noisy_measurements, noise_free_measurements)
            positions = new_state.tolist()
            p = Pool(100)
            console.print("Esto es lo que tarda tanto")
            weights = p.map(mm, positions)
            console.print("Que es lo que tarda tanto?")
            weights = np.array(weights)
            total_weights = np.sum(weights)
            if total_weights == 0:
                is_weight_valid = False
            else:
                is_weight_valid = True
                important_weights = weights / total_weights
            
            if(is_weight_valid):
                particle_resampling_indicies = np.random.choice(new_state.shape[0], new_state.shape[0], replace=True, p=important_weights)
                particle_resampling = new_state[particle_resampling_indicies]
            else:
                particles_xy_indices = np.random.choice(traversable_area.shape[0], size=no_particles, replace=True)
                particles_xy = traversable_area[particles_xy_indices]
                particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)
                particle_resampling = np.hstack([particles_xy, particles_theta])
            
            position_differences = particle_resampling[:, :2] - np.array(robot_pos)[:2]
            #console.print("El problema radica aqui?")
            dists = np.sum(np.abs(position_differences) ** 2, axis=-1) ** (1. / 2)
            distance_differences.append((np.mean(dists), np.std(dists)))
            angle_diffs = (particle_resampling[:, 2] - robot_pos[2]).reshape(-1, 1)
            angle_diffs = np.hstack((angle_diffs, -angle_diffs)) % (2*np.pi)
            angle_diffs = np.min(angle_diffs, axis=1)
            angle_differences.append((np.mean(angle_diffs), np.std(angle_diffs)))
            approximated_robot_x, approximated_robot_y = np.mean(particle_resampling[:, 0]), np.mean(particle_resampling[:, 1])
            GUI.showEstimatedPose((approximated_robot_x, approximated_robot_y))
