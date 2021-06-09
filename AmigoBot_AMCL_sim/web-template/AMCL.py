import config
import numpy as np
import environment
import math
from hal import HAL
from map import Map
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
current_robot_pos = []
prev_robot_pos = [0, 0, 0]


distance_differences = []
angle_differences = []

class AMCL:
    def __init__(self, no_particles=15, no_random_particles=0, localization=False):
        global current_robot_pos, prev_robot_pos

        self.no_particles = no_particles
        self.no_random_particles = no_random_particles
        self.localization = localization
        pose3d_object = ListenerPose3d("/robot0/odom")
        laser_object = ListenerLaser("/robot0/laser_1")
        self.map = Map(laser_object, pose3d_object)
        self.hal = HAL()
        self.scene = 'scene-1'
        self.scene = environment.Environment(self.scene, no_particles=self.no_particles)
        mm = self.scene.map
        self.total_frames=None
        self.show_particles=True
        self.pose = self.hal.pose3d.getPose3d()
        current_robot_pos = (self.pose.x, self.pose.y, self.pose.yaw)

    def animate(self, laser_beam, radar_src):
        global current_robot_pos, prev_robot_pos, distance_differences, angle_differences
        distance = []
        #control = (current_robot_pos[0] - prev_robot_pos[0], current_robot_pos[1] - prev_robot_pos[1], current_robot_pos[2] - prev_robot_pos[2])
        control = (0,0,0)
        robot_pos = (self.pose.x, self.pose.y, self.pose.yaw)

        #radar_src, radar_dest = self.scene.build_radar_beams(robot_pos) #Comentar, funcion que dibuja los rayos laser. Ya implementado en el sistema
        # print("Posicion del radar_src", radar_src.shape) (2x11) (El mismo valor 11 veces)
        # print("Posicion del radar_dest", radar_dest.shape) (2x11)
        for i, point in enumerate(laser_beam):
            distance.append((0,0))
            distance[i] = math.sqrt((point[0] - radar_src[0])**2 + (point[1] - radar_src[1])**2)
        #noise_free_measurements, _, radar_rays = self.scene.vraytracing(np.array(radar_src), np.array(laser_beam)) #Comentar, me parece que traza las lineas de los rayos
        #print("noise_free_measurements", len(noise_free_measurements))
        noise_free_measurements = distance
        noisy_measurements = noise_free_measurements + np.random.normal(0, config.RADAR_NOISE_STD, len(noise_free_measurements))

        if self.show_particles:
            particle_positions, particle_velocities = self.scene.vperform_control(self.scene.particles, control)
            #print("Llega hasta el vmeasurement_model")
            is_weight_valid, important_weights = self.scene.vmeasurement_model(particle_positions, noisy_measurements)

            if is_weight_valid:
                particle_resampling_indicies = np.random.choice(particle_positions.shape[0], particle_positions.shape[0], replace=True, p=important_weights)
                particle_resampling = particle_positions[particle_resampling_indicies]
            else:
                particle_resampling = self.scene.uniform_sample_particles(self.no_particles)

            self.scene.particles = particle_resampling

            position_differences = self.scene.particles[:, :2] - np.array(robot_pos)[:2]

            dists = np.sum(np.abs(position_differences) ** 2, axis=-1) ** (1. / 2)

            distance_differences.append((np.mean(dists), np.std(dists)))

            angle_diffs = (self.scene.particles[:, 2] - robot_pos[2]).reshape(-1, 1)
            angle_diffs = np.hstack((angle_diffs, -angle_diffs)) % (2*np.pi)
            angle_diffs = np.min(angle_diffs, axis=1)
            angle_differences.append((np.mean(angle_diffs), np.std(angle_diffs)))

            approximated_robot_x, approximated_robot_y = np.mean(self.scene.particles[:, 0]), np.mean(self.scene.particles[:, 1])
        prev_robot_pos = current_robot_pos
        #print("Que es lo que devuelven estas particulas: \n", np.array(particle_positions).tolist(), "\n")
        #print("Que es lo que devuelven estas self.scene.particles: \n", np.array(self.scene.particles).tolist(), "\n")
        return np.array(approximated_robot_x).tolist(), np.array(approximated_robot_y).tolist(), np.array(particle_positions).tolist()
        #total_frames = self.scene.total_frames if total_frames is None else total_frames



