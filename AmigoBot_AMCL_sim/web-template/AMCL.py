import config
import numpy as np
import environment
from hal import HAL

robot_pos = None


distance_differences = []
angle_differences = []

class AMCL:
    def __init__(self, no_particles=10, no_random_particles=0, localization=False):
        self.no_particles = no_particles
        self.no_random_particles = no_random_particles
        self.localization = localization


        self.hal = HAL()
        global robot_pos
        self.scene = 'scene-1'
        self.scene = environment.Environment(self.scene, no_particles)
        mm = self.scene.map
        self.total_frames=None
        self.show_particles=True
        self.pose = self.hal.pose3d.getPose3d()
        robot_pos = (self.pose.x, self.pose.y, self.pose.yaw)

    def animate(self):
        global robot_pos, distance_differences, angle_differences

        teleport_pos, control = self.scene.get_control()

        if teleport_pos:
            robot_pos = teleport_pos
        else:
            robot_pos = (self.pose.x, self.pose.y, self.pose.yaw)

            radar_src, radar_dest = self.scene.build_radar_beams(robot_pos) #Comentar, funcion que dibuja los rayos laser. Ya implementado en el sistema
            # print("Posicion del radar_src", radar_src.shape) (2x11) (El mismo valor 11 veces)
            # print("Posicion del radar_dest", radar_dest.shape) (2x11)
            noise_free_measurements, _, radar_rays = self.scene.vraytracing(radar_src, radar_dest) #Comentar, me parece que traza las lineas de los rayos

            noisy_measurements = noise_free_measurements + np.random.normal(0, config.RADAR_NOISE_STD, noise_free_measurements.shape[0])

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

            return approximated_robot_x, approximated_robot_y, self.scene.particles
        #total_frames = self.scene.total_frames if total_frames is None else total_frames



