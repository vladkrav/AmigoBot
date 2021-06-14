import numpy as np
import config
import matplotlib.image as mpimg
from scipy import signal
from functools import partial
from multiprocess import Pool
# import copyreg, copy, pickle


from scipy import stats

import logging

logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%d-%m-%Y:%H:%M:%S',
    level=config.LOG_LEVEL)


class Environment(object):
    def __init__(self, scene_name, no_particles=20):

        self.scene_name = scene_name # Se guarda el nombre de la escena para cargar desde config.py
        self.no_particles = no_particles # Se guarda el numero de particulas 

        map_name = 'scenes/%s.png' % config.SCENCES[self.scene_name]['map']
        self.map = mpimg.imread(map_name)[::-1, :, 0] # Se obtienen las dimensiones en px del mapa
        # print("Que es lo que tiene la variable self.map", self.map[350]) # 1 indica que la posicion esta libre un 0 que esta ocupada y un 0.49803922 que no se conoce
        mark = np.ones((config.ROBOT_DIAMETER, config.ROBOT_DIAMETER)) # matriz de unos de 5x5 (radio del robot)

        self.convolve_mark = (signal.convolve2d(1-self.map[:, :], mark, mode='same', boundary='fill', fillvalue=0)) / np.sum(mark)
        print("cuanto es el self.convolve_mark\n", self.convolve_mark[133][266], "\n")
        self.convolve_mark_overlay = np.copy(self.map)

        threshold = 1/np.sum(mark)
        # print("cuanto es el threshold\n", threshold, "\n")
        self.convolve_mark_overlay[self.convolve_mark > threshold] = 0.2 # 0.2
        # print("Que es self.convolve_mark_overlay\n", self.convolve_mark_overlay, "\n")

        self.map_with_safe_boundary = np.copy(self.map)
        self.map_with_safe_boundary[self.convolve_mark > threshold] = 0.0 # zero is obstacle.

        self.paths = config.SCENCES[scene_name]['paths'] # Array de la ruta...no lo necesito ya que lo muevo yo

        self.controls = [Environment._build_control(l) for l in self.paths] # Se crea el control...no lo necesito ya que lo muevo yo

        total_controls = np.sum([len(a) for a in self.controls]) # Numero total de controles...no lo necesito
        #logging.info('we have %d controls' % total_controls)

        if len(self.paths) > 1: # como no tengo el control del robot...esto ya no se necesita tampoco...
            self.kidnapping_occur_at = len(self.controls[0])
        else:
            self.kidnapping_occur_at = None

        self.total_frames = (len(self.paths) - 1) + total_controls # Numero de iteraciones...no lo necesito...iterar siempre

        self.no_sensors = config.SYSTEM_NO_SENSORS # Numero de rayos laser
        self.radar_thetas = (np.arange(0, self.no_sensors) - self.no_sensors // 2)*(np.pi/self.no_sensors)

        # Area transitable #0.7
        self.traversable_area = np.stack(np.nonzero(1 - (self.map_with_safe_boundary.T < 1)), axis=1)
        self.particles = self.uniform_sample_particles(self.no_particles) # realiza la ubicacion de las particulas
        
        self.control_group_idx = 0
        self.state_idx = 0

        self.total_move = 0

        self._vmeasurement_model_p_hit = np.vectorize(self._measurement_model_p_hit)
    # Distribuye uniformemente por el mapa las particulas
    def uniform_sample_particles(self, no_particles):
        particles_xy_indices = np.random.choice(self.traversable_area.shape[0], size=no_particles, replace=True)
        particles_xy = self.traversable_area[particles_xy_indices]
        particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)

        res = np.hstack([particles_xy, particles_theta])
        return res

    def get_control(self):
        self.total_move = self.total_move + 1

        #logging.info('path %d' % self.control_group_idx)

        if self.state_idx >= len(self.controls[self.control_group_idx]):
            #logging.info('..........')
            self.control_group_idx = self.control_group_idx + 1
            self.state_idx = 0
            teleport_pos = self.paths[self.control_group_idx][0]
            print(teleport_pos)
            return teleport_pos, None

        control = self.controls[self.control_group_idx][self.state_idx]
        self.state_idx = self.state_idx + 1

        return None, control

    def perform_control(self, pos, control, noisy_env=True):

        robot_thetha = pos[2] + control[2]
        control = np.array(control)
        theta_control = np.arctan2(control[1], control[0])
        diff_theta = robot_thetha - theta_control

        c, s = np.cos(diff_theta), np.sin(diff_theta)
        rot = np.array(((c, -s), (s, c)))
        vcontrol = rot.dot(control[:2])
        nx = pos[0] + vcontrol[0]
        ny = pos[1] + vcontrol[1]

        if noisy_env:
            nx = nx + np.random.normal(0, config.SYSTEM_MOTION_NOISE[0])
            ny = ny + np.random.normal(0, config.SYSTEM_MOTION_NOISE[1])

        v = (nx - pos[0], ny - pos[1])

        ntheta = np.arctan2(v[1], v[0])

        new_state = (
            nx,
            ny,
            ntheta
        )

        # logging.debug('-------')
        # logging.debug('control')
        # logging.debug(control)
        # logging.debug('v')
        # logging.debug(v)
        # logging.debug('old state')
        # logging.debug(pos)
        # logging.debug("new state")
        # logging.debug(new_state)

        return new_state, v

    def vperform_control(self, vpos, control):
        print("Cual es la dimension de vpos.shape\n",vpos.shape, "\n")
        new_state, new_v = np.zeros(vpos.shape), np.zeros((vpos.shape[0], 2))

        for i in range(vpos.shape[0]):
            new_state[i], new_v[i] = self.perform_control(vpos[i], control, noisy_env=True)

        return new_state, new_v

    def raytracing(self, src, dest, num_samples=10):
        #logging.debug('src %s -> dest %s ' % (','.join(src.astype(str)), ','.join(dest.astype(str))))

        dx = np.where(src[0] < dest[0], 1, -1)
        dy = np.where(src[1] < dest[1], 1, -1)
        x_steps = src[0] + dx*np.linspace(0, np.abs(src[0]-dest[0]), num_samples)
        y_steps = src[1] + dy*np.linspace(0, np.abs(src[1]-dest[1]), num_samples)

        x_steps_int = np.clip(np.round(x_steps).astype(np.int16), 0, self.map.shape[1]-1)
        y_steps_int = np.clip(np.round(y_steps).astype(np.int16), 0, self.map.shape[0]-1)

        mark = np.zeros(self.map.shape)
        mark[y_steps_int, x_steps_int] = 1

        collided_map = self.map[y_steps_int, x_steps_int] < config.SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD

        if np.sum(collided_map) > 0:
            collisions = np.nonzero(collided_map)
            pos = collisions[0][0]
            position = np.array((x_steps[pos], y_steps[pos]))
            #logging.debug('    collided pos %s' % ','.join(position.astype(str)))
            distance = np.linalg.norm([position[0] - src[0], position[1] - src[1]])
        else:
            position = dest
            distance = config.RADAR_MAX_LENGTH

        rel_position = [
            (position[0] - src[0]),
            (position[1] - src[1]),
        ]
        #logging.debug('  position %s' % ','.join(np.array(position).astype(str)))
        #logging.debug('  rel position %s' % ','.join(np.array(rel_position).astype(str)))

        return distance, position, rel_position

    def vraytracing(self, srcs, dests, **kwargs):

        distances = np.zeros(srcs.shape[1])
        positions = np.zeros(srcs.shape)
        rel_positions = np.zeros(srcs.shape)

        for i in range(srcs.shape[1]):
            distances[i], positions[:, i], rel_positions[:, i] = self.raytracing(srcs[:, i], dests[:, i], **kwargs)

        return distances, positions, rel_positions

    def build_radar_beams(self, pos):
        radar_src = np.array([[pos[0]] * self.no_sensors, [pos[1]] * self.no_sensors])

        radar_theta = self.radar_thetas + pos[2]
        radar_rel_dest = np.stack(
            (
                np.cos(radar_theta)*config.RADAR_MAX_LENGTH,
                np.sin(radar_theta)*config.RADAR_MAX_LENGTH
            ), axis=0
        )

        radar_dest = np.zeros(radar_rel_dest.shape)
        radar_dest[0, :] = np.clip(radar_rel_dest[0, :] + radar_src[0, :], 0, self.map.shape[1])
        radar_dest[1, :] = np.clip(radar_rel_dest[1, :] + radar_src[1, :], 0, self.map.shape[0])

        return radar_src, radar_dest

    def measurement_model(self, pos, observed_measurements):
        if self.map_with_safe_boundary[int(pos[1]), int(pos[0])] < config.SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD:
            return 0.0

        radar_src, radar_dest = self.build_radar_beams(pos)
        noise_free_measurements, _, radar_rays = self.vraytracing(radar_src, radar_dest)

        particle_measurements = noise_free_measurements

        qs = self._vmeasurement_model_p_hit(observed_measurements, particle_measurements)

        return np.prod(qs)

    def vmeasurement_model(self, positions, observed_measurements):
        mm = partial(self.measurement_model, observed_measurements=observed_measurements)

        positions = positions.tolist()

        p = Pool(10)
        #with Pool(10) as p:
        weights = p.map(mm, positions)

        weights = np.array(weights)
        total_weights = np.sum(weights)

        if total_weights == 0:
            #logging.debug('all weights are zero')
            return False, None
        else:
            return True, weights / total_weights


    @classmethod
    def _measurement_model_p_hit(cls, z, z_star):
        dd = np.concatenate([config.SYSTEM_MC_GRIDS, [z]])

        prob = stats.norm.pdf(dd, loc=z_star, scale=config.SYSTEM_MEASURE_MODEL_LOCAL_NOISE_STD)

        normalizers = np.sum(prob[:-1])

        return prob[-1] / normalizers

    @staticmethod
    def _build_control(landmarks):

        controls = []
        for i in range(1, len(landmarks)):
            prev_lm = landmarks[i-1]
            curr_lm = landmarks[i]

            x_move = [config.ROBOT_MAX_MOVE_DISTANCE]*int(np.abs((curr_lm[0] - prev_lm[0]) / config.ROBOT_MAX_MOVE_DISTANCE))
            y_move = [config.ROBOT_MAX_MOVE_DISTANCE]*int(np.abs((curr_lm[1] - prev_lm[1]) / config.ROBOT_MAX_MOVE_DISTANCE))
            max_moves = np.max([len(x_move), len(y_move)])

            x_move = x_move + [0]*(max_moves - len(x_move))
            y_move = y_move + [0]*(max_moves - len(y_move))
            theta_move = [0]*(max_moves-1) + [curr_lm[2] - prev_lm[2]]

            cc = list(zip(x_move, y_move, theta_move))
            controls = controls + cc
        return controls
