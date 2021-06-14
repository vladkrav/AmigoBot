no_particles = 500
no_random_particles = 500
type_localization = True
map = ((729, 769))
show_particles = True

RADAR_WITDH = 0.5
RADAR_MAX_LENGTH = 266 #50
RADAR_NOISE_STD = 0.2

SYSTEM_MOTION_NOISE = [0.01, 0.01, 0.00]
SYSTEM_MEASURE_MODEL_LOCAL_NOISE_STD = 20
SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD = 0.7 # 1 mean traversable
SYSTEM_NO_SENSORS = 400
SYSTEM_MC_INTEGRAL_GRID = 0.1
SYSTEM_MC_GRIDS = np.arange(0, RADAR_MAX_LENGTH + SYSTEM_MC_INTEGRAL_GRID, SYSTEM_MC_INTEGRAL_GRID)

ROBOT_MAX_MOVE_DISTANCE = 2.5
ROBOT_DEFAULT_X = 0
ROBOT_DEFAULT_Y = 0
ROBOT_DEFAULT_THETA = 0
ROBOT_APPROXIMATED_DIRECTION_LENGTH = 5
ROBOT_SYMBOL_SIZE = 6
ROBOT_STEP_SIZE = 0.3
ROBOT_STEP_THETA_SIZE = np.pi / 2.0
ROBOT_DIAMETER = 5
ROBOT_MOVEMENT_MODEL_PARAMS = [0.01, 0.01]

SENSOR_SYMBOL_SIZE = 5

PARTICLE_SYMBOL_SIZE = 2
PARTICLE_DIRECTION_DISTANCE = 3
PARTICLE_OPACITY = 0.5
particles = uniform_sample_particles(no_particles)

def animate(self, laser_beam, radar_src):
    global current_robot_pos, prev_robot_pos, distance_differences, angle_differences
    distance = []
    control = (0,0,0)
    robot_pos = (self.pose.x, self.pose.y, self.pose.yaw)
    for i, point in enumerate(laser_beam):
        distance.append((0,0))
        distance[i] = math.sqrt((point[0] - radar_src[0])**2 + (point[1] - radar_src[1])**2)

    noise_free_measurements = distance
    noisy_measurements = noise_free_measurements + np.random.normal(0, RADAR_NOISE_STD, len(noise_free_measurements))

    if self.show_particles:
        particle_positions, particle_velocities = self.scene.vperform_control(self.scene.particles, control)
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
    return np.array(approximated_robot_x).tolist(), np.array(approximated_robot_y).tolist(), np.array(particle_positions).tolist()

def uniform_sample_particles(self, no_particles):
    particles_xy_indices = np.random.choice(self.traversable_area.shape[0], size=no_particles, replace=True)
    particles_xy = self.traversable_area[particles_xy_indices]
    particles_theta = np.random.uniform(0.0, 2*np.pi, (no_particles, 1)) % (2*np.pi)

    res = np.hstack([particles_xy, particles_theta])
    return res
def vperform_control(self, vpos, control):
    new_state, new_v = np.zeros(vpos.shape), np.zeros((vpos.shape[0], 2))

    for i in range(vpos.shape[0]):
        new_state[i], new_v[i] = self.perform_control(vpos[i], control, noisy_env=True)

    return new_state, new_v

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
        nx = nx + np.random.normal(0, SYSTEM_MOTION_NOISE[0])
        ny = ny + np.random.normal(0, SYSTEM_MOTION_NOISE[1])

    v = (nx - pos[0], ny - pos[1])

    ntheta = np.arctan2(v[1], v[0])

    new_state = (
        nx,
        ny,
        ntheta
    )
    return new_state, v
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
        
def measurement_model(self, pos, observed_measurements):
    if self.map_with_safe_boundary[int(pos[1]), int(pos[0])] < SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD:
        return 0.0

    radar_src, radar_dest = self.build_radar_beams(pos)
    noise_free_measurements, _, radar_rays = self.vraytracing(radar_src, radar_dest)

    particle_measurements = noise_free_measurements

    qs = self._vmeasurement_model_p_hit(observed_measurements, particle_measurements)

    return np.prod(qs)

def vraytracing(self, srcs, dests, **kwargs):

    distances = np.zeros(srcs.shape[1])
    positions = np.zeros(srcs.shape)
    rel_positions = np.zeros(srcs.shape)

    for i in range(srcs.shape[1]):
        distances[i], positions[:, i], rel_positions[:, i] = self.raytracing(srcs[:, i], dests[:, i], **kwargs)

    return distances, positions, rel_positions
def raytracing(self, src, dest, num_samples=10):

    dx = np.where(src[0] < dest[0], 1, -1)
    dy = np.where(src[1] < dest[1], 1, -1)
    x_steps = src[0] + dx*np.linspace(0, np.abs(src[0]-dest[0]), num_samples)
    y_steps = src[1] + dy*np.linspace(0, np.abs(src[1]-dest[1]), num_samples)

    x_steps_int = np.clip(np.round(x_steps).astype(np.int16), 0, self.map.shape[1]-1)
    y_steps_int = np.clip(np.round(y_steps).astype(np.int16), 0, self.map.shape[0]-1)

    mark = np.zeros(self.map.shape)
    mark[y_steps_int, x_steps_int] = 1

    collided_map = self.map[y_steps_int, x_steps_int] < SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD

    if np.sum(collided_map) > 0:
        collisions = np.nonzero(collided_map)
        pos = collisions[0][0]
        position = np.array((x_steps[pos], y_steps[pos]))
        distance = np.linalg.norm([position[0] - src[0], position[1] - src[1]])
    else:
        position = dest
        distance = RADAR_MAX_LENGTH

    rel_position = [
        (position[0] - src[0]),
        (position[1] - src[1]),
    ]
    return distance, position, rel_position