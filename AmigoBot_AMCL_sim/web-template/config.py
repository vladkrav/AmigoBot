import numpy as np
import logging

RADAR_COLOR = 'b'
RADAR_WITDH = 0.5
RADAR_MAX_LENGTH = 266 #50
RADAR_NOISE_STD = 0.2 / 0.03

SYSTEM_MOTION_NOISE = [0.1 / 0.03, 0.1 / 0.03, 0.00 / 0.03]
SYSTEM_MEASURE_MODEL_LOCAL_NOISE_STD = 20
SYSTEM_MAP_OCCUPIED_AREA_THRESHOLD = 0.7 # 1 mean traversable
SYSTEM_NO_SENSORS = 400
SYSTEM_MC_INTEGRAL_GRID = 0.1
SYSTEM_MC_GRIDS = np.arange(0, RADAR_MAX_LENGTH + SYSTEM_MC_INTEGRAL_GRID, SYSTEM_MC_INTEGRAL_GRID)

ROBOT_MAX_MOVE_DISTANCE = 2.5
ROBOT_DEFAULT_X = 0
ROBOT_DEFAULT_Y = 0
ROBOT_DEFAULT_THETA = 0
ROBOT_SYMBOL = 'bo'
ROBOT_APPROXIMATED_SYMBOL = 'ro'
ROBOT_APPROXIMATED_DIRECTION_LENGTH = 5
ROBOT_SYMBOL_SIZE = 6
ROBOT_STEP_SIZE = 0.3
ROBOT_STEP_THETA_SIZE = np.pi / 2.0
ROBOT_DIAMETER = 5
ROBOT_MOVEMENT_MODEL_PARAMS = [0.01, 0.01]


SENSOR_SYMBOL = 'gs'
SENSOR_SYMBOL_SIZE = 5

PARTICLE_SYMBOL = 'ro'
PARTICLE_SYMBOL_SIZE = 2
PARTICLE_DIRECTION_DISTANCE = 3
PARTICLE_OPACITY = 0.5

LOG_LEVEL = logging.INFO

SCENCES = {
    'scene-1': {
        'map': 'scene-1',
        'paths': [
            [
                (266, 133-20, 2*np.pi-0.5*np.pi),
                (165, 100-80, np.pi), # next move theta
                (40, 100-80, 0.5*np.pi),
                (40, 100-70, 0.0),
                (110, 100-70, 0.5*np.pi),
                (110, 100-35, np.pi),
                (20, 100-35, np.pi),
            ]
        ]
    },
    'scene-2': {
        'map': 'scene-2',
        'paths': [
            [
                (135, 200-90, np.pi),
                (80, 200-85, 3*np.pi/2),
                (80, 200-125, np.pi),
                (55, 200-125, np.pi/2),
                (55, 200-85, np.pi),
                (25, 200-85, 3*np.pi/2),
                (25, 200-170, np.pi/4),
                (50, 200-170, -np.pi/4),
                (80, 200-170, 0),
                (162.5, 200-170, np.pi/6),
                (190, 200-150, np.pi/2),
                (190, 200-85, np.pi/2),
            ]
        ]
    },
    'scene-8.12': {
        'map': 'scene-8.12',
        'paths': [
            [
                (122.5, 200-145, np.pi),
                (15,  200-145, np.pi/2),
                (15,  200-35, 0),
                (115, 200-35, -np.pi/2),
                (115, 200-80, 0),
                (200,   200-80, 0),
            ]
        ]
    },
    'scene-1-kidnapping': {
        'map': 'scene-1',
        'paths': [
            [
                (165, 100-20, 2*np.pi-0.5*np.pi),
                (165, 100-80, np.pi), # next move theta
                (100, 100-80, 0.5*np.pi),
                # (40, 100-70, 0.0),
                # (110, 100-70, 0.5*np.pi),
                # (110, 100-35, np.pi),
                # (20, 100-35, np.pi),
            ],
            [
                (165, 100-20, 2*np.pi-0.5*np.pi),
                (165, 100-80, np.pi), # next move theta
                (40, 100-80, 0.5*np.pi),
                (40, 100-70, 0.0),
                (110, 100-70, 0.5*np.pi),
                (110, 100-35, np.pi),
                (20, 100-35, np.pi),
            ]
        ]
    },
    'scene-2-kidnapping': {
        'map': 'scene-2',
        'paths': [
            [
                (135, 200-90, np.pi),
                (80, 200-85, 3*np.pi/2),
                (80, 200-125, np.pi),
                (55, 200-125, np.pi/2),
                (55, 200-85, np.pi),
                (25, 200-85, 3*np.pi/2),
                (25, 200-170, np.pi/4),
                (50, 200-170, -np.pi/4),
                (80, 200-170, 0),
                (120, 200-170, np.pi/4),
            ],
            [
                (135, 200-90, np.pi),
                (80, 200-85, 3*np.pi/2),
                (80, 200-125, np.pi),
                (55, 200-125, np.pi/2),
                (55, 200-85, np.pi),
                (25, 200-85, 3*np.pi/2),
                (25, 200-170, np.pi/4),
                (50, 200-170, -np.pi/4),
                (80, 200-170, 0),
                (162.5, 200-170, np.pi/6),
                (190, 200-150, np.pi/2),
                (190, 200-85, np.pi/2),
            ]
        ]
    },
}
