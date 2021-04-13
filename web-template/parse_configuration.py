import json
import math

class Config:
    def __init__(self):
        with open('config_real.json') as file:
            self.config = json.load(file)
        self.pos_x = self.config['CONFIGURATION']['POS_X']
        self.pos_y = self.config['CONFIGURATION']['POS_Y']
        self.orientation = math.radians(self.config['CONFIGURATION']['ORIENTATION'])
        self.sonar_0 = self.config['CONFIGURATION']['SONAR_0']
        self.sonar_1 = self.config['CONFIGURATION']['SONAR_1']
        self.sonar_2 = self.config['CONFIGURATION']['SONAR_2']
        self.sonar_3 = self.config['CONFIGURATION']['SONAR_3']
        self.sonar_4 = self.config['CONFIGURATION']['SONAR_4']
        self.sonar_5 = self.config['CONFIGURATION']['SONAR_5']
        self.sonar_6 = self.config['CONFIGURATION']['SONAR_6']
        self.sonar_7 = self.config['CONFIGURATION']['SONAR_7']
        self.laser = self.config['CONFIGURATION']['LASER']
        self.topic_pose = self.config['CONFIGURATION']['TOPIC_POSE']
        self.topic_motors = self.config['CONFIGURATION']['TOPIC_MOTOR']
        self.topic_laser = self.config['CONFIGURATION']['TOPIC_LASER']
        self.topic_sonar_0 = self.config['CONFIGURATION']['TOPIC_SONAR0']
        self.topic_sonar_1 = self.config['CONFIGURATION']['TOPIC_SONAR1']
        self.topic_sonar_2 = self.config['CONFIGURATION']['TOPIC_SONAR2']
        self.topic_sonar_3 = self.config['CONFIGURATION']['TOPIC_SONAR3']
        self.topic_sonar_4 = self.config['CONFIGURATION']['TOPIC_SONAR4']
        self.topic_sonar_5 = self.config['CONFIGURATION']['TOPIC_SONAR5']
        self.topic_sonar_6 = self.config['CONFIGURATION']['TOPIC_SONAR6']
        self.topic_sonar_7 = self.config['CONFIGURATION']['TOPIC_SONAR7']
