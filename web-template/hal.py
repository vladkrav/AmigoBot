import rospy
import cv2
import threading
import time
from datetime import datetime

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser
from interfaces.sonar import ListenerSonar

# Hardware Abstraction Layer
class HAL:
    
    def __init__(self):
        
    	rospy.init_node("HAL")
    
    	self.motors = PublisherMotors("/robot0/cmd_vel", 0.2, 0.2)
    	self.pose3d = ListenerPose3d("/robot0/odom")
        self.sonar_0 = ListenerSonar("/robot0/sonar_0")
        self.sonar_1 = ListenerSonar("/robot0/sonar_1")
        self.sonar_2 = ListenerSonar("/robot0/sonar_2")
        self.sonar_3 = ListenerSonar("/robot0/sonar_3")
        self.sonar_4 = ListenerSonar("/robot0/sonar_4")
        self.sonar_5 = ListenerSonar("/robot0/sonar_5")
        self.sonar_6 = ListenerSonar("/robot0/sonar_6")
        self.sonar_7 = ListenerSonar("/robot0/sonar_7")
    	self.laser = ListenerLaser("/robot0/laser_1")
        self.sonar = [self.sonar_0, self.sonar_1, self.sonar_2, self.sonar_3, self.sonar_4, self.sonar_5, self.sonar_6, self.sonar_7]
    	
    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
