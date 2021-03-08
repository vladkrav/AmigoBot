import numpy as np
import math
from math import pi as pi
import cv2

class Map:
	def __init__(self,laser_object, pose3d):
		self.pose3d = pose3d
		self.payload = {}
		self.laser_topic = laser_object
	
	def RTx(self, angle, tx, ty, tz):
		RT = np.matrix([[1, 0, 0, tx], [0, math.cos(angle), -math.sin(angle), ty], 
						[0, math.sin(angle), math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTy(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty],
						[-math.sin(angle), 0, math.cos(angle), tz], [0, 0, 0, 1]])
		return RT
		
	def RTz(self, angle, tx, ty, tz):
		RT = np.matrix([[math.cos(angle), -math.sin(angle), 0, tx], [math.sin(angle), math.cos(angle), 0, ty],
						[0, 0, 1, tz], [0, 0, 0, 1]])
		return RT
		
	def RTVacuum(self):
		RTz = self.RTz(pi/2, 50, 70, 0)
		return RTz

	def getRobotCoordinates(self):
		pose = self.pose3d.getPose3d()
		x = pose.x
		y = pose.y
		
		#scale_y = -10; offset_y = 174
		scale_y = -10; offset_y = 174.5
		y = scale_y * y + offset_y
		
		#scale_x = 10; offset_x = 10
		scale_x = 10; offset_x = 15
		x = scale_x * x + offset_x
		
		return x, y

	def getRobotAngle(self):
		pose = self.pose3d.getPose3d()
		rt = pose.yaw

		ty = math.cos(-rt) - math.sin(-rt)
		tx = math.sin(-rt) + math.cos(-rt)

		return tx, ty

	# Function to reset
	def reset(self):
		# Nothing to do, service takes care!
		pass
	
	def setLaserValues(self):
		# Init laser array
		laser = []
		self.laser = self.laser_topic.getLaserData()

		if(self.laser):
			#angle = int(round(math.degrees(self.laser.maxAngle)))
			angle = int(round(math.degrees(2*math.pi)))
		for i in range(angle):
			laser.append((0,0))
		if(self.laser):
			for i in range(angle):
				dist = self.laser.values[i]
				angle = math.radians(i)
				if(dist == float("inf")):
					dist = self.laser.maxRange
				laser[i] = (dist, angle)
		return laser, self.laser.maxRange
