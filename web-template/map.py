import numpy as np
import math
from math import pi as pi
from hal import HAL
class Map:
	def __init__(self,laser_object, pose3d):
		self.pose3d = pose3d
		self.payload = {}
		self.hal = HAL()
		self.laser_topic = laser_object
		self.sonar_0 = {'pos_x': 0.076, 'pos_y': 0.1, 'orientation': 1.5708}
		self.sonar_1 = {'pos_x': 0.125, 'pos_y': 0.075, 'orientation': 0.715585}
		self.sonar_2 = {'pos_x': 0.15, 'pos_y': 0.03, 'orientation': 0.261799}
		self.sonar_3 = {'pos_x': 0.15, 'pos_y': -0.03, 'orientation': -0.261799}
		self.sonar_4 = {'pos_x': 0.125, 'pos_y': -0.075, 'orientation': -0.715585}
		self.sonar_5 = {'pos_x': 0.076, 'pos_y': -0.1, 'orientation': -1.5708}
		self.sonar_6 = {'pos_x': -0.14, 'pos_y': -0.058, 'orientation': -2.53073}
		self.sonar_7 = {'pos_x': -0.14, 'pos_y': 0.058, 'orientation': 2.53073}
		self.laser = {'pos_x': 0.09, 'pos_y': 0.0, 'orientation': 0.0}
	
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

	# def getRobotCoordinates(self):
	# 	pose = self.pose3d.getPose3d()
	# 	x = pose.x
	# 	y = pose.y
		
	# 	scale_y = 33.25; offset_y = 0.33
	# 	y = 729 - (scale_y * y + offset_y)
		
	# 	scale_x = 33.25; offset_x = 0.33
	# 	x = scale_x * x + offset_x
		
	# 	return x, y

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
	
	def setRobotValues(self):
		pose = self.pose3d.getPose3d()
		yaw = pose.yaw
		x = pose.x
		y = pose.y
		z = 0
		radius = 0.15
		base = radius/2
		coord_contorno = []
		coord_robot = []
		# Vertice 1
		alfa = math.asin((base)/radius)
		xr, yr = self.local2relative(0, 0, radius, math.pi - alfa)
		xg, yg = self.relative2global(x, y, xr, yr, yaw)
		xc, yc = self.global2canvas(xg, yg)
		coord_contorno.append((0,0))
		coord_contorno [0] = (xc, yc)
	 	# Vertice 2
		xr, yr = self.local2relative(0, 0, radius, math.pi + alfa)
		xg, yg = self.relative2global(x, y, xr, yr, yaw)
		xc, yc = self.global2canvas(xg, yg)
		coord_contorno.append((0,0))
		coord_contorno [1] = (xc, yc)
		# Vertice 3
		xr, yr = self.local2relative(0, 0, radius, 0)
		xg, yg = self.relative2global(x, y, xr, yr, yaw)
		xc, yc = self.global2canvas(xg, yg)
		coord_contorno.append((0,0))
		coord_contorno [2] = (xc, yc)

	 	# Position of the robot in the canvas
		xc, yc = self.global2canvas(x, y)
		coord_robot.append((0,0))
		coord_robot = (xc, yc)
		return coord_robot, coord_contorno
	
	def setSonarValues(self):
		self.sonares = []
		self.dist = []
		# Configuration of the sonar x, y (in metres) and orientation (in radians)
		self.sonares.append((0,0,0))
		self.sonares[0] = (0.076, 0.1, 1.5708)
		self.sonares.append((0,0,0))
		self.sonares[1] = (0.125, 0.075, 0.715585)
		self.sonares.append((0,0,0))
		self.sonares[2] = (0.15, 0.03, 0.261799)
		self.sonares.append((0,0,0))
		self.sonares[3] = (0.15, -0.03, -0.261799)
		self.sonares.append((0,0,0))
		self.sonares[4] = (0.125, -0.075, -0.715585)
		self.sonares.append((0,0,0))
		self.sonares[5] = (0.076, -0.1, -1.5708)
		self.sonares.append((0,0,0))
		self.sonares[6] = (-0.14, -0.058, -2.53073)
		self.sonares.append((0,0,0))
		self.sonares[7] = (-0.14, 0.058, 2.53073)
		# SONAR 0
		self.dist.append(0)
		self.dist[0] = self.hal.sonar_0.getSonarData().distances
		if self.dist[0] == float("inf"):
			self.dist[0] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[0] == float("-inf"):
			self.dist[0] = self.hal.sonar_0.getSonarData().minRange
		# SONAR 1
		self.dist.append(0)
		self.dist[1] = self.hal.sonar_1.getSonarData().distances
		if self.dist[1] == float("inf"):
			self.dist[1] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[1] == float("-inf"):
			self.dist[1] = self.hal.sonar_0.getSonarData().minRange
		# SONAR 2
		self.dist.append(0)
		self.dist[2] = self.hal.sonar_2.getSonarData().distances
		if self.dist[2] == float("inf"):
			self.dist[2] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[2] == float("-inf"):
			self.dist[2] = self.hal.sonar_0.getSonarData().minRange
		#SONAR 3
		self.dist.append(0)
		self.dist[3] = self.hal.sonar_3.getSonarData().distances
		if self.dist[3] == float("inf"):
			self.dist[3] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[3] == float("-inf"):
			self.dist[3] = self.hal.sonar_0.getSonarData().minRange
		#SONAR 4
		self.dist.append(0)
		self.dist[4] = self.hal.sonar_4.getSonarData().distances
		if self.dist[4] == float("inf"):
			self.dist[4] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[4] == float("-inf"):
			self.dist[4] = self.hal.sonar_0.getSonarData().minRange
		#SONAR 5
		self.dist.append(0)
		self.dist[5] = self.hal.sonar_5.getSonarData().distances
		if self.dist[5] == float("inf"):
			self.dist[5] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[5] == float("-inf"):
			self.dist[5] = self.hal.sonar_0.getSonarData().minRange
		#SONAR 6
		self.dist.append(0)
		self.dist[6] = self.hal.sonar_6.getSonarData().distances
		if self.dist[6] == float("inf"):
			self.dist[6] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[6] == float("-inf"):
			self.dist[6] = self.hal.sonar_0.getSonarData().minRange
		#SONAR 7
		self.dist.append(0)
		self.dist[7] = self.hal.sonar_7.getSonarData().distances
		if self.dist[7] == float("inf"):
			self.dist[7] = self.hal.sonar_0.getSonarData().maxRange
		elif self.dist[7] == float("-inf"):
			self.dist[7] = self.hal.sonar_0.getSonarData().minRange

		pose = self.pose3d.getPose3d()
		yaw = pose.yaw
		x = pose.x
		y = pose.y
		cone = 0.261799
		vertices = []
		sonar_sensor = []
		# Se definen las posiciones de los vertices del triangulo
		for i in range(0,8):
			vertices.append((0,0,0,0))
			distance = self.dist[i]
			hipotenusa = distance/(math.cos(cone/2))
			if(self.sonares[i][2] < 0):
				orientation = self.sonares[i][2] - cone/2
				xr_1, yr_1 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hipotenusa, orientation)
				orientation = self.sonares[i][2] + cone/2
				xr_2, yr_2 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hipotenusa, orientation)
			else:
				orientation = self.sonares[i][2] + cone/2
				xr_1, yr_1 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hipotenusa, orientation)
				orientation = self.sonares[i][2] - cone/2
				xr_2, yr_2 = self.local2relative(self.sonares[i][0], self.sonares[i][1], hipotenusa, orientation)	
			xg_1, yg_1 = self.relative2global(x, y, xr_1, yr_1, yaw)
			xg_2, yg_2 = self.relative2global(x, y, xr_2, yr_2, yaw)
			xc_1, yc_1 = self.global2canvas(xg_1, yg_1)
			xc_2, yc_2 = self.global2canvas(xg_2, yg_2)
			vertices[i] = (xc_1, yc_1, xc_2, yc_2)
		# Se definen las posiciones de los sensores sonar
		for i in range(0,8):
			sonar_sensor.append((0,0))
			xg, yg = self.relative2global(x, y, self.sonares[i][0], self.sonares[i][1], yaw)
			xc, yc = self.global2canvas(xg, yg)
			sonar_sensor[i] = (xc, yc)
		return vertices, sonar_sensor

	def setLaserValues(self):
		pose = self.pose3d.getPose3d()
		yaw = pose.yaw
		x = pose.x
		y = pose.y
		xg, yg = self.relative2global(x, y, self.laser['pos_x'], self.laser['pos_y'], yaw)
		xc, yc = self.global2canvas(xg, yg)
		laser_sensor = [xc, yc]
		laser = self.hal.laser.getLaserData()
		laser_beam = []
		for i in range(0,400):
			laser_beam.append((0,0))
			if(i == 0):
				angle = -math.pi
			else:
				angle = angle + 0.015708
			dist = laser.values[i]
			if(dist == float("inf")):
				dist = laser.maxRange
			xr, yr = self.local2relative(self.laser['pos_x'], self.laser['pos_y'], dist, angle)
			xg, yg = self.relative2global(x, y, xr, yr, yaw)
			xc, yc = self.global2canvas(xg, yg)
			laser_beam[i] = (xc, yc)
		return laser_beam, laser_sensor

	def local2relative(self, posx, posy, distance, orientation):
		xr = posx + math.cos(orientation)*distance
		yr = posy + math.sin(orientation)*distance
	
		return xr, yr
	
	def relative2global(self, posx, posy, xr, yr, yaw):
		xg = posx + math.cos(yaw)*xr - math.sin(yaw)*yr
		yg = posy + math.sin(yaw)*xr + math.cos(yaw)*yr
		
		return xg, yg
	
	def global2canvas(self, posx, posy):
		scale_y = 33.25; offset_y = 0.33
		scale_x = 33.25; offset_x = 0.33
		x = scale_x * posx + offset_x
		y = 729 - (scale_y * posy + offset_y)
	
		return x, y