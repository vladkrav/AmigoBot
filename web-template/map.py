import numpy as np
import math
from math import pi as pi
import cv2
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
		self.cone = 0.261799
		self.sonar = [self.sonar_0['orientation'], self.sonar_1['orientation'], self.sonar_2['orientation'], self.sonar_3['orientation'],self.sonar_4['orientation'],self.sonar_5['orientation'],self.sonar_6['orientation'],self.sonar_7['orientation']]
	
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
		
		scale_y = 33.25; offset_y = 0.33
		y = 729 - (scale_y * y + offset_y)
		
		scale_x = 33.25; offset_x = 0.33
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
	
	# def setLaserValues(self):
	# 	# Init laser array
	# 	laser = []
	# 	#self.laser = self.laser_topic.getLaserData()

	# 	if(self.laser):
	# 		#angle = int(round(math.degrees(self.laser.maxAngle)))
	# 		angle = int(round(math.degrees(2*math.pi)))
	# 	for i in range(angle):
	# 		laser.append((0,0))
	# 	if(self.laser):
	# 		for i in range(angle):
	# 			dist = self.laser.values[i]
	# 			angle = math.radians(i)
	# 			if(dist == float("inf")):
	# 				dist = self.laser.maxRange
	# 			laser[i] = (dist, angle)
	# 	return laser, self.laser.maxRange

	def transformfun(self, posx, posy, orientation, distance):
		'''
			Transform the position of the measured point to the robot baseline.
			@posx: Position X of the sensor in the local baseline of the robot.
			@posy: Position Y of the sensor in the local baseline of the robot.
			@orientation: Orientation of the sensor.
			@distance: Distance reading by sensor.

			@return: Returns the position of the point in the robot baseline.
		'''
		cone = 0.261799
		pos_vertices = []

		xr = posx + math.cos(orientation)*distance
		yr = posy + math.sin(orientation)*distance

		# Calculating the vertices of the triangle
		hipotenusa = distance/(math.cos(cone/2))
		if orientation < 0:
			x_1 = posx + math.cos(orientation - cone/2)*hipotenusa
			y_1 = posy + math.sin(orientation - cone/2)*hipotenusa
			x_2 = posx + math.cos(orientation + cone/2)*hipotenusa
			y_2 = posy + math.sin(orientation + cone/2)*hipotenusa
		else:
			x_1 = posx + math.cos(orientation + cone/2)*hipotenusa
			y_1 = posy + math.sin(orientation + cone/2)*hipotenusa
			x_2 = posx + math.cos(orientation - cone/2)*hipotenusa
			y_2 = posy + math.sin(orientation - cone/2)*hipotenusa		
		pos_vertices = [x_1, y_1, x_2, y_2]

		return xr, yr, pos_vertices

	def global2canvas(self):
		pose = self.pose3d.getPose3d()
		yaw = pose.yaw
		x = pose.x
		y = pose.y
		points_global, point_global_sensor, pos_vertices, laser_points_global, laser_global = self.relative2global(x, y, yaw)
		pos_global_point = []
		pos_global_sensor = []
		pos_global_vertices = []
		pos_global_laser = []
		pos_global_laser_points = []

		scale_y = 33.25; offset_y = 0.33
		scale_x = 33.25; offset_x = 0.33
		for i in range(8):
			pos_global_point.append((0,0))
			key = 'sonar_' + str(i)
			p_x = points_global[key][0]
			p_y = points_global[key][1]
			x = scale_x * p_x + offset_x
			y = 729 - (scale_y * p_y + offset_y)
			pos_global_point[i] = (x, y)

		for i in range(8):
			pos_global_sensor.append((0,0))
			key = 'sonar_' + str(i)
			p_x = point_global_sensor[key][0]
			p_y = point_global_sensor[key][1]
			x = scale_x * p_x + offset_x
			y = 729 - (scale_y * p_y + offset_y)
			pos_global_sensor[i] = (x, y)

		# Coordinates of the laser points in the canvas
		for i in range(0,400):
			pos_global_laser_points.append((0,0))
			p_x = laser_points_global[i][0]
			p_y = laser_points_global[i][1]
			x = scale_x * p_x + offset_x
			y = 729 - (scale_y * p_y + offset_y)
			pos_global_laser_points[i] = (x, y)
		
		p_x = laser_global[0]
		p_y = laser_global[1]
		x = scale_x*p_x + offset_x
		y = 729 - (scale_y*p_y + offset_y)
		pos_global_laser = (x, y)

		# Coordinates of the vertices in the canvas
		for i in range(8):
			pos_global_vertices.append([0,0,0,0])
			key = 'sonar_' + str(i)
			p_x_1 = pos_vertices[key][0]
			p_y_1 = pos_vertices[key][1]
			p_x_2 = pos_vertices[key][2]
			p_y_2 = pos_vertices[key][3]
			p_x_1 = scale_x * p_x_1 + offset_x
			p_y_1 = 729 - (scale_y * p_y_1 + offset_y)
			p_x_2 = scale_x * p_x_2 + offset_x
			p_y_2 = 729 - (scale_y * p_y_2 + offset_y)
			pos_global_vertices[i] = (p_x_1, p_y_1, p_x_2, p_y_2)

		return pos_global_point, pos_global_sensor, pos_global_vertices, pos_global_laser_points, pos_global_laser

	def transformfun(self, posx, posy, orientation, distance):
		'''
			Transform the position of the measured point to the robot baseline.

			@posx: Position X of the sensor in the local baseline of the robot.
			@posy: Position Y of the sensor in the local baseline of the robot.
			@orientation: Orientation of the sensor.
			@distance: Distance reading by sensor.

			@return: Returns the position of the point in the robot baseline and the vertices of the sonar triangle.
		'''
		cone = 0.261799
		pos_vertices = []

		xr = posx + math.cos(orientation)*distance
		yr = posy + math.sin(orientation)*distance

		# Calculating the vertices of the triangle
		hipotenusa = distance/(math.cos(cone/2))
		if orientation < 0:
			x_1 = posx + math.cos(orientation - cone/2)*hipotenusa
			y_1 = posy + math.sin(orientation - cone/2)*hipotenusa
			x_2 = posx + math.cos(orientation + cone/2)*hipotenusa
			y_2 = posy + math.sin(orientation + cone/2)*hipotenusa
		else:
			x_1 = posx + math.cos(orientation + cone/2)*hipotenusa
			y_1 = posy + math.sin(orientation + cone/2)*hipotenusa
			x_2 = posx + math.cos(orientation - cone/2)*hipotenusa
			y_2 = posy + math.sin(orientation - cone/2)*hipotenusa		
		pos_vertices = [x_1, y_1, x_2, y_2]

		return xr, yr, pos_vertices

	def robotBaseline(self):
		'''
			Coordinates on the robot baseline.
		'''	
		laser_points = []
		laser = self.hal.laser.getLaserData()
		xr_0, yr_0, pos_vertices_0 = self.transformfun(self.sonar_0['pos_x'], self.sonar_0['pos_y'], self.sonar_0['orientation'], self.hal.sonar_0.getSonarData().distances)
		xr_1, yr_1, pos_vertices_1 = self.transformfun(self.sonar_1['pos_x'], self.sonar_1['pos_y'], self.sonar_1['orientation'], self.hal.sonar_1.getSonarData().distances)
		xr_2, yr_2, pos_vertices_2 = self.transformfun(self.sonar_2['pos_x'], self.sonar_2['pos_y'], self.sonar_2['orientation'], self.hal.sonar_2.getSonarData().distances)
		xr_3, yr_3, pos_vertices_3 = self.transformfun(self.sonar_3['pos_x'], self.sonar_3['pos_y'], self.sonar_3['orientation'], self.hal.sonar_3.getSonarData().distances)
		xr_4, yr_4, pos_vertices_4 = self.transformfun(self.sonar_4['pos_x'], self.sonar_4['pos_y'], self.sonar_4['orientation'], self.hal.sonar_4.getSonarData().distances)
		xr_5, yr_5, pos_vertices_5 = self.transformfun(self.sonar_5['pos_x'], self.sonar_5['pos_y'], self.sonar_5['orientation'], self.hal.sonar_5.getSonarData().distances)
		xr_6, yr_6, pos_vertices_6 = self.transformfun(self.sonar_6['pos_x'], self.sonar_6['pos_y'], self.sonar_6['orientation'], self.hal.sonar_6.getSonarData().distances)
		xr_7, yr_7, pos_vertices_7 = self.transformfun(self.sonar_7['pos_x'], self.sonar_7['pos_y'], self.sonar_7['orientation'], self.hal.sonar_7.getSonarData().distances)
		
		for i in range(0,400):
			laser_points.append((0,0))
			if(i == 0):
				angle = -math.pi
			else:
				angle = angle + 0.015708
			dist = laser.values[i]
			if dist == float("inf"):
			 	dist = self.hal.laser.getLaserData().maxRange
			xr = self.laser['pos_x'] + math.cos(angle)*dist
			yr = self.laser['pos_y'] + math.sin(angle)*dist
			laser_points[i] = (xr, yr)

		points_pos = {'sonar_0': [xr_0, yr_0], 'sonar_1': [xr_1, yr_1], 'sonar_2': [xr_2, yr_2], 'sonar_3': [xr_3, yr_3], 'sonar_4': [xr_4, yr_4], 'sonar_5': [xr_5, yr_5], 'sonar_6': [xr_6, yr_6], 'sonar_7': [xr_7, yr_7]}
		pos_vertices = {'sonar_0': pos_vertices_0, 'sonar_1': pos_vertices_1, 'sonar_2': pos_vertices_2, 'sonar_3': pos_vertices_3, 'sonar_4': pos_vertices_4, 'sonar_5': pos_vertices_5, 'sonar_6': pos_vertices_6, 'sonar_7': pos_vertices_7}
		
		return points_pos, pos_vertices, laser_points

	def relative2global(self, posx, posy, yaw):

		points_pos, pos_vertices, laser_points = self.robotBaseline()
		pos_vertices_global = []
		laser_points_global = []
		# Global position of the points measured by the sonar
		xg_0 = posx + math.cos(yaw)*points_pos['sonar_0'][0] - math.sin(yaw)*points_pos['sonar_0'][1]
		yg_0 = posy + math.sin(yaw)*points_pos['sonar_0'][0] + math.cos(yaw)*points_pos['sonar_0'][1]
		xg_1 = posx + math.cos(yaw)*points_pos['sonar_1'][0] - math.sin(yaw)*points_pos['sonar_1'][1]
		yg_1 = posy + math.sin(yaw)*points_pos['sonar_1'][0] + math.cos(yaw)*points_pos['sonar_1'][1]
		xg_2 = posx + math.cos(yaw)*points_pos['sonar_2'][0] - math.sin(yaw)*points_pos['sonar_2'][1]
		yg_2 = posy + math.sin(yaw)*points_pos['sonar_2'][0] + math.cos(yaw)*points_pos['sonar_2'][1]
		xg_3 = posx + math.cos(yaw)*points_pos['sonar_3'][0] - math.sin(yaw)*points_pos['sonar_3'][1]
		yg_3 = posy + math.sin(yaw)*points_pos['sonar_3'][0] + math.cos(yaw)*points_pos['sonar_3'][1]
		xg_4 = posx + math.cos(yaw)*points_pos['sonar_4'][0] - math.sin(yaw)*points_pos['sonar_4'][1]
		yg_4 = posy + math.sin(yaw)*points_pos['sonar_4'][0] + math.cos(yaw)*points_pos['sonar_4'][1]
		xg_5 = posx + math.cos(yaw)*points_pos['sonar_5'][0] - math.sin(yaw)*points_pos['sonar_5'][1]
		yg_5 = posy + math.sin(yaw)*points_pos['sonar_5'][0] + math.cos(yaw)*points_pos['sonar_5'][1]
		xg_6 = posx + math.cos(yaw)*points_pos['sonar_6'][0] - math.sin(yaw)*points_pos['sonar_6'][1]
		yg_6 = posy + math.sin(yaw)*points_pos['sonar_6'][0] + math.cos(yaw)*points_pos['sonar_6'][1]
		xg_7 = posx + math.cos(yaw)*points_pos['sonar_7'][0] - math.sin(yaw)*points_pos['sonar_7'][1]
		yg_7 = posy + math.sin(yaw)*points_pos['sonar_7'][0] + math.cos(yaw)*points_pos['sonar_7'][1]

		points_global = {'sonar_0': [xg_0, yg_0], 'sonar_1': [xg_1, yg_1], 'sonar_2': [xg_2, yg_2], 'sonar_3': [xg_3, yg_3], 'sonar_4': [xg_4, yg_4], 'sonar_5': [xg_5, yg_5], 'sonar_6': [xg_6, yg_6], 'sonar_7': [xg_7, yg_7]}
		# Calculating the position of the sensor in the global baseline.
		xsg_0 = posx + math.cos(yaw)*self.sonar_0['pos_x'] - math.sin(yaw)*self.sonar_0['pos_y']
		ysg_0 = posy + math.sin(yaw)*self.sonar_0['pos_x'] + math.cos(yaw)*self.sonar_0['pos_y']
		xsg_1 = posx + math.cos(yaw)*self.sonar_1['pos_x'] - math.sin(yaw)*self.sonar_1['pos_y']
		ysg_1 = posy + math.sin(yaw)*self.sonar_1['pos_x'] + math.cos(yaw)*self.sonar_1['pos_y']
		xsg_2 = posx + math.cos(yaw)*self.sonar_2['pos_x'] - math.sin(yaw)*self.sonar_2['pos_y']
		ysg_2 = posy + math.sin(yaw)*self.sonar_2['pos_x'] + math.cos(yaw)*self.sonar_2['pos_y']
		xsg_3 = posx + math.cos(yaw)*self.sonar_3['pos_x'] - math.sin(yaw)*self.sonar_3['pos_y']
		ysg_3 = posy + math.sin(yaw)*self.sonar_3['pos_x'] + math.cos(yaw)*self.sonar_3['pos_y']
		xsg_4 = posx + math.cos(yaw)*self.sonar_4['pos_x'] - math.sin(yaw)*self.sonar_4['pos_y']
		ysg_4 = posy + math.sin(yaw)*self.sonar_4['pos_x'] + math.cos(yaw)*self.sonar_4['pos_y']
		xsg_5 = posx + math.cos(yaw)*self.sonar_5['pos_x'] - math.sin(yaw)*self.sonar_5['pos_y']
		ysg_5 = posy + math.sin(yaw)*self.sonar_5['pos_x'] + math.cos(yaw)*self.sonar_5['pos_y']
		xsg_6 = posx + math.cos(yaw)*self.sonar_6['pos_x'] - math.sin(yaw)*self.sonar_6['pos_y']
		ysg_6 = posy + math.sin(yaw)*self.sonar_6['pos_x'] + math.cos(yaw)*self.sonar_6['pos_y']
		xsg_7 = posx + math.cos(yaw)*self.sonar_7['pos_x'] - math.sin(yaw)*self.sonar_7['pos_y']
		ysg_7 = posy + math.sin(yaw)*self.sonar_7['pos_x'] + math.cos(yaw)*self.sonar_7['pos_y']

		points_global_sensor = {'sonar_0': [xsg_0, ysg_0], 'sonar_1': [xsg_1, ysg_1], 'sonar_2': [xsg_2, ysg_2], 'sonar_3': [xsg_3, ysg_3], 'sonar_4': [xsg_4, ysg_4], 'sonar_5': [xsg_5, ysg_5], 'sonar_6': [xsg_6, ysg_6], 'sonar_7': [xsg_7, ysg_7]}
		
		for i in range(8):
			pos_vertices_global.append((0,0,0,0))
			key = 'sonar_' + str(i)
			xv_1 = posx + math.cos(yaw)*pos_vertices[key][0] - math.sin(yaw)*pos_vertices[key][1]
			yv_1 = posy + math.sin(yaw)*pos_vertices[key][0] + math.cos(yaw)*pos_vertices[key][1]
			xv_2 = posx + math.cos(yaw)*pos_vertices[key][2] - math.sin(yaw)*pos_vertices[key][3]
			yv_2 = posy + math.sin(yaw)*pos_vertices[key][2] + math.cos(yaw)*pos_vertices[key][3]
			pos_vertices_global[i] = (xv_1, yv_1, xv_2, yv_2)
	
		global_vertices = {'sonar_0': pos_vertices_global[0], 'sonar_1': pos_vertices_global[1], 'sonar_2': pos_vertices_global[2], 'sonar_3': pos_vertices_global[3], 'sonar_4': pos_vertices_global[4], 'sonar_5': pos_vertices_global[5], 'sonar_6': pos_vertices_global[6], 'sonar_7': pos_vertices_global[7]}
		
		# Global position of the measured points by laser
		for i in range(0,400):
			laser_points_global.append((0,0))
			xg = posx + math.cos(yaw)*laser_points[i][0] - math.sin(yaw)*laser_points[i][1]
			yg = posy + math.sin(yaw)*laser_points[i][0] + math.cos(yaw)*laser_points[i][1]
			laser_points_global[i] = (xg, yg)
		
		# Global position of the laser
		xg_laser = posx + math.cos(yaw)*self.laser['pos_x'] - math.sin(yaw)*self.laser['pos_y']
		yg_laser = posy + math.sin(yaw)*self.laser['pos_x'] + math.cos(yaw)*self.laser['pos_y']
		laser_global = (xg_laser, yg_laser)

		return points_global, points_global_sensor, global_vertices, laser_points_global, laser_global

	

