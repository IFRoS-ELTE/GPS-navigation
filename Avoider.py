#!/usr/bin/env python3
import math
class Avoider():
	directions = []

	def __init__(self):
		self.directions = [-1,-1,-1,-1]

	def parse(self, scan):
        #initialize the minimum distance for each direction
		angle = scan.angle_min
		forward = -1
		right = -1
		back = -1
		left = -1

        #check points based on radians, by starting from min_angle, going through all points and incrementing by angle_increment
		for point in scan.ranges:
			if(((angle > -math.pi and angle < -math.pi*3/4) or (angle > math.pi *(3/4) and angle < math.pi)) and (back == -1 or point < back or back == 'inf')):
				back = point
			if((angle > -math.pi*3/4 and angle < -math.pi/4) and (right == -1 or point < right or right == 'inf')):
				right = point
			if((angle > -math.pi/4 and angle < math.pi/4) and (forward == -1 or point < forward or forward == 'inf')):
				forward = point
			if((angle > math.pi/4 and angle < math.pi*3/4) and (left == -1 or point < left or left == 'inf')):
				left = point
			angle += scan.angle_increment
            
        #return the minimum distance for each direction
		self.directions = [forward, right, back, left]
		print(self.directions)
