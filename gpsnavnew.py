#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

#declare latitude and longitude with placeholder value

lat = 48
lon = 20

#list targets, as tuples of latitude and longitude, set the first one as target

targets = [
(47.4739296,19.0579776), # gate
(47.4740669,19.0579415), # entrance
(47.4740227,19.0577421) # garden center
]
targetid = 0
targetlat = targets[targetid][0]
targetlon = targets[targetid][1]

#whether or not to enable LIDAR obstacle avoidance for debug purposes and set its variables for stopping distance

lidarenabled = True
directions = [-1,-1,-1,-1]
obstaclestopdistance = 1
obstaclestopdistancefar = 5

#declare variables for GPS navigation, and set deviation limit

distance = 0
heading = 0
fix = False
northangle = 0
devlimit = 15

#set turn and move speed, and stoping distance from target

turnspeed = 0.2
forwardspeed = 0.5
targetstopdistance = 3

#handles magnetic messages to determine heading
def magcallback(imumsg):
    #take the current magnetic vector, and the one calibrated when the robot is facing north
    xcurrent = imumsg.vector.x
    ycurrent = imumsg.vector.y
    xnorth = 0.0
    ynorth = -0.5
    #calculate the angle between them
    dot = xcurrent*xnorth+ycurrent*ynorth
    det = xnorth*ycurrent-ynorth*xcurrent
    angle = math.atan2(det,dot)
    angle = angle*180/math.pi
    global northangle
    northangle = angle

#handles lidar messags for obstacle detection
def lasercallback(lasermsg):
    #get message, set defaults for direction distances and angle
    scan = lasermsg
    angle = scan.angle_min
    forward = -1
    right = -1
    back = -1
    left = -1

    #go through all the points, each is angle_increment radians away from the previous, starting from angle_min
    #get the minimal distance to the points in each of the four quarters of the circle for the four directions
    #starts and ends at the back, going counter clockwise
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
    global directions
    directions = [forward, left, back, right]
    if(not lidarenabled):
        directions=['inf','inf','inf','inf']

#handle gps messages for navigation
def gnsscallback(gnssmsg):
    #check for gps connection, if found, get current latitude and longitude
    if gnssmsg.status.status :
	    rospy.loginfo('No Fix...')
    else:
        global fix
        fix = True
        global lat
        lat = gnssmsg.latitude
        global lon
        lon = gnssmsg.longitude
    #calculate difference from target latitude and longitude, convert them to radians
    latdiff = targetlat - lat
    londiff = targetlon - lon
    latrad = lat*math.pi/180
    lonrad = lon*math.pi/180
    targetlatrad = targetlat*math.pi/180
    targetlonrad = targetlon*math.pi/180
    latdiffrad = latdiff*math.pi/180
    londiffrad = londiff*math.pi/180
    #calculate distance and heading using method linked in readme
    earthradius = 6371000
    a = math.sin(latdiffrad/2) * math.sin(latdiffrad/2) + math.cos(latrad/2) * math.cos(targetlatrad/2) * math.sin(londiffrad/2)*math.sin(londiffrad/2)
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    global distance
    distance = earthradius*c
    y = math.sin(targetlonrad - lonrad)*math.cos(targetlatrad)
    x = math.cos(latrad)*math.sin(targetlatrad)-math.sin(latrad)*math.cos(targetlatrad)*math.cos(londiffrad)
    headrad = math.atan2(y,x)
    global heading
    heading = headrad*180/math.pi

#main method
def listener():
    #initialize node, subscribe to messages
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/imu/mag', Vector3Stamped, magcallback)
    rospy.Subscriber('/scan', LaserScan, lasercallback)
    rospy.Subscriber('/gnss', NavSatFix, gnsscallback)                                                
    
    #initialize control message for publishing
    vel = Twist()
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    
    #get target
    global targetlat
    targetlat = targets[targetid][0]
    global targetlon
    targetlon = targets[targetid][1]
    
    #variable for avoidance state
    avoiddir = 0
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
        #check for obstacles, near and far
        cangoforward = (directions[0] == 'inf' or directions[0] > obstaclestopdistance)
        cangoright = (directions[1] == 'inf' or directions[3] > obstaclestopdistance)
        cangoleft = (directions[3] == 'inf' or directions[1] > obstaclestopdistance)
        cangoback = (directions[2] == 'inf' or directions[2] > obstaclestopdistance)
        cangoforwardfar = (directions[0] == 'inf' or directions[0] > obstaclestopdistancefar)
        cangorightfar = (directions[1] == 'inf' or directions[3] > obstaclestopdistancefar)
        cangoleftfar = (directions[3] == 'inf' or directions[1] > obstaclestopdistancefar)
        cangobackfar = (directions[2] == 'inf' or directions[2] > obstaclestopdistancefar)
        
        #log info
        rospy.loginfo('AngleFromNorth: %s HeadingtoTarget: %s Distance: %s Lat: %s Lon: %s TargetLat: %s TargetLon: %s Front: %s Right: %s Left: %s Back: %s', northangle, heading, distance, lat, lon, targetlat, targetlon, cangoforward, cangoright, cangoleft, cangoback)
        
        #navigation
        if(fix):
            #set movement and turning to 0
            vel.linear.x = 0
            vel.angular.z = 0
            
            #move forward if able and close to facing the target
            if(northangle > heading-devlimit and northangle < heading+devlimit and cangoforward):
                vel.linear.x = forwardspeed
            
            #turn towards target
            turn = 0
            if(northangle>heading):
                #right of target
                turn = 1
            if(northangle<heading):
                #left of target
                turn = -1
            if(northangle > 90 and heading < -90):
                turn = -1
            if(northangle < -90 and heading > 90):
                turn = 1
                
            #avoid obstacle while moving
            if(not cangoforwardfar and avoiddir == 0):
                avoiddir = turn
	    if(not cangoforwardfar and avoiddir != 0):
                turn = avoiddir
                
            #check to see if past obstacle
            if(cangoforwardfar):
                if(avoiddir == -1 and cangorightfar): 
                    avoidir = 0
                if(avoiddir == 1 and cangoleftfar): 
                    avoidir = 0
                    
            #turning for avoidance overwrites turning to target
            if(avoiddir != 0):
                turn = avoiddir
                
            #turning away from close obstacles is more important
            if(not cangoleft):
                turn = -1
            if(not cangoright):
                turn = 1
                
            #move forward or back while avoiding close obstacles to avoid getting stuck
            if(not cangoleft or not cangoright and cangoback):
                vel.linear.x = -forwardspeed
            if(not cangoleft or not cangoright and cangoforward):
                vel.linear.x = forwardspeed
                
            #set turning in message
            vel.angular.z = turn*turnspeed
            
            #log movement
            if(turn == -1):
                print('turning right')
            if(turn == 1):
                print('turning left')
            if(vel.linear.x > 0):
                print('going forward')
            if(vel.linear.x < 0):
                print('going back')
        
        #stop and set new target if close
        if(fix and distance < targetstopdistance):
            vel.linear.x = 0
            vel.angular.z = 0
            print('Arrived at destination')
            global targetid
            targetid += 1
            if(targetid == len(targets)):
                targetid=0
            targetlat = targets[targetid][0]
            targetlon = targets[targetid][1]
        
        #publish movement
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    listener()
