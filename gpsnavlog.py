#!/usr/bin/env python


import rospy
import math
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

lat = 48
lon = 20
targetlat = 60
targetlon = 30
targetstopdistance = 10
distance = 0
heading = 0
fix = False

northangle = 0
devlimit = 5

directions = [-1,-1,-1,-1]
obstaclestopdistance = 1

def magcallback(imumsg):
    xcurrent = imumsg.vector.x
    ycurrent = imumsg.vector.y
    xnorth = 0.0
    ynorth = -0.5
    dot = xcurrent*xnorth+ycurrent*ynorth
    det = xnorth*ycurrent-ynorth*xcurrent
    angle = math.atan2(det,dot)
    angle = angle*180/math.pi
    global northangle
    northangle = angle
#    rospy.loginfo('North  %s', northangle)

def lasercallback(lasermsg):
    scan = lasermsg
    angle = scan.angle_min
    forward = -1
    right = -1
    back = -1
    left = -1

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
    directions = [forward, right, back, left]

def gnsscallback(gnssmsg):
    rospy.loginfo('Gnss  %s', gnssmsg.status.status)
    latdiff = targetlat - lat
    londiff = targetlon - lon
    latrad = lat*math.pi/180
    lonrad = lon*math.pi/180
    targetlatrad = targetlat*math.pi/180
    targetlonrad = targetlon*math.pi/180
    latdiffrad = latdiff*math.pi/180
    londiffrad = londiff*math.pi/180
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

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/imu/mag', Vector3Stamped, magcallback)
    rospy.Subscriber('/scan', LaserScan, lasercallback)
    rospy.Subscriber('/gnss', NavSatFix, gnsscallback)
                                                    
    
    vel = Twist()
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
        latdiff = targetlat - lat
        londiff = targetlon - lon
        latrad = lat*math.pi/180
        lonrad = lon*math.pi/180
        targetlatrad = targetlat*math.pi/180
        targetlonrad = targetlon*math.pi/180
        latdiffrad = latdiff*math.pi/180
        londiffrad = londiff*math.pi/180
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
    
        rospy.loginfo('AngleFromNorth: %s HeadingtoTarget: %s Distance: %s Lat: %s Lon: %s TargetLat: %s TargetLon: %s', northangle, heading, distance, lat, lon, targetlat, targetlon)
        if(northangle > heading-devlimit and northangle < heading+devlimit):
            print('facing target')
            if(directions[0] == 'inf' or directions[0] > obstaclestopdistance):
                vel.linear.x = 0.2
                vel.angular.z = 0
                print('going forward')
            elif (directions[1] == 'inf' or directions[1] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = -0.2
                print('turning left')
            elif (directions[3] == 'inf' or directions[3] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = 0.2
                print('turning right')
            elif (directions[2] == 'inf' or directions[2] > obstaclestopdistance):
                vel.linear.x = -0.2
                vel.angular.z = 0
                print('going back')
        elif(northangle > heading):
            print('right of target')
            if (directions[1] == 'inf' or directions[1] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = -0.2
                print('turning left')
            elif(directions[0] == 'inf' or directions[0] > obstaclestopdistance):
                vel.linear.x = 0.2
                vel.angular.z = 0
                print('going forward')
            elif (directions[3] == 'inf' or directions[3] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = 0.2
                print('turning right')
            elif (directions[2] == 'inf' or directions[2] > obstaclestopdistance):
                vel.linear.x = -0.2
                vel.angular.z = 0
                print('going back')
        elif(northangle < heading):
            print('left of target')
            if (directions[3] == 'inf' or directions[3] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = 0.2
                print('turning right')
            elif(directions[0] == 'inf' or directions[0] > obstaclestopdistance):
                vel.linear.x = 0.2
                vel.angular.z = 0
                print('g oing forward')
            elif (directions[1] == 'inf' or directions[1] > obstaclestopdistance):
                vel.linear.x = 0
                vel.angular.z = -0.2
                print('turning left')
            elif (directions[2] == 'inf' or directions[2] > obstaclestopdistance):
                vel.linear.x = -0.2
                vel.angular.z = 0
                print('going back')
                
        if(distance < targetstopdistance):
            print('Arrived at destination')
        vel.linear.x = 0
        vel.angular.z = 0
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    listener()
