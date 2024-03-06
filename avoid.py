#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans

directions = []

def radiantoangle(rad):
    return rad*(180/pi)

def parse(self, scan):
    angle = scan.angle_min
    forward = -1
    right = -1
    back = -1
    left = -1
    
    for point in scan.ranges:
        if(radiantoangle(angle)> 0 and forward != -1)
            forward = point
        if(radiantoangle(angle)> 90 and forward != -1)
            right = point
        if(radiantoangle(angle)> 180 and forward != -1)
            back = point
        if(radiantoangle(angle)> 270 and forward != -1)
            left = point
        angle +=scan.angle_increment
        directions = [forward, right, back, left]

def main():

    vel = Twist()
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, parse)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
        if(directions[0] == 'inf'):
            vel.linear.x = 0.5
            vel.angular.z = 0
        else if (directions[1] == 'inf'):
            vel.linear.x = 0
            vel.angular.z = 0.5
        else if (directions[3] == 'inf'):
            vel.linear.x = 0
            vel.angular.z = -0.5
        else if (directions[2] == 'inf'):
            vel.linear.x = -0.5
            vel.angular.z = 0
        pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
