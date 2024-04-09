#!/usr/bin/env python3
from Avoider import Avoider
import rospy
import math
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans



def main():

    vel = Twist()
    # Instanciate our avoider object
    avoider = Avoider()
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, avoider.parse)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
        #check forward direction
        if(avoider.directions[0] == 'inf' or avoider.directions[0] > 1):
            vel.linear.x = 0.2
            vel.angular.z = 0
            print('going forward')
        #check right direction
        elif (avoider.directions[1] == 'inf' or avoider.directions[1] > 1):
            vel.linear.x = 0
            vel.angular.z = -0.2
            print('turning right')
        #check left direction
        elif (avoider.directions[3] == 'inf' or avoider.directions[3] > 1):
            vel.linear.x = 0
            vel.angular.z = 0.2
            print('turning left')
        #check backward direction
        elif (avoider.directions[2] == 'inf' or avoider.directions[2] > 1):
            vel.linear.x = -0.2
            vel.angular.z = 0
            print('going back')
        pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
