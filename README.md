# GPS-navigation
GPS waypoint navigation, using LIDAR object avoidance

To run the code, place the gpsnavnew.py python script into catkin_ws/src/Obstacle-Navigation-ROS/mysimplenav/Scripts and enable execution on them, then use 'rosrun Obstacle-Navigation-ROS gpsnavnew.py' after settin

# GPS
https://www.movable-type.co.uk/scripts/latlong.html
Based on the solutions at this site, the distance and heading to the target is calculated from the current latitude and longitude, and those of the target

# Magnet
The magnetometer gives the vector pointing in the direction of the magnetic field, towards north, compared to the current orientation of the robot.
Based on this, the heading compared to north can be calculated, as the angle between the current direction of the magnet vector and the one calibrated when the robot is facing north.

# LIDAR
The LIDAR returns points in a circle, starting and ending at a given angle, with a given angle increment between points, in radians.
Starting behind the robot at pi/2 radians, it goes around counter clockwise, saving the closest point in each quarter, the cutoffs being +/- 1/4 and 3/4 radians.

# Listener
Establishes the ROS node for listening and publishing messages, from the various measuring devices and to the wheels.
Controlling the robot, it tries to turn towards the target based on it's magnetic heading, compared to that towards the target.
When it is within a given range, it goes forward if nothing is in the way.
If an obstacle is far enough, it tries to maneuver around it while moving, by turning in the direction the target is, until the way forward and the direction the obstacle would have ended up on are free
If it can't move forward because the obstacle is close, it moves backward, and turns away from close obstacles on the sides overriding turning towards the target, while keeping that movement
