# GPS-navigation
GPS waypoint navigation, using LIDAR object avoidance

Obstacles are detected by the LIDAR in a point cloud, these are defined with an angle and distance
The angle starts and ends behind the robot, at -pi and pi, and each point is a constant increment further than the last
This way, we can iterate through them, and know in which quadrant it falls (a picture is included among the files)

In avoid.py, the main class, to read the data, we create a node, subscribe to the 'scan' topic to receive sensor_msgs/LaserScan messages from it, and bind it to the method of the Avoider.py class. This parses the data and returns the shortest distance detected in each quadrant. If this is short enough, by default 1 meter, it is detected as an obstacle, and the robot is steered accordingly, in avoid.py, by publishing a geometry_msgs/Twist message to the 'cmd_vel' topic
