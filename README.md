# home_service_robot
simulation a home service robot with ROS

packages:

gmapping: this pkg is used to provide SLAM.

turtlebot_teleop: this package is used to navigate the robot in the gazebo world to map the environment.

turtlebot_rviz_launchers: load the Rviz workspace.

turtlebot_gazebo: this pkg create a gazebo environment with world file and turtlebot simulation robot.

pick_objects: this pkg drive robot from the base location to a goal location. With the help of Dijkstra's algorithm, this node provides a safe path by considering obstacle avoidance. Pick_objects node uses move_base service which allows the robot to find a path toward goals.

add_markers: In this pkg, the add_markers node creates a virtual object in the Rviz environment. this node subscribes to odometry to track the robot position plus publish the visualization_marke topic to communicate with nodes. Inside this node, a Pub-Sub class is defined to be used variables easily.

Note: In this project, I've used RTAB_Map pkg instead of gmapping to build a map as I did in pervius project.

