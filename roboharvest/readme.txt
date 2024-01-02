RoboHarvest Package

Steps to run the package
1. Copy the package into your workspace/src
2. Colcon build
3. source install/setup.bash
4. ros2 launch roboharvest debug.launch.py (This opens both the Gazebo and RVIZ window.)
5. using the following scripts you can run the robot.

scripts folder has all the required codes to run the robot.

rc_tarzan.py - Code to use the mobile robot using keyboard, teleop script for the mobile robot.
rc_ur10.py - Code to use the ur10 arm with keyboard, teleop script for the ur10.
car_a2b.py - Open Loop Controller script where the robot moves in x direction.
a2b.py - Code where the arm goes to the position of the fruit from the home position.
a2b2c.py - Code where the arm goes to the position of the fruit and come back to the basket position.
inv_kin.py - Code to draw the circle (Inverse Kinematics Validation).
diff_diff.py - Teleop Script for diff drive of the mobile robot.

Contact me if you have any queries.
