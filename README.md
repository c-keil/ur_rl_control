# Readme

# TODO
-Implement main control loop (mostly copy from test script)
-Add safety features - dead man switch - max - accel

# Usage
CAUTION: Check the initial joint configuration and make sure it is near the default joint config. The arm is sometimes in a very different position after other people have used it, and may cause damage to the gripper when starting motion for the first time.
 
Launch the base ur_robot_driver with the correct vel controller and other def
parameters with
`roslaunch test_vel_controller ur5e_driver.launch`
