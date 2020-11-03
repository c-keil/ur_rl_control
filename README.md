# Readme

# TODO
-Implement main control loop (mostly copy from test script)

-Add safety features - dead man switch - max - accel

-resolve encoder absolute position rollover issue: add a function that can set the current default_encoder_position setpoints to be 2pi wraped closest values. If the daq is turned on, then moved a significant amount (especially likely with the base joint), the user might expect close to zero difference from the refference position, but in reality there is ~2pi difference.

-Feature, detect when position error is too large and stop

-Add ability to slowly track to user current position

-Test with encoder publishing from a separate computer

-Test encoder publishing rate - default at 100hz

# Usage
CAUTION: Check the initial joint configuration and make sure it is near the default joint config. The arm is sometimes in a very different position after other people have used it, and may cause damage to the gripper when starting motion for the first time.

Launch the base ur_robot_driver with the correct vel controller and other def
parameters with
`roslaunch test_vel_controller ur5e_driver.launch`
