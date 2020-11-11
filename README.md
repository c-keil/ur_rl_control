# Readme

# TODO

-Add visuals for motion prediction, joint lommit indication, keepout zones, etc.

-Tune joint gains

-Add max acceleration limit

-Add safety features - dead man switch - keep out zones

-resolve encoder absolute position rollover issue: add a function that can set the current default_encoder_position setpoints to be 2pi wraped closest values. If the daq is turned on, then moved a significant amount (especially likely with the base joint), the user might expect close to zero difference from the reference position, but in reality there is ~2pi difference. ---- this can be fixed by automatically wrapping the daq input, centered on the saved configuration

-Feature, detect when position error is too large and stop

-Add ability to slowly track to user current position

-Test with encoder publishing from a separate computer

-Test encoder publishing rate - default at 100hz

-Separate Node to keep track of ref pos? To do this in parallel with keyboard shortcuts?

-control arm too big?

-Forward kinematics, safety position limits/keep out zones

-implement pausing and restarting functions - slow catch-up? - update reference position?

-velocity reference safety limit - check DAQ input  

<!-- -Define handling of joint lims inside vel control loop -->

# Usage
CAUTION: Check the initial joint configuration and make sure it is near the default joint config. The arm is sometimes in a very different position after other people have used it, and may cause damage to the gripper when starting motion for the first time.

Launch the base ur_robot_driver with the correct vel controller and other def
parameters with
`roslaunch test_vel_controller ur5e_driver.launch`
