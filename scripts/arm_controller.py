#! /usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
import time
from scipy.interpolate import InterpolatedUnivariateSpline

from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from ur5teleop.msg import jointdata, Joint
# from controller_manager_msgs.srv import ListControllers
# from controller_manager_msgs.srv import SwitchController

joint_vel_lim = 1.0
# control_arm_saved_zero = np.array([0.61465591, 1.22207916, 0.36568704, 1.04289675, 3.14202762, 5.99422216])
control_arm_saved_zero = np.array([ 0.62736291, 1.19030643, -0.03857971, 0.99792278, 3.12708712, 5.91298103])
#define initial state

# # TODO Arm class
    #Breaking at shutdown
    #Follow traj

class ur5e_arm():
    '''Defines velocity based controller for ur5e arm for use in teleop project
    '''
    shutdown = False
    joint_reorder = [2,1,0,3,4,5]
    joint_p_gains = np.array([10.0]*6) #works up to at least 20 on wrist 3

    default_pos = (np.pi/180)*np.array([90.0, -90.0, 90.0, -90.0, -90, 180.0])

    lower_lims = (np.pi/180)*np.array([0.0, -100.0, 0.0, -180.0, -180.0, 90.0])
    upper_lims = (np.pi/180)*np.array([180.0, 0.0, 150.0, 0.0, 0.0, 270.0])
    conservative_lower_lims = (np.pi/180)*np.array([45.0, -100.0, 45.0, -135.0, -135.0, 135.0])
    conservative_upper_lims = (np.pi/180)*np.array([135, -45.0, 140.0, -45.0, -45.0, 225.0])

    #default control arm setpoint - should be calibrated to be 1 to 1 with default_pos
    #the robot can use relative joint control, but this saved defailt state can
    #be used to return to a 1 to 1, absolute style control
    control_arm_def_config = control_arm_saved_zero
    control_arm_ref_config = deepcopy(control_arm_def_config) #can be changed to allow relative motion

    #define fields that are updated by the subscriber callbacks
    current_joint_positions = np.zeros(6)
    current_joint_velocities = np.zeros(6)

    current_daq_positions = np.zeros(6)
    current_daq_velocities = np.zeros(6)
    current_daq_rel_positions = np.zeros(6) #current_daq_positions - control_arm_ref_config

    def __init__(self, test_control_signal = False, conservative_joint_lims = True):
        '''set up controller class variables & parameters'''

        if conservative_joint_lims:
            self.lower_lims = self.conservative_lower_lims
            self.upper_lims = self.conservative_upper_lims

        #launch nodes
        rospy.init_node('teleop_controller', anonymous=True)
        #start subscribers
        if test_control_signal:
            print('Running in test mode ... no daq input')
            self.test_control_signal = test_control_signal
        else:
            rospy.Subscriber("daqdata_filtered", jointdata, self.daq_callback)

        #joint feedback subscriber
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)

        #start vel publisher
        self.vel_pub = rospy.Publisher("/joint_group_vel_controller/command",
                            Float64MultiArray,
                            queue_size=1)

        #set shutdown safety behavior
        rospy.on_shutdown(self.shutdown_safe)
        time.sleep(0.5)
        self.stop_arm() #ensure arm is not moving if it was already

        self.velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.vel_ref = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        print("Joint Limmits: ")
        print(self.upper_lims)
        print(self.lower_lims)

    def joint_state_callback(self, data):
        self.current_joint_positions[self.joint_reorder] = data.position
        self.current_joint_velocities[self.joint_reorder] = data.velocity

    def daq_callback(self, data):
        self.current_daq_positions[:] = [data.encoder1.pos, data.encoder2.pos, data.encoder3.pos, data.encoder4.pos, data.encoder5.pos, data.encoder6.pos]
        self.current_daq_velocities[:] = [data.encoder1.vel, data.encoder2.vel, data.encoder3.vel, data.encoder4.vel, data.encoder5.vel, data.encoder6.vel]
        np.subtract(self.current_daq_positions,self.control_arm_ref_config,out=self.current_daq_rel_positions) #update relative position

    def calibrate_control_arm_zero_position(self, interactive = True):
        '''Sets the control arm zero position to the current encoder joint states
        TODO: Write configuration to storage for future use'''
        if interactive:
            _ = raw_input("Hit enter when ready to save the control arm ref pos.")
        self.control_arm_def_config = deepcopy(self.current_daq_positions)
        print("Control Arm Default Position Setpoint:\n{}\n".format(self.control_arm_def_config))

    def is_joint_position(self, position):
        '''Verifies that this is a 1dim numpy array with len 6'''
        if isinstance(position, np.ndarray):
            return position.ndim==1 and len(position)==6
        else:
            return False

    def shutdown_safe(self):
        '''Should ensure that the arm is brought to a stop before exiting'''
        self.shutdown = True
        self.stop_arm()
        self.stop_arm()

    def stop_arm(self):
        '''commands zero velocity until sure the arm is stopped'''
        while np.any(np.abs(self.current_joint_velocities)>0.001):
            self.vel_pub.publish(Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    def in_joint_lims(self, position):
        '''expects an array of joint positions'''
        return np.all(self.lower_lims < position) and np.all(self.upper_lims > position)

    def identify_joint_lim(self, position):
        '''expects an array of joint positions. Prints a human readable list of
        joints that exceed limmits, if any'''
        if self.in_joint_lims(position):
            print("All joints ok")
            return True
        else:
            for i, pos in enumerate(position):
                if pos<self.lower_lims[i]:
                    print('Joint {}: Position {:.5} exceeds lower bound {:.5}'.format(i,pos,self.lower_lims[i]))
                if pos>self.upper_lims[i]:
                    print('Joint {}: Position {:.5} exceeds upper bound {:.5}'.format(i,pos,self.lower_lims[i]))
            return False

    def move_to(self, position, speed = 0.25, error_thresh = 0.01, override_initial_joint_lims = False):
        '''CAUTION - use joint lim override with extreme caution. Intended to
        allow movement from outside the lims back to acceptable position.

        Defines a simple joing controller to bring the arm to a desired
        configuration without teleop input. Intended for testing or to reach
        present initial positions, etc.'''

        #define max speed slow for safety
        if speed > 0.5:
            print("Limiting speed to 0.5 rad/sec")
            speed = 0.5

        #calculate traj from current position
        start_pos = deepcopy(self.current_joint_positions)
        max_disp = np.max(np.abs(position-start_pos))
        end_time = max_disp/speed

        #make sure this is a valid joint position
        if not self.is_joint_position(position):
            print("Invalid Joint Position, Exiting move_to function")
            return False

        #check joint llims
        if not override_initial_joint_lims:
            if not self.identify_joint_lim(start_pos):
                print("Start Position Outside Joint Lims...")
                return False
        if not self.identify_joint_lim(position):
            print("Commanded Postion Outside Joint Lims...")
            return False


        print('Executing Move to : \n{}\nIn {} seconds'.format(position,end_time))
        #list of interpolators ... this is kind of dumb, there is probably a better solution
        traj = [InterpolatedUnivariateSpline([0.,end_time],[start_pos[i],position[i]],k=1) for i in range(6)]

        position_error = np.array([1.0]*6) #set high position error
        pos_ref = deepcopy(start_pos)
        rate = rospy.Rate(500) #lim loop to 500 hz
        start_time = time.time()
        reached_pos = False
        while True and not self.shutdown: #chutdown is set on ctrl-c.
            loop_time = time.time()-start_time
            if loop_time < end_time:
                pos_ref[:] = [traj[i](loop_time) for i in range(6)]
            else:
                pos_ref = position
                # break
                if np.all(np.abs(position_error)<error_thresh):
                    print("reached target position")
                    self.stop_arm()
                    reached_pos = True
                    break

            position_error = pos_ref - self.current_joint_positions
            vel_ref_temp = self.joint_p_gains*position_error
            #enforce max velocity setting
            np.clip(vel_ref_temp,-joint_vel_lim,joint_vel_lim,vel_ref_temp)
            self.vel_ref.data = vel_ref_temp
            self.vel_pub.publish(self.vel_ref)
            # print(pos_ref)
            #wait
            rate.sleep()

        #make sure arm stops
        self.stop_arm()
        return reached_pos

    def move(self):
        '''TODO Implement main control loop for teleoperation use. '''
        # raise NotImplementedError()

        max_pos_error = 0.5 #radians/sec
        low_joint_vel_lim = 0.1

        position_error = np.zeros(6)
        absolute_position_error = np.zeros(6)
        position_error_exceeded_by = np.zeros(6)
        vel_ref_array = np.zeros(6)
        ref_pos = deepcopy(self.current_joint_positions)
        rate = rospy.Rate(500)
        while True and not self.shutdown: #chutdown is set on ctrl-c.
            #get ref position inplace - avoids repeatedly declaring new array
            np.add(self.default_pos,self.current_daq_rel_positions,out = ref_pos)

            #enforce joint lims
            np.clip(ref_pos, self.lower_lims, self.upper_lims, ref_pos)

            #inplace error calculation
            np.subtract(ref_pos,self.current_joint_positions,position_error)
            # np.abs(position_error,out=absolute_position_error)
            # #if position error is very large, we should slowly approach the target ...
            # #maybe we should use a filter to do this to move a small amount in the direction
            # np.subtract(absolute_position_error,max_pos_error,out=position_error_exceeded_by)
            # max_error = np.max(position_error_exceeded_by)
            # if max_error > 0: #scale error signal down
            #     position_error *= max_pos_error/max_error

            # print(position_error)
            #calculate vel signal
            np.multiply(position_error,self.joint_p_gains,out=vel_ref_array)
            #enforce max velocity setting
            # np.clip(vel_ref_array,-joint_vel_lim,joint_vel_lim,vel_ref_array)
            np.clip(vel_ref_array,-low_joint_vel_lim,low_joint_vel_lim,vel_ref_array)
            #publish
            self.vel_ref.data = vel_ref_array
            self.vel_pub.publish(self.vel_ref)
            #wait
            rate.sleep()
        self.stop_arm()


if __name__ == "__main__":
    #This script is included for testing purposes
    print("starting")

    arm = ur5e_arm(test_control_signal=False)
    time.sleep(1)
    arm.stop_arm()

    # arm.calibrate_control_arm_zero_position(interactive = True)
    # arm.move_to(arm.default_pos, speed = 0.1, override_initial_joint_lims=True)

    print("Current Arm Position")
    print(arm.current_joint_positions)
    print("DAQ position:")
    print(arm.current_daq_positions)
    # daq_pos = deepcopy(arm.current_daq_positions)
    # daq_offset = arm.current_daq_positions - arm.control_arm_def_config
    # print("DAQ offset from default pos: \n{}".format(daq_offset))
    # if 'y'==raw_input('Execute Move? (y/n)'):
    #     target_pos = arm.default_pos + daq_offset
    #     arm.move_to(target_pos, speed = 0.1, override_initial_joint_lims=False)
    raw_input("Hit enter when ready to move")
    # arm.move()

    arm.stop_arm()


    # target_pos = arm.default_pos
    # target_pos[5]+=1.0
    # arm.move_to(target_pos, speed = 0.1, override_initial_joint_lims=True)
