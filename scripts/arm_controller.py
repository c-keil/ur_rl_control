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
    current_joint_positions = np.zeros(6)
    current_joint_velocities = np.zeros(6)

    def __init__(self, test_control_signal = False ):
        '''set up controller class variables & parameters'''

        #launch nodes
        rospy.init_node('teleop_controller', anonymous=True)
        #start subscribers
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        if test_control_signal:
            print('Running in test mode')
            self.test_control_signal = test_control_signal
        else:
            rospy.Subscriber("daqdata_filtered", jointdata, self.daq_callback)
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

    def joint_state_callback(self, data):
        self.current_joint_positions[self.joint_reorder] = data.position
        self.current_joint_velocities[self.joint_reorder] = data.velocity

    def daq_callback(self, data):
        self.absolute_encoder_input = data.encoder1.pos

    def is_joint_position(self, position)->bool:
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
        return True if np.all(self.lower_lims < position) and np.all(self.upper_lims > position) else False

    def move_to(self, position, speed = 0.25, error_thresh = 0.01, override_initial_joint_lims = False)->bool:
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
            if not self.in_joint_lims(start_pos):
                print("Start Position Outside Joint Lims...")
                return
        if not self.in_joint_lims(position):
            print("Commanded Postion Outside Joint Lims...")
            return


        print('Executing Move to : \n{}\nIn {} seconds'.format(position,end_time))
        #list of interpolators ... this is kind of dumb, there is probably a better solution
        traj = [InterpolatedUnivariateSpline([0.,end_time],[start_pos[i],position[i]],k=1) for i in range(6)]

        position_error = np.array([1.0]*6) #set high position error
        pos_ref = deepcopy(start_pos)
        rate = rospy.Rate(500) #lim loop to 500 hz
        start_time = time.time()
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
        return

    def move():
        '''TODO Implement main control loop for teleoperation use. '''
        raise NotImplementedError()


if __name__ == "__main__":
    #This script is included for testing purposes
    print("starting")

    arm = ur5e_arm(test_control_signal=True)
    time.sleep(1)
    print(arm.current_joint_positions)
    arm.stop_arm()

    # target_pos = arm.default_pos
    # target_pos[5]+=1.0
    # arm.move_to(target_pos, speed = 0.1, override_initial_joint_lims=True)
