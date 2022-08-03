#! /usr/bin/env python

import rospy
import rospkg
import tf
from copy import deepcopy
import time
from scipy.interpolate import InterpolatedUnivariateSpline
import numpy as np

from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.msg import SafetyMode
from ur_dashboard_msgs.srv import IsProgramRunning, GetSafetyMode
from std_msgs.msg import Bool

# Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

r = rospkg.RosPack()
path = r.get_path('ur_rl_control')
robot = URDF.from_xml_file(path+"/config/ur5e.urdf")

#parameters
joint_vel_lim = 1.0
sample_rate = 500
#joint inversion - accounts for encoder axes being inverted inconsistently
joint_inversion = np.array([1,1,-1,1,1,1]) # digital encoder dummy arm
two_pi = np.pi*2

class Ur5e_Cartesian_Controller():
    '''Stacks on tip of the compliant jointspace controller to generate cartesian trajectories'''

    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # forwawrd kinematics
    chain = tree.getChain("base_link", "wrist_3_link")
    # print chain.getNrOfJoints()
    kdl_kin = KDLKinematics(robot, "base_link", "wrist_3_link")

    tree_op = kdl_tree_from_urdf_model(dummy_arm)
    chain_op = tree_op.getChain("base_link", "wrist_3_link")
    kdl_kin_op = KDLKinematics(dummy_arm, "base_link", "wrist_3_link")

    current_joint_positions = np.zeros(6)
    current_joint_velocities = np.zeros(6)
    safety_mode = 1
    shutdown = False
    joint_state_message = JointState()

    def __init__(self):

        print("starting cartesian controller")

        #launch nodes
        rospy.init_node('cartesian controller', anonymous=True)
        rospy.on_shutdown(self.shutdown_safe)

        #start subscribers
        #start robot state subscriber (detects fault or estop press)
        # rospy.Subscriber('/ur_hardware_interface/safety_mode',SafetyMode, self.safety_callback)
        #joint feedback subscriber
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)

        #service to check if robot program is running
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running')
        #TODO wait for the joint controller to be up and running
        self.remote_control_running = rospy.ServiceProxy('ur_hardware_interface/dashboard/program_running', IsProgramRunning)
        #service to check safety mode
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode')
        self.safety_mode_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        self.ref_pub = rospy.Publisher("/joint_ref", JointState, queue_size=1)

        print("initialized")

    # def safety_callback(self, data):
    def joint_state_callback(self, data):
        self.current_joint_positions[self.joint_reorder] = data.position
        self.current_joint_velocities[self.joint_reorder] = data.velocity

    def shutdown_safe(self):
        '''Should ensure that the arm is brought to a stop before exiting'''
        self.shutdown = True
        print('Stopping -> Shutting Down')
        self.stop_arm()
        print('Stopped')

    def fill_out_joint_state(self, pos, vel):


    def stop_arm(self, safe = False):
        '''Commands zero velocity until sure the arm is stopped. If safe is False
        commands immediate stop, if set to a positive value, will stop gradually'''

        # if safe:
        #     loop_rate = rospy.Rate(200)
        #     start_time = time.time()
        #     start_vel = deepcopy(self.current_joint_velocities)
        #     max_accel = np.abs(start_vel/self.breaking_stop_time)
        #     vel_mask = np.ones(6)
        #     vel_mask[start_vel < 0.0] = -1
        #     while np.any(np.abs(self.current_joint_velocities)>0.0001) and not rospy.is_shutdown():
        #         command_vels = [0.0]*6
        #         loop_time = time.time() - start_time
        #         for joint in range(len(command_vels)):
        #             vel = start_vel[joint] - vel_mask[joint]*max_accel[joint]*loop_time
        #             if vel * vel_mask[joint] < 0:
        #                 vel = 0self.current_joint_velocities)>0.0001):
        #     self.vel_pub.publish(Flo
        #             command_vels[joint] = vel
        #         self.vel_pub.publish(Float64MultiArray(data = command_vels))
        #         if np.sum(command_vels) == 0:
        #             break
        #         loop_rate.sleep()

        # while np.any(np.abs(self.current_joint_velocities)>0.0001):
        #     self.vel_pub.publish(Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        command = JointState()
        command.header = Header()

    def move_to_joints(self, joints):
        '''moves the arm linearly through joint space to reach the "joints"'''
        print("joint debug")
        #TODO check joint lims

        speed = 0.25
        error_thresh = 0.01

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

        # #check joint llims
        # if not override_initial_joint_lims:
        #     if not self.identify_joint_lim(start_pos):
        #         print("Start Position Outside Joint Lims...")
        #         return False
        # if not self.identify_joint_lim(position):
        #     print("Commanded Postion Outside Joint Lims...")
        #     return False

        print('Executing Move to : \n{}\nIn {} seconds'.format(position,end_time))

        position_error = np.array([1.0]*6) #set high position error
        pos_ref = deepcopy(start_pos)
        rate = rospy.Rate(500) #lim loop to 500 hz
        start_time = time.time()
        reached_pos = False
        while not self.shutdown and not rospy.is_shutdown() and self.safety_mode == 1:
            if require_enable and not self.enabled:
                print('Lost Enable, stopping')
                break

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
            vel_ref_temp = self.joint_p_gains_varaible*position_error
            #enforce max velocity setting
            np.clip(vel_ref_temp,-joint_vel_lim,joint_vel_lim,vel_ref_temp)
            self.vel_ref.data = vel_ref_temp
            self.vel_pub.publish(self.vel_ref)
            # print(pos_ref)
            #wait
            rate.sleep()

        #make sure arm stops
        self.stop_arm(safe = True)
        return reached_pos


    def move_to_cartesian(self, pose):
        '''moves the arm linearly through cartesian space'''
        print("cartesian debug")
