#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
from copy import deepcopy
import time
from scipy.interpolate import InterpolatedUnivariateSpline

from filter import PythonBPF

from ur_kinematics.ur_kin_py import forward, forward_link
from kinematics import analytical_ik, nearest_ik_solution

import tf
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from ur5teleop.msg import jointdata, Joint
from ur_dashboard_msgs.msg import SafetyMode
from ur_dashboard_msgs.srv import IsProgramRunning, GetSafetyMode
from std_msgs.msg import Bool

from ur_rl_control.srv import CommandJoints, CommandJointsResponse, CommandPose, CommandPoseResponse

# Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

r = rospkg.RosPack()
path = r.get_path('ur_teleop_controller')
robot = URDF.from_xml_file(path+"/config/ur5e.urdf")
dummy_arm = URDF.from_xml_file(path+"/config/dummy_arm.urdf")

# from controller_manager_msgs.srv import ListControllers
# from controller_manager_msgs.srv import SwitchController

joint_vel_lim = 1.0
sample_rate = 500
control_arm_saved_zero = np.array([0.51031649, 1.22624958, 3.31996918, 0.93126088, 3.1199832, 9.78404331])

#define initial state

#joint inversion - accounts for encoder axes being inverted inconsistently
joint_inversion = np.array([1,1,-1,1,1,1]) # digital encoder dummy arm
two_pi = np.pi*2
# # TODO Arm class
    #Breaking at shutdown
    #Follow traj
#test_point = np.array([0.04,0.0,-0.21,1]).reshape(-1,1)
# test_point = np.array([0.0,0.2,0.0,1]).reshape(-1,1)
gripper_collision_points =  np.array([[0.04, 0.0, -0.21, 1.0], #fingertip
                                      [0.05, 0.04, 0.09,  1.0],  #hydraulic outputs
                                      [0.05, -0.04, 0.09,  1.0]]).T
def pose_matrix(position, orientation):
    '''returns a pose matrix from a position vector and quaternion'''
    pose = np.eye(4)
    pose = tf.transformations.quaternion_matrix(orientation)
    pose[:3,3] = position.reshape(-1)
    return pose

def get_quaternion_axis_angle(q):
    '''returns axis angle representation for a quaternion. Alays returns the minimum angle rotation direction'''
    q = np.array(q)/np.linalg.norm(q)

    angle = 2 * np.arccos(q[3])
    #check for angle = 0
    if angle > 0.0001:
        axis = q[:3] / np.sqrt(1-q[3]*q[3])
        #check for angle greater than pi
        # TODO check that this is actually what we want to do
        if angle > np.pi:
            angle = 2*np.pi - angle
            axis *= -1
    else:
        axis = np.array([1,0,0]) #arbitrary axis

    return axis, angle

class Base_Trajectory():
    def update_trajectroy():
        return True

class Joint_Trajectory(Base_Trajectory):
    def __init__(self, start_position, end_position, elapsed_time):
        print('initilaizing joint trajectroy')
        self.start_position = start_position
        self.end_position = end_position
        self.command_position = np.zeros(self.end_position.shape)
        # self.speed = speed
        self.elapsed_time = elapsed_time
        self.loop_start_time = None
        self.active = True
        #list of interpolators ... this is kind of dumb, there is probably a better solution
        self.traj = [InterpolatedUnivariateSpline([0.,self.elapsed_time],[self.start_position[i],self.end_position[i]],k=1) for i in range(6)]

    def update_trajectory(self):
        if self.loop_start_time is None:
            self.loop_start_time = time.time()
        
        loop_time = time.time()-self.loop_start_time
        if loop_time < self.elapsed_time:
            self.command_position[:] = [self.traj[i](loop_time) for i in range(6)]
        else:
            self.command_position = self.end_position 
            self.active = False

        return self.command_position


class Linear_Trajectory(Base_Trajectory):
    def __init__(self, joint_positions, elapsed_time):
        print('initilaizing psudo linear trajectroy')
        self.end_position = deepcopy(joint_positions[-1])
        self.interpolated_joint_positions = zip(*joint_positions)
        self.samples = len(self.interpolated_joint_positions[0])
        self.command_position = np.zeros((6))
        # self.speed = speed
        self.elapsed_time = elapsed_time
        self.loop_start_time = None
        self.active = True
        #list of interpolators ... this is kind of dumb, there is probably a better solution
        x = np.linspace(0,elapsed_time,self.samples)
        self.traj = [InterpolatedUnivariateSpline(x, j_pos,k=1) for j_pos in self.interpolated_joint_positions]

    def update_trajectory(self):
        if self.loop_start_time is None:
            self.loop_start_time = time.time()
        
        loop_time = time.time()-self.loop_start_time
        if loop_time < self.elapsed_time:
            self.command_position[:] = [self.traj[i](loop_time) for i in range(6)]
        else:
            self.command_position = self.end_position 
            self.active = False

        return self.command_position

## TODO Move some of the params in to a configuration file
class ur5e_arm():
    '''Defines velocity based controller for ur5e arm for use in teleop project
    '''
    safety_mode = -1
    shutdown = False
    enabled = True
    jogging = False
    current_trajectory = None #should hold a trajectory object when active
    joint_reorder = [2,1,0,3,4,5]
    breaking_stop_time = 0.1 #when stoping safely, executes the stop in 0.1s Do not make large!

    #throws an error and stops the arm if there is a position discontinuity in the
    #encoder input freater than the specified threshold
    #with the current settings of 100hz sampling, 0.1 radiands corresponds to
    #~10 rps velocity, which is unlikely to happen unless the encoder input is wrong
    position_jump_error = 0.1
    # ains = np.array([10.0]*6) #works up to at least 20 on wrist 3
    joint_p_gains_varaible = np.array([5.0, 5.0, 5.0, 10.0, 10.0, 10.0]) #works up to at least 20 on wrist 3
    joint_ff_gains_varaible = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # default_pos = (np.pi/180)*np.array([90.0, -90.0, 90.0, -90.0, -45.0, 180.0])
    # default_pos = (np.pi/180)*np.array([90.0, -90.0, 90.0, -90.0, 270.0, 90.0]) #top down grasp
    # print(default_pos)
    # time.sleep(10)
    default_pos = (np.pi/180)*np.array([90.0, -90.0, 90.0, 0.0, 90.0, 90.0])
    robot_ref_pos = deepcopy(default_pos)
    saved_ref_pos = None
    daq_ref_pos = deepcopy(default_pos)

    # lower_lims = (np.pi/180)*np.array([5.0, -120.0, 5.0, -90.0, 0.0, 0.0])
    # upper_lims = (np.pi/180)*np.array([175.0, 5.0, 175.0, 90.0, 180.0, 180.0])
    
    max_joint_speeds = 3.0 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    max_joint_acc = 5.0 * np.array([1.0, 1.0, 1.0, 3.0, 3.0, 3.0])
    max_linear_speed = 0.01
    homing_joint_speeds = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
    jogging_joint_speeds = 2.0 * homing_joint_speeds
    homing_joint_acc = 2.0 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    # max_joint_speeds = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])*0.1
    #default control arm setpoint - should be calibrated to be 1 to 1 with default_pos
    #the robot can use relative joint control, but this saved defailt state can
    #be used to return to a 1 to 1, absolute style control
    control_arm_def_config = np.mod(control_arm_saved_zero,np.pi*2)
    control_arm_ref_config = deepcopy(control_arm_def_config) #can be changed to allow relative motion

    #define fields that are updated by the subscriber callbacks
    current_joint_positions = np.zeros(6)
    current_joint_velocities = np.zeros(6)

    current_daq_positions = np.zeros(6)
    current_daq_velocities = np.zeros(6)
    #DEBUG
    current_daq_rel_positions = np.zeros(6) #current_daq_positions - control_arm_ref_config
    current_daq_rel_positions_waraped = np.zeros(6)

    first_daq_callback = True
    first_wrench_callback = True

    # define bandpass filter parameters
    fl = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    fh = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    fs = sample_rate
    filter = PythonBPF(fs, fl, fh)

    wrench = np.zeros(6)
    est_wrench_int_term = np.zeros(6)
    load_mass_publish_rate = 1
    load_mass_publish_index = 0

    ## control loop extra parameters
    current_wrench_global = np.zeros(6)
    current_joint_torque = np.zeros(6)

    joint_torque_error = np.zeros(6)
    wrench_global_error = np.zeros(6)

    joint_inertia = np.array([5.0, 5.0, 5.0, 1.0, 1.0, 1.0])
    joint_stiffness = 400 * np.array([0.6, 1.0, 1.0, 1.0, 1.0, 1.0])
    zeta = 1.0

    recent_data_focus_coeff = 0.99
    p = 1 / recent_data_focus_coeff

    last_command_joint_velocities = np.zeros(6)

    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # forwawrd kinematics
    chain = tree.getChain("base_link", "wrist_3_link")
    # print chain.getNrOfJoints()
    kdl_kin = KDLKinematics(robot, "base_link", "wrist_3_link")

    tree_op = kdl_tree_from_urdf_model(dummy_arm)
    chain_op = tree_op.getChain("base_link", "wrist_3_link")
    kdl_kin_op = KDLKinematics(dummy_arm, "base_link", "wrist_3_link")


    def __init__(self, test_control_signal = False):
        '''set up controller class variables & parameters'''

        #keepout (limmited to z axis height for now)
        self.keepout_enabled = True
        self.z_axis_lim = -0.37 # floor 0.095 #short table # #0.0 #table

        #launch nodes
        rospy.init_node('teleop_controller', anonymous=True)
        
        #rosparam check for joint lims and init position
        if not rospy.has_param("/start_position"):
            rospy.logerr("ur5e_arm() - __init__ - settings params not loaded")
        self.default_pos = (np.pi/180)*np.array(rospy.get_param("/start_position"))
        self.lower_lims = (np.pi/180)*np.array(rospy.get_param("/joint_lims/lower"))
        self.upper_lims = (np.pi/180)*np.array(rospy.get_param("/joint_lims/upper"))
        self.robot_ref_pos = deepcopy(self.default_pos)
        self.daq_ref_pos = deepcopy(self.default_pos)

        #start subscribers
        #start robot state subscriber (detects fault or estop press)
        rospy.Subscriber('/ur_hardware_interface/safety_mode',SafetyMode, self.safety_callback)
        #joint feedback subscriber
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        #wrench feedback
        rospy.Subscriber("wrench", WrenchStamped, self.wrench_callback)
        #service to check if robot program is running
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_running')
        self.remote_control_running = rospy.ServiceProxy('ur_hardware_interface/dashboard/program_running', IsProgramRunning)
        #service to check safety mode
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_safety_mode')
        self.safety_mode_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)

        #start vel publisher
        self.vel_pub = rospy.Publisher("/joint_group_vel_controller/command",
                            Float64MultiArray,
                            queue_size=1)

        #vertical direction force measurement publisher
        self.load_mass_pub = rospy.Publisher("/load_mass",
                            Float64MultiArray,
                            queue_size=1)
        self.test_data_pub = rospy.Publisher("/test_data",
                            Float64MultiArray,
                            queue_size=1)
        self.is_homing_pub = rospy.Publisher("/is_homing", Bool, queue_size=1)

        #ref pos publisher DEBUG
        self.daq_pos_pub = rospy.Publisher("/debug_ref_pos",
                            Float64MultiArray,
                            queue_size=1)
        self.daq_pos_wraped_pub = rospy.Publisher("/debug_ref_wraped_pos",
                            Float64MultiArray,
                            queue_size=1)
        self.ref_pos = Float64MultiArray(data=[0,0,0,0,0,0])

        #services
        self.joint_traj_service = rospy.Service('joint_trajectory_command', CommandJoints, self.handle_joint_traj)
        self.linear_joint_traj_service = rospy.Service('linear_trajectory_command', CommandPose, self.handle_linear_traj)

        #DEBUG
        # self.daq_pos_debug = Float64MultiArray(data=[0,0,0,0,0,0])
        # self.daq_pos_wraped_debug = Float64MultiArray(data=[0,0,0,0,0,0])

        #set shutdown safety behavior
        rospy.on_shutdown(self.shutdown_safe)
        time.sleep(0.5)
        self.stop_arm() #ensure arm is not moving if it was already

        self.velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.vel_ref = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.load_mass = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.test_data = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.is_homing = False

        rospy.loginfo("Joint Limmits: \n{} \n{}".format(self.upper_lims,self.lower_lims))

        if not self.ready_to_move():
            print('User action needed before commands can be sent to the robot.')
            self.user_prompt_ready_to_move()
        else:
            print('Ready to move')
    

    def get_cartesian_ref_posion(self):
        return np.array(self.cartesian_ref_pose[:3,3]).reshape(-1)

    def get_cartesian_ref_orientation(self):
        return self.cartesian_ref_pose[:3,:3]

    def get_cartesian_ref_quaternion(self):
        return tf.transformations.quaternion_from_matrix(self.cartesian_ref_pose)

    def joint_state_callback(self, data):
        self.current_joint_positions[self.joint_reorder] = data.position
        self.current_joint_velocities[self.joint_reorder] = data.velocity

    def handle_joint_traj(self, data):
        '''Recieves a joint trajectory and starts the trajectroy if appropriate'''
        rospy.loginfo("complaiant_controller-handle_joint_traj : trajectroy recieved")

        joints = data.joints
        traj_time = 5.0

        response = CommandJointsResponse(status = Bool(data = self.new_joint_trajectory(joints, traj_time)))
        return response

    def handle_linear_traj(self, data):
        '''recieves a liner joint trajectory and starts if appropriate'''
        pose = data.pose #assumes pose is relative to the current orientation
        rospy.loginfo("complaiant_controller-handle_linear_traj : trajectroy recieved")
        current_pose = np.array(self.kdl_kin.forward(self.current_joint_positions))
        current_position = current_pose[:3,3].reshape(-1)
        current_orientation = tf.transformations.quaternion_from_matrix(current_pose)

        delta_pos = np.array([pose.position.x,pose.position.y,pose.position.z])
        delta_rot = np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        rotation_axis, rotation_angle = get_quaternion_axis_angle(delta_rot)
        command_position = current_position + delta_pos
        command_orientation = tf.transformations.tf.transformations.quaternion_multiply(delta_rot, current_orientation)
        rospy.loginfo("complaiant_controller-handle_linear_traj : commanded position delta {}".format(delta_pos))
        rospy.loginfo("complaiant_controller-handle_linear_traj : commanded orientation delta axis: {}, angle: {}".format(rotation_axis, rotation_angle))
        #TODO insert max delta check?
        pose_matrix = pose_matrix(command_position, command_orientation)
        
        response = CommandPoseResponse()
        response.status = Bool(self.new_linear_trajectory(pose_matrix, speed = self.max_linear_speed))

        return response

    def wrench_callback(self, data):
        self.current_wrench = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
        
        if self.first_wrench_callback:
            self.filter.calculate_initial_values(self.current_wrench)
        
        Ja = self.kdl_kin.jacobian(self.current_joint_positions)
        FK = self.kdl_kin.forward(self.current_joint_positions)
        RT = FK[:3,:3]
        filtered_wrench = np.array(self.filter.filter(self.current_wrench))
        np.matmul(RT, filtered_wrench[:3], out = self.current_wrench_global[:3])
        np.matmul(RT, filtered_wrench[3:], out = self.current_wrench_global[3:])
        np.matmul(Ja.transpose(), self.current_wrench_global, out = self.current_joint_torque)
        # print(self)
        self.first_wrench_callback = False

    # def daq_callback(self, data):
    #     previous_positions = deepcopy(self.current_daq_positions)
    #     self.current_daq_positions[:] = [data.encoder1.pos, data.encoder2.pos, data.encoder3.pos, data.encoder4.pos, data.encoder5.pos, data.encoder6.pos]
    #     self.current_daq_velocities[:] = [data.encoder1.vel, data.encoder2.vel, data.encoder3.vel, data.encoder4.vel, data.encoder5.vel, data.encoder6.vel]
    #     self.current_daq_velocities *= joint_inversion #account for diferent conventions

    #     if not self.first_daq_callback and np.any(np.abs(self.current_daq_positions - previous_positions) > self.position_jump_error):
    #         print('stopping arm - encoder error!')
    #         print('Daq position change is too high')
    #         print('Previous Positions:\n{}'.format(previous_positions))
    #         print('New Positions:\n{}'.format(self.current_daq_positions))
    #         self.shutdown_safe()
    #     # np.subtract(,,out=) #update relative position
    #     self.current_daq_rel_positions = self.current_daq_positions - self.control_arm_ref_config
    #     self.current_daq_rel_positions *= joint_inversion
    #     self.current_daq_rel_positions_waraped = np.mod(self.current_daq_rel_positions+np.pi,two_pi)-np.pi
    #     self.first_daq_callback = False

    # def wrap_relative_angles(self):
    ## TODO Expand this to be a better exception handler 
    def safety_callback(self, data):
        '''Detect when safety stop is triggered'''
        self.safety_mode = data.mode
        if not data.mode == 1:
            #estop or protective stop triggered
            #send a breaking command
            print('\nFault Detected, sending stop command\n')
            self.stop_arm() #set commanded velocities to zero
            print('***Please clear the fault and restart the UR-Cap program before continuing***')

            #wait for user to fix the stop
            # self.user_wait_safety_stop()
    
    ## TODO Change function name, deadman_callback(self, data)
    def enable_callback(self, data):
        '''Detects the software enable/disable safety switch'''
        self.enabled = data.data

    ## DEPRECATED
    def jogging_callback(self, data):
        '''Detects the software jogging switch'''
        self.jogging = data.data

    ## TODO Merge this into exception handler
    def user_wait_safety_stop(self):
        #wait for user to fix the stop
        while not self.safety_mode == 1:
            raw_input('Safety Stop or other stop condition enabled.\n Correct the fault, then hit enter to continue')

    ## TODO Merge this into exception handler
    def ensure_safety_mode(self):
        '''Blocks until the safety mode is 1 (normal)'''
        while not self.safety_mode == 1:
            raw_input('Robot safety mode is not normal, \ncheck the estop and correct any faults, then restart the external control program and hit enter. ')

    def get_safety_mode(self):
        '''Calls get safet mode service, does not return self.safety_mode, which is updated by the safety mode topic, but should be the same.'''
        return self.safety_mode_proxy().safety_mode.mode

    ## TODO Safety handling
    def ready_to_move(self):
        '''returns true if the safety mode is 1 (normal) and the remote program is running'''
        return self.get_safety_mode() == 1 and self.remote_control_running()

    ## TODO Safety handling
    def user_prompt_ready_to_move(self):
        '''Blocking dialog to get the user to reset the safety warnings and start the remote program'''
        while True:
            if not self.get_safety_mode() == 1:
                print(self.get_safety_mode())
                raw_input('Safety mode is not Normal. Please correct the fault, then hit enter.')
            else:
                break
        while True:
            if not self.remote_control_running():
                raw_input('The remote control URCap program has been pause or was not started, please restart it, then hit enter.')
            else:
                break
        print('\nRemote control program is running, and safety mode is Normal\n')
    
    ## DEPRECATED
    def calibrate_control_arm_zero_position(self, interactive = True):
        '''Sets the control arm zero position to the current encoder joint states
        TODO: Write configuration to storage for future use'''
        if interactive:
            _ = raw_input("Hit enter when ready to save the control arm ref pos.")
        self.control_arm_def_config = np.mod(deepcopy(self.current_daq_positions),np.pi*2)
        self.control_arm_ref_config = deepcopy(self.control_arm_def_config)
        print("Control Arm Default Position Setpoint:\n{}\n".format(self.control_arm_def_config))

    def set_current_config_as_control_ref_config(self,
                                                 reset_robot_ref_config_to_current = True,
                                                 interactive = True):
        '''
        Initialize the encoder value of the dummy arm
        '''
        if interactive:
            _ = raw_input("Hit enter when ready to set the control arm ref pos.")
        self.control_arm_ref_config = np.mod(deepcopy(self.current_daq_positions),np.pi*2)
        if reset_robot_ref_config_to_current:
            self.robot_ref_pos = deepcopy(self.current_joint_positions)
        print("Control Arm Ref Position Setpoint:\n{}\n".format(self.control_arm_def_config))
    
    ## DEPRECATED
    def capture_control_arm_ref_position(self, interactive = True):
        '''Captures the current joint positions, and resolves encoder startup
        rollover issue. This adds increments of 2*pi to the control_arm_saved_zero
        to match the current joint positions to the actual saved position.'''
        max_acceptable_error = 0.6
        tries = 3
        for i in range(tries):
            if interactive:
                _ = raw_input("Hit enter when ready to capture the control arm ref pos. Try {}/{}".format(i+1,tries))
            #get current config
            control_arm_config = deepcopy(self.current_daq_positions)
            # print('Current DAQ Position:')
            # print(control_arm_config)
            #check if there is a significant error
            # config_variant1 = control_arm_config+2*np.pi
            # config_variant2 = control_arm_config-2*np.pi
            rot_offsets = [0, 2*np.pi, -2*np.pi]
            config_variants = [self.control_arm_def_config+off for off in rot_offsets]
            # error_set_1 = np.abs(self.control_arm_def_config - control_arm_config)
            # error_set_2 = np.abs(self.control_arm_def_config - config_variant1)
            # error_set_3 = np.abs(self.control_arm_def_config - config_variant2)
            error_sets = [np.abs(control_arm_config - var) for var in config_variants]
            print(error_sets)
            # print(error_set_2)
            # print(error_set_3)
            #if a 2*pi offset is a good match, reset the def_config to match
            error_too_great = [False]*6
            new_controll_config = deepcopy(self.control_arm_def_config)
            #TODO change behabior for base joint
            for joint in range(6):
                # configs = [control_arm_config, config_variant1, config_variant2]
                # offsets = [error_set_1[joint], error_set_2[joint], error_set_3[joint]]
                errors = [err[joint] for err in error_sets]
                min_error_idx = np.argmin(errors)
                if errors[min_error_idx]<max_acceptable_error:
                    new_controll_config[joint] = config_variants[min_error_idx][joint]
                    # new_controll_config[joint] = new_controll_config[joint]+rot_offsets[min_error_idx]
                else:
                    error_too_great[joint] = True
            if any(error_too_great):
                print('Excessive error. It may be necessary to recalibrate.')
                print('Make sure arm matches default config and try again.')
            else:
                print('Encoder Ref Capture successful.')
                print('New control arm config:\n{}'.format(new_controll_config))
                print('Updated from:')
                print(self.control_arm_def_config)
                time.sleep(1)
                self.control_arm_def_config = new_controll_config
                self.control_arm_ref_config = deepcopy(new_controll_config)
                break

    ## TODO Vanity check function
    def is_joint_position(self, position):
        '''Verifies that this is a 1dim numpy array with len 6'''
        if isinstance(position, np.ndarray):
            return position.ndim==1 and len(position)==6
        else:
            return False

    def shutdown_safe(self):
        '''Should ensure that the arm is brought to a stop before exiting'''
        self.shutdown = True
        print('Stopping -> Shutting Down')
        self.stop_arm()
        print('Stopped')

    def stop_arm(self, safe = False):
        '''Commands zero velocity until sure the arm is stopped. If safe is False
        commands immediate stop, if set to a positive value, will stop gradually'''

        if safe:
            loop_rate = rospy.Rate(200)
            start_time = time.time()
            start_vel = deepcopy(self.current_joint_velocities)
            max_accel = np.abs(start_vel/self.breaking_stop_time)
            vel_mask = np.ones(6)
            vel_mask[start_vel < 0.0] = -1
            while np.any(np.abs(self.current_joint_velocities)>0.0001) and not rospy.is_shutdown():
                command_vels = [0.0]*6
                loop_time = time.time() - start_time
                for joint in range(len(command_vels)):
                    vel = start_vel[joint] - vel_mask[joint]*max_accel[joint]*loop_time
                    if vel * vel_mask[joint] < 0:
                        vel = 0
                    command_vels[joint] = vel
                self.vel_pub.publish(Float64MultiArray(data = command_vels))
                if np.sum(command_vels) == 0:
                    break
                loop_rate.sleep()

        while np.any(np.abs(self.current_joint_velocities)>0.0001):
            self.vel_pub.publish(Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    ## TODO Vanity check function
    def in_joint_lims(self, position):
        '''expects an array of joint positions'''
        return np.all(self.lower_lims < position) and np.all(self.upper_lims > position)

    def identify_joint_lim(self, position):
        '''expects an array of joint positions. Prints a human readable list of
        joints that exceed limmits, if any'''
        if self.in_joint_lims(position):
            # print("All joints ok")
            return True
        else:
            for i, pos in enumerate(position):
                if pos<self.lower_lims[i]:
                    print('Joint {}: Position {:.5} exceeds lower bound {:.5}'.format(i,pos,self.lower_lims[i]))
                if pos>self.upper_lims[i]:
                    print('Joint {}: Position {:.5} exceeds upper bound {:.5}'.format(i,pos,self.lower_lims[i]))
            return False

    ## DEPRECATED
    def remote_program_running(self):
        print('remote : ',self.remote_control_running().program_running)
        return self.remote_control_running().program_running

    # move_to() function based on joint admittance controller
    def move_to(self):
        result = False
        rate = rospy.Rate(sample_rate)
        self.init_joint_admittance_controller()
        while not rospy.is_shutdown():
            current_homing_pos = self.current_daq_positions * joint_inversion + self.daq_ref_pos
            if not self.identify_joint_lim(current_homing_pos):
                print('Homing desired position outside robot position limit. Please change dummy position')
                result = False
                break
            #check safety
            if not self.ready_to_move():
                self.user_prompt_ready_to_move()
                continue
            #check enabled
            if not self.enabled:
                result = False
                break
            
            self.joint_admittance_controller(ref_pos = current_homing_pos,
                                             max_speeds = self.jogging_joint_speeds,
                                             max_acc = self.homing_joint_acc)
            
            if not np.any(np.abs(current_homing_pos - self.current_joint_positions)>0.1):
                print("Home position reached")
                result = True
                break
            #wait
            rate.sleep()
        self.stop_arm(safe = True)
        self.vel_ref.data = np.array([0.0]*6)
        return result

    # homing() function: initialzing UR to default position
    def homing(self):
    
        rate = rospy.Rate(sample_rate)
        self.init_joint_admittance_controller()
        while not rospy.is_shutdown():
            #check safety
            if not self.ready_to_move():
                self.user_prompt_ready_to_move()
                continue
            #check enabled
            if not self.enabled:
                self.stop_arm(safe = True)
                self.vel_ref.data = np.array([0.0]*6)
                time.sleep(0.01)
                continue
            print(self.default_pos)
            print(self.current_joint_positions)
            self.joint_admittance_controller(ref_pos = self.default_pos,
                                             max_speeds = self.homing_joint_speeds,
                                             max_acc = self.homing_joint_acc)

            if not np.any(np.abs(self.default_pos - self.current_joint_positions)>0.05):
                print("Home position reached")
                break

            #wait
            rate.sleep()
        
        self.stop_arm(safe = True)
        self.vel_ref.data = np.array([0.0]*6)

    ## TODO Refinement required
    def return_collison_free_config(self, reference_positon):
        '''takes the proposed set of joint positions for the real robot and
        checks the forward kinematics for collisions with the floor plane and the
        defined gripper points. Returns the neares position with the same orientation
        that is not violating the floor constraint.'''
        pose = forward(reference_positon)
        collision_positions = np.dot(pose, gripper_collision_points)

        min_point = np.argmin(collision_positions[2,:])
        collision = collision_positions[2,min_point] < self.z_axis_lim
        if collision:
            # print('Z axis overrun: {}'.format(pose[2,3]))
            #saturate pose
            diff = pose[2,3] - collision_positions[2][min_point]
            # print(diff)
            pose[2,3] = self.z_axis_lim + diff
            # pose[2,3] = self.z_axis_lim
            #get joint ref
            reference_positon = nearest_ik_solution(analytical_ik(pose,self.upper_lims,self.lower_lims),self.current_joint_positions,threshold=0.2)
        return reference_positon
    
    def inverse_kinematics(self, pose):
        '''IK call through kdl'''
        assert pose.shape == (4,4)
        assert type(pose) == np.ndarray

        return self.kdl_kin.inverse(pose, q_guess = self.current_joint_positions, min_joints = self.lower_lims, max_joints = self.upper_lims)

    def new_joint_trajectory(self, end_position, elapsed_time):
        '''adds a new trajectroy'''
        if self.current_trajectory is None:
            self.current_trajectory = Joint_Trajectory(self.current_joint_positions, end_position, elapsed_time)
            print("Trajectory Added")
            return True
        print("Failed to add trajectory")
        return False
    
    def new_linear_trajectory(self, end_pose, speed = 0.01):
        '''adds pseudo linear trajectroy'''
        assert end_pose.shape == (4,4)

        if speed > self.max_linear_speed:
            speed = self.max_linear_speed
        # TODO check end pose in workspace

        if self.current_trajectory is None:
            #get interpolated joint positions
            start_pose = np.array(self.kdl_kin.forward(self.current_joint_positions))
            num_samples = 10
            end_pose = end_pose
            start_position = start_pose[:3,3].flatten()
            end_position = end_pose[:3,3].flatten()
            magnitude = np.linalg.norm(end_position - start_position)
            elapsed_time = magnitude/speed
            rospy.loginfo("compliant_controller - new_linear_trajectory : trajectroy time calculated as {}".format(elapsed_time))
            start_orientation = tf.transformations.quaternion_from_matrix(start_pose)
            end_orientation = tf.transformations.quaternion_from_matrix(end_pose)
            interpolated_orientations = [tf.transformations.quaternion_slerp(start_orientation, end_orientation, i) for i in np.linspace(0,1,num_samples)]
            interpolated_positions = [sample*(end_position-start_position) + start_position for sample in np.linspace(0,1,num_samples)]
            # print("DEBUG")
            # print(start_position)
            # print(end_position)
            # print(interpolated_positions[0], interpolated_positions[num_samples-1])
            interpolated_poses = [pose_matrix(pos, rot) for pos, rot in zip(interpolated_positions, interpolated_orientations)]
            # interpolated_joint_positions = [self.inverse_kinematics(pose) for pose in interpolated_poses]
            interpolated_joint_positions = [self.inverse_kinematics(interpolated_poses[0])]
            for i, pose in enumerate(interpolated_poses[1:]):
                #use the previous joint positions as the seed for the new search
                interpolated_joint_positions.append(self.kdl_kin.inverse(pose, 
                                                                        q_guess = interpolated_joint_positions[i], 
                                                                        min_joints = self.lower_lims,
                                                                        max_joints = self.upper_lims))

            #safety check
            if any(pos is None for pos in interpolated_joint_positions):
                print("part of trajectroy has no Ik solution")
                for i in range(len(interpolated_poses)):
                    if interpolated_joint_positions[i] is None:
                        break
                print("Pose # {} : \n{}".format(i, interpolated_poses[i]))
                return False
            if np.any(interpolated_joint_positions[0] - self.current_joint_positions > 0.05):
                print("trajectroy start violation")
                return False
            #joint limit check TODO make more robust

            self.current_trajectory = Linear_Trajectory(interpolated_joint_positions, elapsed_time)
            print("Trajectory Added")
            return True
        print("Failed to add trajectory")
        return False

    def update_trajectory(self):
        '''Gets the current commanded position for the compliant trajectory'''
        command_position = self.robot_ref_pos
        if not self.current_trajectory is None:
            command_position = self.current_trajectory.update_trajectory()
            if not self.current_trajectory.active:
                print('Trajectory complete')
                self.current_trajectory = None
        return command_position

    ## TODO Break down this into separate modular functions
    def move(self,
             capture_start_as_ref_pos = False,
             dialoge_enabled = True):
        '''Main control loop for teleoperation use.'''
        if not self.ready_to_move():
            self.user_prompt_ready_to_move()

        rate = rospy.Rate(sample_rate)
        self.init_joint_admittance_controller(capture_start_as_ref_pos, dialoge_enabled)

        while not self.shutdown and self.safety_mode == 1 and not rospy.is_shutdown(): #shutdown is set on ctrl-c.
            self.robot_ref_pos = self.update_trajectory() #updates robot_ref_pos
            self.joint_admittance_controller(ref_pos = self.robot_ref_pos,
                                             max_speeds = self.max_joint_speeds,
                                             max_acc = self.max_joint_acc)
            # self.joint_admittance_controller(ref_pos = self.current_daq_rel_positions_waraped + self.robot_ref_pos,
            #                                  max_speeds = self.max_joint_speeds,
            #                                  max_acc = self.max_joint_acc)
            #wait
            rate.sleep()
        self.stop_arm(safe = True)
        self.vel_ref.data = np.array([0.0]*6)

    # online torque error id
    def force_torque_error_estimation(self, position_error, force_error, force):
        if not np.any(np.abs(position_error)>0.01) and not np.any(np.abs(self.current_joint_velocities)>0.001):
            force_error += self.p / (1 + self.p) * (force - force_error)
            self.p = (self.p - self.p ** 2 / (1 + self.p)) / self.recent_data_focus_coeff
        return force_error
        
    # joint admittance controller initialization
    def init_joint_admittance_controller(self,
                                         capture_start_as_ref_pos = False,
                                         dialoge_enabled = True):
        
        self.joint_torque_error = deepcopy(self.current_joint_torque)
        self.vel_admittance = np.zeros(6)

        if capture_start_as_ref_pos:
            self.set_current_config_as_control_ref_config(interactive = dialoge_enabled)
            self.current_daq_rel_positions_waraped = np.zeros(6)

    # joint admittance controller
    def joint_admittance_controller(self,
                                    ref_pos,
                                    max_speeds,
                                    max_acc):
        np.clip(ref_pos, self.lower_lims, self.upper_lims, ref_pos)
        joint_pos_error = np.subtract(ref_pos, self.current_joint_positions)
        vel_ref_array = np.multiply(joint_pos_error, self.joint_p_gains_varaible)

        self.joint_torque_error = self.force_torque_error_estimation(joint_pos_error, self.joint_torque_error, self.current_joint_torque)
        joint_torque_after_correction = self.current_joint_torque - self.joint_torque_error

        acc = (joint_torque_after_correction + self.joint_stiffness * joint_pos_error
                + 0.5 * 2 * self.zeta * np.sqrt(self.joint_stiffness * self.joint_inertia) * (self.current_daq_velocities - self.current_joint_velocities)
                - 0.5 * 2 * self.zeta * np.sqrt(self.joint_stiffness * self.joint_inertia) * self.current_joint_velocities) / self.joint_inertia
        np.clip(acc, -max_acc, max_acc, acc)
        self.vel_admittance += acc / sample_rate

        vel_ref_array[0] += self.vel_admittance[0]
        vel_ref_array[1] += self.vel_admittance[1]
        vel_ref_array[2] += self.vel_admittance[2]

        np.clip(vel_ref_array, self.last_command_joint_velocities - max_acc / sample_rate, self.last_command_joint_velocities + max_acc / sample_rate, vel_ref_array)
        np.clip(vel_ref_array, -max_speeds, max_speeds, vel_ref_array)
        self.last_command_joint_velocities = vel_ref_array

        #publish
        self.vel_ref.data = vel_ref_array
        # self.ref_vel_pub.publish(self.vel_ref)
        self.vel_pub.publish(self.vel_ref)        

    # Cartesian admittance controller initialization

    # Cartesian admittance controller

    ## TODO Main loop, add saftey handling here
    def run(self):
        '''Run runs the move routine repeatedly, accounting for the
        enable/disable switch'''

        print('Put the control arm in start configuration.')
        print('Depress and hold the deadman switch when ready to move.')

        while not rospy.is_shutdown():
            #check safety
            if not self.safety_mode == 1:
                time.sleep(0.01)
                continue
            #check enabled
            if not self.enabled:
                time.sleep(0.01)
                continue
            # current_homing_pos = deepcopy(self.current_daq_positions * joint_inversion + self.daq_ref_pos)
            # if not self.identify_joint_lim(current_homing_pos):
            #     print('Homing desired position outside robot position limit. Please change dummy position')
            #     continue
            
            self.is_homing = True
            self.is_homing_pub.publish(self.is_homing)
            # reached_home = self.move_to(current_homing_pos,speed = 0.2,override_initial_joint_lims=True,require_enable = True)
            reached_home = self.move_to()
            self.is_homing = False
            self.is_homing_pub.publish(self.is_homing)
            if reached_home:
                #start moving
                print('Starting Free Movement')
                self.move(capture_start_as_ref_pos = True,
                          dialoge_enabled = False)



if __name__ == "__main__":
    #This script is included for testing purposes
    print("starting")

    arm = ur5e_arm(test_control_signal=True)
    time.sleep(1)
    arm.stop_arm()

    # print(arm.move_to_robost(arm.default_pos,
    #                          speed = 0.1,
    #                          override_initial_joint_lims=True,
    #                          require_enable = True))
    arm.homing()
    # pose = np.array(forward(arm.current_joint_positions))
    pose = np.array(arm.kdl_kin.forward(arm.current_joint_positions))
    print("pose:")
    print(pose)
    pose[0,3] += 0.1
    # print(pose)
    # positions = nearest_ik_solution(analytical_ik(pose, arm.upper_lims, arm.lower_lims), arm.current_joint_positions)
    # print(positions)
    positions = arm.kdl_kin.inverse(pose, q_guess = arm.current_joint_positions, min_joints = arm.lower_lims, max_joints = arm.upper_lims)
    print(positions)
    print(dir(arm.kdl_kin))
    print(arm.kdl_kin.joint_limits_lower)
    print(arm.kdl_kin.joint_limits_upper)
    # print(pose)
    raw_input("Hit enter when ready to move")
    print(arm.new_linear_trajectory(pose, speed = 0.01))
    # print(arm.new_joint_trajectory(positions, 1.0))
    arm.run()

    arm.stop_arm()
