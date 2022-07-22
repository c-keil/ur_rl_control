#! /usr/bin/env python
from multiprocessing import current_process
import rospy
import rospkg
import tf
import numpy as np
from copy import deepcopy
import time
from scipy.interpolate import InterpolatedUnivariateSpline

from filter import PythonBPF

# from ur_kinematics.ur_kin_py import forward, forward_link
# from kinematics import analytical_ik, nearest_ik_solution

from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
# from ur5teleop.msg import jointdata, Joint
from ur_dashboard_msgs.msg import SafetyMode
from ur_dashboard_msgs.srv import IsProgramRunning, GetSafetyMode
from std_msgs.msg import Bool

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
class ur5e_arm():
    '''Defines velocity based controller for ur5e arm for use in teleop project
    '''
    safety_mode = -1
    shutdown = False
    enabled = True #not safe
    traj_active = False #used to determine when a trajectory has been sent to the robot in the control loop
    jogging = False
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

    default_pos = (np.pi/180)*np.array([90.0, -90.0, 90.0, 0.0, 90.0, 90.0])
    cartesian_ref_pose = np.eye(4) #TODO make initialization roboust
    robot_ref_pos = deepcopy(default_pos)
    saved_ref_pos = None
    daq_ref_pos = deepcopy(default_pos)

    lower_lims = (np.pi/180)*np.array([0, -120, 0, -90, 0, 0])
    upper_lims = (np.pi/180)*np.array([180, 5, 180, 90, 180, 210.0])
    # conservative_lower_lims = (np.pi/180)*np.array([45.0, -100.0, 45.0, -135.0, -135.0, 135.0])
    # conservative_upper_lims = (np.pi/180)*np.array([135, -45.0, 140.0, -45.0, -45.0, 225.0])
    # max_joint_speeds = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])
    max_joint_speeds = 3.0 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    max_joint_acc = 2.0 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    # max_joint_speeds = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])*0.1
    #default control arm setpoint - should be calibrated to be 1 to 1 with default_pos
    #the robot can use relative joint control, but this saved defailt state can
    #be used to return to a 1 to 1, absolute style control
    control_arm_def_config = np.mod(control_arm_saved_zero,np.pi*2)
    control_arm_ref_config = deepcopy(control_arm_def_config) #can be changed to allow relative motion

    #define fields that are updated by the subscriber callbacks
    current_joint_positions = np.zeros(6)
    current_joint_velocities = np.zeros(6)
    current_pose = np.eye(4)

    current_daq_positions = np.zeros(6)
    current_daq_velocities = np.zeros(6)
    #DEBUG
    current_daq_rel_positions = np.zeros(6) #current_daq_positions - control_arm_ref_config
    current_daq_rel_positions_waraped = np.zeros(6)

    first_daq_callback = True

    # define bandpass filter parameters
    fl = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    fh = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    fs = sample_rate
    filter = PythonBPF(fs, fl, fh)

    inertia_offset = np.array([5.0, 5.0, 5.0, 1.0, 1.0, 1.0])
    # inertia_offset = np.array([5.0, 5.0, 5.0, 0.5, 0.5, 0.5])

    wrench = np.zeros(6)
    est_wrench_int_term = np.zeros(6)
    load_mass_publish_rate = 1
    load_mass_publish_index = 0

    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # forwawrd kinematics
    chain = tree.getChain("base_link", "wrist_3_link")
    # print chain.getNrOfJoints()
    kdl_kin = KDLKinematics(robot, "base_link", "wrist_3_link")

    tree_op = kdl_tree_from_urdf_model(dummy_arm)
    chain_op = tree_op.getChain("base_link", "wrist_3_link")
    kdl_kin_op = KDLKinematics(dummy_arm, "base_link", "wrist_3_link")


    def __init__(self, test_control_signal = False, conservative_joint_lims = True):
        '''set up controller class variables & parameters'''

        if conservative_joint_lims:
            self.lower_lims = self.conservative_lower_lims
            self.upper_lims = self.conservative_upper_lims

        #keepout (limmited to z axis height for now)
        self.keepout_enabled = True
        self.z_axis_lim = -0.37 # floor 0.095 #short table # #0.0 #table

        #set cartesian ref pos to current pose (should be reset by the user as the start point for any experiment)
        self.set_ref_pose_to_current()

        #launch nodes
        rospy.init_node('teleop_controller', anonymous=True)
        #start subscribers
        if test_control_signal:
            print('Running in test mode ... no daq input')
            self.test_control_signal = test_control_signal
        else:
            rospy.Subscriber("daqdata_filtered", jointdata, self.daq_callback)

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
        #start subscriber for deadman enable
        # rospy.Subscriber('/enable_move',Bool,self.enable_callback)
        #start subscriber for jogging enable
        # rospy.Subscriber('/jogging',Bool,self.jogging_callback)

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


        print("Joint Limmits: ")
        print(self.upper_lims)
        print(self.lower_lims)

        if not self.ready_to_move():
            print('User action needed before commands can be sent to the robot.')
            self.user_prompt_ready_to_move()
        else:
            print('Ready to move')

    def set_ref_pose_to_current(self):
        self.cartesian_ref_pose = self.kdl_kin.forward(self.current_joint_positions)
        return True

    def get_cartesian_ref_posion(self):
        return np.array(self.cartesian_ref_pose[:3,3]).reshape(-1)
    
    def get_cartesian_ref_orientation(self):
        return self.cartesian_ref_pose[:3,:3]
    
    def get_cartesian_ref_quaternion(self):
        return tf.transformations.quaternion_from_matrix(self.cartesian_ref_pose)


    def joint_state_callback(self, data):
        self.current_joint_positions[self.joint_reorder] = data.position
        self.current_joint_velocities[self.joint_reorder] = data.velocity

    def wrench_callback(self, data):
        self.current_wrench = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

    def daq_callback(self, data):
        previous_positions = deepcopy(self.current_daq_positions)
        self.current_daq_positions[:] = [data.encoder1.pos, data.encoder2.pos, data.encoder3.pos, data.encoder4.pos, data.encoder5.pos, data.encoder6.pos]
        self.current_daq_velocities[:] = [data.encoder1.vel, data.encoder2.vel, data.encoder3.vel, data.encoder4.vel, data.encoder5.vel, data.encoder6.vel]
        self.current_daq_velocities *= joint_inversion #account for diferent conventions

        if not self.first_daq_callback and np.any(np.abs(self.current_daq_positions - previous_positions) > self.position_jump_error):
            print('stopping arm - encoder error!')
            print('Daq position change is too high')
            print('Previous Positions:\n{}'.format(previous_positions))
            print('New Positions:\n{}'.format(self.current_daq_positions))
            self.shutdown_safe()
        # np.subtract(,,out=) #update relative position
        self.current_daq_rel_positions = self.current_daq_positions - self.control_arm_ref_config
        self.current_daq_rel_positions *= joint_inversion
        self.current_daq_rel_positions_waraped = np.mod(self.current_daq_rel_positions+np.pi,two_pi)-np.pi
        self.first_daq_callback = False

    # def wrap_relative_angles(self):
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

    def enable_callback(self, data):
        '''Detects the software enable/disable safety switch'''
        self.enabled = data.data

    def jogging_callback(self, data):
        '''Detects the software jogging switch'''
        self.jogging = data.data

    def user_wait_safety_stop(self):
        #wait for user to fix the stop
        while not self.safety_mode == 1:
            raw_input('Safety Stop or other stop condition enabled.\n Correct the fault, then hit enter to continue')

    def ensure_safety_mode(self):
        '''Blocks until the safety mode is 1 (normal)'''
        while not self.safety_mode == 1:
            raw_input('Robot safety mode is not normal, \ncheck the estop and correct any faults, then restart the external control program and hit enter. ')

    def get_safety_mode(self):
        '''Calls get safet mode service, does not return self.safety_mode, which is updated by the safety mode topic, but should be the same.'''
        return self.safety_mode_proxy().safety_mode.mode

    def ready_to_move(self):
        '''returns true if the safety mode is 1 (normal) and the remote program is running'''
        return self.get_safety_mode() == 1 and self.remote_control_running()

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
        if interactive:
            _ = raw_input("Hit enter when ready to set the control arm ref pos.")
        self.control_arm_ref_config = np.mod(deepcopy(self.current_daq_positions),np.pi*2)
        if reset_robot_ref_config_to_current:
            self.robot_ref_pos = deepcopy(self.current_joint_positions)
        print("Control Arm Ref Position Setpoint:\n{}\n".format(self.control_arm_def_config))

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
        # self.stop_arm()

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

    def remote_program_running(self):
        print('remote : ',self.remote_control_running().program_running)
        return self.remote_control_running().program_running

    def move_to_robost(self,
                position,
                speed = 0.25,
                error_thresh = 0.01,
                override_initial_joint_lims = False,
                require_enable = False):
        '''Calls the move_to method as necessary to ensure that the goal position
        is reached, accounting for interruptions due to safety faults, and the
        enable deadman if require_enable is selected'''

        if require_enable:
            print('Depress and hold the deadman switch when ready to move.')
            print('Release to stop')

        while not rospy.is_shutdown():
            #check safety
            if not self.ready_to_move():
                self.user_prompt_ready_to_move()
                continue
            #check enabled
            if not self.enabled:
                time.sleep(0.01)
                continue
            #start moving
            print('Starting Trajectory')
            result = self.move_to(position,
                                  speed = speed,
                                  error_thresh = error_thresh,
                                  override_initial_joint_lims = override_initial_joint_lims,
                                  require_enable = require_enable)
            if result:
                break
        print('Reached Goal')


    def move_to(self,
                position,
                speed = 0.25,
                error_thresh = 0.01,
                override_initial_joint_lims = False,
                require_enable = False):
        '''CAUTION - use joint lim override with extreme caution. Intended to
        allow movement from outside the lims back to acceptable position.

        Defines a simple joing controller to bring the arm to a desired
        configuration without teleop input. Intended for testing or to reach
        present initial positions, etc.'''

        #ensure safety sqitch is not enabled
        if not self.ready_to_move():
            self.user_prompt_ready_to_move()

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
        while not self.shutdown and not rospy.is_shutdown() and self.safety_mode == 1: #chutdown is set on ctrl-c.
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

    # def return_collison_free_config(self, reference_positon):
    #     '''takes the proposed set of joint positions for the real robot and
    #     checks the forward kinematics for collisions with the floor plane and the
    #     defined gripper points. Returns the neares position with the same orientation
    #     that is not violating the floor constraint.'''
    #     pose = self.kdl_kin.forward(reference_positon)
    #     collision_positions = np.dot(pose, gripper_collision_points)
    #
    #     min_point = np.argmin(collision_positions[2,:])
    #     collision = collision_positions[2,min_point] < self.z_axis_lim
    #     if collision:
    #         # print('Z axis overrun: {}'.format(pose[2,3]))
    #         #saturate pose
    #         diff = pose[2,3] - collision_positions[2][min_point]
    #         # print(diff)
    #         pose[2,3] = self.z_axis_lim + diff
    #         # pose[2,3] = self.z_axis_lim
    #         #get joint ref
    #         reference_positon = nearest_ik_solution(analytical_ik(pose,self.upper_lims,self.lower_lims),self.current_joint_positions,threshold=0.2)
    #     return reference_positon

    def move(self,
             capture_start_as_ref_pos = False,
             dialoge_enabled = True):
        '''Main control loop for teleoperation use.'''
        if not self.ready_to_move():
            self.user_prompt_ready_to_move()

        max_pos_error = 0.5 #radians/sec
        low_joint_vel_lim = 0.5

        # impedance setting
        # zeta = 0.707
        zeta = 1.0
        # virtual_stiffness = 400 * np.array([0.6, 1.0, 1.0, 1.0, 1.0, 1.0])

        coupling_stiffness = 200.0 * np.array([1, 1, 1, 0.2, 0.2, 0.2])
        # inertia = 20.0 * np.array([1, 1, 1, 1, 1, 1])
        coupling_damping = coupling_stiffness * 1.4

        vel = np.zeros(6)
        pos = np.zeros(6)
        vel_ref = np.zeros(6)
        pos_ref = np.zeros(6)

        force = np.zeros(6)

        vd = np.zeros(6)
        ad = np.zeros(6)

        joint_torque = np.zeros(6)
        joint_vd = np.zeros(6)

        wrench = np.zeros(6)
        filtered_wrench = np.zeros(6)
        wrench_global = np.zeros(6)

        end_effector_inertia = 0.45 * np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        est_wrench_global = np.zeros(6)
        end_effector_vel = np.zeros(6)
        momentum_observer_gain = 60

        pin_damping = 0.05

        vel_ref_array = np.zeros(6)
        load_mass_array = np.zeros(6)
        self.vel_ref.data = vel_ref_array
        rate = rospy.Rate(1) #TODO put back to 500 

        self.filter.calculate_initial_values(self.current_wrench)

        # initialization
        joint_daq_pos = self.current_daq_positions * joint_inversion + self.daq_ref_pos
        Ja = self.kdl_kin.jacobian(self.current_joint_positions)
        FK = self.kdl_kin.forward(self.current_joint_positions)
        RT = FK[:3,:3]
        Ja_ref = self.kdl_kin_op.jacobian(joint_daq_pos)
        FK_ref = self.kdl_kin_op.forward(joint_daq_pos)
        RT_ref = FK_ref[:3,:3]
        wrench = self.current_wrench
        filtered_wrench = np.array(self.filter.filter(wrench))
        np.matmul(RT, filtered_wrench[:3], out = wrench_global[:3])
        np.matmul(RT, filtered_wrench[3:], out = wrench_global[3:])

        recent_data_focus_coeff = 0.99
        p = 1 / recent_data_focus_coeff
        wrench_global_error = wrench_global
        est_wrench_global = wrench_global
        self.est_wrench_int_term = wrench_global / momentum_observer_gain

        q = 1 / recent_data_focus_coeff
        wrench_global_error_display = wrench_global

        init_pos_ref = np.zeros(6)
        init_pos_ref[:3] = np.array([FK_ref[0,3],FK_ref[1,3],FK_ref[2,3]])
        init_pos_ref[4] = np.arcsin(-RT_ref[2,0]) # theta
        last_theta_ref = init_pos_ref[4]
        init_pos_ref[5] = np.arctan2(RT_ref[1,0], RT_ref[0,0]) # phi
        init_pos_ref[3] = np.arctan2(RT_ref[2,1], RT_ref[2,2]) # psi

        init_pos = np.zeros(6)
        init_pos[:3] = np.array([FK[0,3],FK[1,3],FK[2,3]])
        init_pos[4] = np.arcsin(-RT[2,0]) # theta
        last_theta = init_pos[4]
        init_pos[5] = np.arctan2(RT[1,0], RT[0,0]) # phi
        init_pos[3] = np.arctan2(RT[2,1], RT[2,2]) # psi

        # go back to joint control
        ref_pos = deepcopy(self.current_joint_positions)
        position_error = np.zeros(6)
        zeta = 1.0
        virtual_stiffness = 400 * np.array([0.6, 1.0, 1.0, 1.0, 1.0, 1.0])
        filtered_wrench = np.zeros(6)
        wrench_global = np.zeros(6)
        joint_desired_torque = np.zeros(6)
        acc = np.zeros(6)
        vr = np.zeros(6)


        self.filter.calculate_initial_values(self.current_wrench)

        Ja = self.kdl_kin.jacobian(self.current_joint_positions)
        FK = self.kdl_kin.forward(self.current_joint_positions)
        RT = FK[:3,:3]
        wrench = self.current_wrench
        filtered_wrench = np.array(self.filter.filter(wrench))
        np.matmul(RT, filtered_wrench[:3], out = wrench_global[:3])
        np.matmul(RT, filtered_wrench[3:], out = wrench_global[3:])
        np.matmul(Ja.transpose(), wrench_global, out = joint_desired_torque)
        recent_data_focus_coeff = 0.99
        p = 1 / recent_data_focus_coeff
        joint_torque_error = joint_desired_torque
        wrench_global_error = wrench_global

        if capture_start_as_ref_pos:
            self.set_current_config_as_control_ref_config(interactive = dialoge_enabled)
            self.current_daq_rel_positions_waraped = np.zeros(6)
        # print('safety_mode',self.safety_mode)
        while not self.shutdown and self.safety_mode == 1 and self.enabled: #chutdown is set on ctrl-c.
            # # daq cartesian position
            # # joint_daq_pos = self.current_daq_positions * joint_inversion + self.daq_ref_pos
            # np.add(self.robot_ref_pos,self.current_daq_rel_positions_waraped,out = ref_pos)
            # np.clip(ref_pos, self.lower_lims, self.upper_lims, ref_pos)
            # np.subtract(ref_pos, self.current_joint_positions, position_error)
            # np.multiply(position_error,self.joint_p_gains_varaible,out=vel_ref_array)

            # jacobian
            Ja = np.array(self.kdl_kin.jacobian(self.current_joint_positions))
            FK = np.array(self.kdl_kin.forward(self.current_joint_positions))
            RT = FK[:3,:3]

            #calculate cartesian error
            if self.traj_active:
                print('calculcating desired position')    
                #calculate desired position

            #calculate cartesian error
            self.current_pose = FK 
            cartesian_position_error = self.get_cartesian_ref_posion().reshape(-1) - self.current_pose[:3,3]
            current_orientation = tf.transformations.quaternion_from_matrix(FK)
            orientation_error = tf.transformations.quaternion_multiply(self.get_cartesian_ref_quaternion(), tf.transformations.quaternion_inverse(current_orientation))
            axis, angle = get_quaternion_axis_angle(orientation_error)
            angular_vel_error = axis*angle
            # TODO cap cartesian error
            error = np.array([cartesian_position_error[0],
                              cartesian_position_error[1], 
                              cartesian_position_error[2], 
                              angular_vel_error[0], 
                              angular_vel_error[1], 
                              angular_vel_error[2]]).reshape((6,1))

            #convert to joint velocities
            # TODO fix vel ref error 
            vel_ref_array = (np.linalg.inv(Ja)*error).reshape(-1)
            # print(type(Ja))
            # print(np.linalg.inv(Ja))
            # print(type(error))
            # print(cartesian_position_error)
            print("angular vel error")
            print(angular_vel_error)
            print("joint vels")
            print(vel_ref_array)
            print("")
            

            wrench = self.current_wrench
            filtered_wrench = np.array(self.filter.filter(wrench))
            np.matmul(RT, filtered_wrench[:3], out = wrench_global[:3])
            np.matmul(RT, filtered_wrench[3:], out = wrench_global[3:])
            np.matmul(Ja.transpose(), wrench_global, out = joint_desired_torque)

            # online joint torque error id
            if np.any(np.abs(position_error)>0.01) or np.any(np.abs(self.current_joint_velocities)>0.001):
                joint_desired_torque_after_correction = joint_desired_torque - joint_torque_error
                wg = wrench_global - wrench_global_error
                flag = 1
            else:
                joint_torque_error = joint_torque_error + p / (1 + p) * (joint_desired_torque - joint_torque_error)
                wrench_global_error = wrench_global_error + p / (1 + p) * (wrench_global - wrench_global_error)
                p = (p - p ** 2 / (1 + p)) / recent_data_focus_coeff
                joint_desired_torque_after_correction = joint_desired_torque - joint_torque_error
                wg = wrench_global - wrench_global_error
                flag = -1

            acc = (joint_desired_torque + virtual_stiffness * position_error
                    + 0.5 * 2 * zeta * np.sqrt(virtual_stiffness * self.inertia_offset) * (self.current_daq_velocities - self.current_joint_velocities)
                    - 0.5 * 2 * zeta * np.sqrt(virtual_stiffness * self.inertia_offset) * self.current_joint_velocities) / (self.inertia_offset)
            np.clip(acc, -self.max_joint_acc, self.max_joint_acc, acc)
            vr += acc / sample_rate

            vel_ref_array[2] += vr[2]
            vel_ref_array[0] += vr[0]
            vel_ref_array[1] += vr[1]

            '''*************************************************************************************'''

            # # # jacobian
            # # calculate cartesian position and orientation error 
            # # TODO switch from euler angle error to quaternion error calculation
            # Ja = self.kdl_kin.jacobian(self.current_joint_positions)
            # FK = self.kdl_kin.forward(self.current_joint_positions)
            # RT = FK[:3,:3]
            # Ja_ref = self.kdl_kin_op.jacobian(joint_daq_pos)
            # FK_ref = self.kdl_kin_op.forward(joint_daq_pos)
            # RT_ref = FK_ref[:3,:3]

            # pos_ref[:3] = np.array([FK_ref[0,3],FK_ref[1,3],FK_ref[2,3]])
            # #pos_ref[4] = np.arctan2(-RT_ref[2,0], np.sqrt(RT_ref[0,0]**2 + RT_ref[1,0]**2)) # theta
            # pos_ref[4] = np.arcsin(-RT_ref[2,0])
            # 500elif pos_ref[4] - last_theta_ref < -np.pi/2:
            #     pos_ref[4] = pos_ref[4] + np.pi
            # las_theta_ref = pos_ref[4]
            # cos_theta_ref_sign = np.sign(np.cos(pos_ref[4]))
            # pos_ref[5] = np.arctan2(RT_ref[1,0]*cos_theta_ref_sign, RT_ref[0,0]*cos_theta_ref_sign) # phi
            # pos_ref[3] = np.arctan2(RT_ref[2,1]*cos_theta_ref_sign, RT_ref[2,2]*cos_theta_ref_sign) # psi

            # pos[:3] = np.array([FK[0,3],FK[1,3],FK[2,3]])
            # #pos[4] = np.arctan2(-RT[2,0], np.sqrt(RT[0,0]**2 + RT[1,0]**2)) # theta
            # pos[4] = np.arcsin(-RT[2,0])
            # if pos[4] - last_theta > np.pi/2:
            #    pos[4] = pos[4] - np.pi
            # elif pos[4] - last_theta < -np.pi/2:
            #    pos[4] = pos[4] + np.pi
            # last_theta = pos[4]
            # cos_theta_sign = np.sign(np.cos(pos[4]))
            # pos[5] = np.arctan2(RT[1,0]*cos_theta_sign, RT[0,0]*cos_theta_sign) # phi
            # pos[3] = np.arctan2(RT[2,1]*cos_theta_sign, RT[2,2]*cos_theta_sign) # psi

            # np.matmul(Ja_ref, self.current_daq_velocities, out = vel_ref)
            # np.matmul(Ja, self.current_joint_velocities, out = vel)
            # vel_error = vel - vel_ref

            # # unwrap rotation position angle to go beyond -pi and pi
            # for i in range(3,6):
            #     if pos[i] - init_pos[i] > np.pi:
            #         pos[i] = pos[i] - 2*np.pi
            #     elif pos[i] - init_pos[i] < -np.pi:
            #         pos[i] = pos[i] + 2*np.pi

            #     if pos_ref[i] - init_pos_ref[i] > np.pi:
            #         pos_ref[i] = pos_ref[i] - 2*np.pi
            #     elif pos_ref[i] - init_pos_ref[i] < -np.pi:
            #         pos_ref[i] = pos_ref[i] + 2*np.pi

            # pos_error = pos - pos_ref
            # relative_pos_error = pos - init_pos - pos_ref + init_pos_ref
            # relative_pos_error[3] = -relative_pos_error[3]
            # relative_pos_error[4] = -relative_pos_error[4]

            # wrench = self.current_wrench
            # filtered_wrench = np.array(self.filter.filter(wrench))
            # np.matmul(RT, filtered_wrench[:3], out = wrench_global[:3])
            # np.matmul(RT, filtered_wrench[3:], out = wrench_global[3:])
            # # np.matmul(Ja, self.current_joint_velocities, out = end_effector_vel)
            # # self.est_wrench_int_term += (wrench_global - est_wrench_global) / sample_rate
            # # est_wrench_global = momentum_observer_gain * (1.0 * end_effector_inertia * end_effector_vel + self.est_wrench_int_term)
            # # np.matmul(Ja.transpose(), wrench_global, out = joint_desired_torque)

            # # online joint torque error id
            # # position_error = self.current_daq_positions + self.robot_ref_pos - self.current_joint_positions
            # if np.any(np.abs(relative_pos_error)>0.01) or np.any(np.abs(self.current_joint_velocities)>0.001):
            #     # joint_desired_torque_after_correction = joint_desired_torque - joint_torque_error
            #     wg = wrench_global - wrench_global_errorvel_ref_array
            # else:
            #     wrench_global_error_display = wrench_global_error_display + q / (1 + q) * (wrench_global - wrench_global_error_display)
            #     q = (q - q ** 2 / (1 + q)) / recent_data_focus_coeff
            #     wgd = wrench_global - wrench_global_error_display
            
            # load_mass_array = wg / 10
            # np.clip(load_mass_array,-100.0,0.0,load_mass_array)
            # self.load_mass.data = load_mass_array
            # if self.load_mass_publish_index < sample_rate / self.load_mass_publish_rate:
            #     self.load_mass_publish_index = self.load_mass_publish_index + 1
            # else:
            #     self.load_mass_publish_index = 0
            #     self.load_mass_pub.publish(self.load_mass)

            # # cartesian integration approach
            # ad = (wg - virtual_stiffness * relative_pos - 2 * zeta * np.sqrt(virtual_stiffness * inertia) * vel) / inertia
            # vd += ad / sample_rate
            # # turn on or turn off rotation
            # vd_tool[3:5] = np.array([0,0])
            # damped_psedu_inv = np.matmul(Ja.transpose(),np.linalg.inv(np.matmul(Ja,Ja.transpose()) + pin_damping**2*np.identity(6)))
            # np.matmul(np.linalg.inv(Ja), vd_tool, out = vd)
            # u,s,vh = np.linalg.svd(Ja)
            # # print(s)
            # np.matmul(damped_psedu_inv, vd_tool, out = vd)
            # np.clip(vd,-self.max_joint_speeds,self.max_joint_speeds,vd)

            # # joint space inertia
            # force = wg - coupling_stiffness * relative_pos_error - coupling_damping * vel_error
            # # force[:3] = wg[:3] - coupling_stiffness[:3] * relative_pos_error[:3] - coupling_damping[:3] * vel_error[:3]
            # # force[3:] = - coupling_stiffness[3:] * relative_pos_error[3:] - coupling_damping[3:] * vel_error[3:]
            # np.matmul(Ja.transpose(), force, out = joint_torque)
            # joint_ad = joint_torque / self.inertia_offset
            # np.clip(joint_ad,-self.max_joint_acc,self.max_joint_acc,joint_ad)
            # joint_vd += joint_ad / sample_rate
            # np.clip(joint_vd,-self.max_joint_speeds,self.max_joint_speeds,joint_vd)
            # # joint_pos = deepcopy(self.current_joint_positions)
            # # joint_pd = joint_pos + joint_vd / sample_rate
            # vel_lower_bound = (self.lower_lims - self.current_joint_positions) * sample_rate
            # np.clip(vel_lower_bound,-self.max_joint_speeds,0.0,vel_lower_bound)
            # vel_upper_bound = (self.upper_lims - self.current_joint_positions) * sample_rate
            # np.clip(vel_upper_bound,0.0,self.max_joint_speeds,vel_upper_bound)
            # np.clip(joint_vd,vel_lower_bound,vel_upper_bound,joint_vd)

            # vel_ref_array = joint_vd

            #enforce max velocity setting
            np.clip(vel_ref_array,-self.max_joint_speeds,self.max_joint_speeds,vel_ref_array)

            # self.test_data.data = vel_error
            self.test_data_pub.publish(self.test_data)

            #publish
            self.vel_ref.data = vel_ref_array
            self.vel_ref.data = 0*vel_ref_array #TODO remove safety calculation
            # self.ref_vel_pub.publish(self.vel_ref)
            self.vel_pub.publish(self.vel_ref)
            #wait
            rate.sleep()
        self.stop_arm(safe = True)

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
            current_homing_pos = deepcopy(self.current_daq_positions * joint_inversion + self.daq_ref_pos)
            if not self.identify_joint_lim(current_homing_pos):
                print('Homing desired position outside robot position limit. Please change dummy position')
                continue

            self.is_homing = True
            self.is_homing_pub.publish(self.is_homing)
            reached_goal = self.move_to(current_homing_pos,speed = 0.2,override_initial_joint_lims=True,require_enable = True)
            self.is_homing = False
            self.is_homing_pub.publish(self.is_homing)
            if reached_goal:
                #start moving
                print('Starting Free Movement')
                self.move(capture_start_as_ref_pos = True,
                          dialoge_enabled = False)

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

if __name__ == "__main__":
    #This script is included for testing purposes
    print("starting")

    arm = ur5e_arm(test_control_signal=True, conservative_joint_lims = False)
    time.sleep(1)
    arm.stop_arm()


    # print(arm.remote_control_running().program_running)
    # raw_input('waiting')
    # print(arm.remote_control_running().program_running)

    # arm.calibrate_control_arm_zero_position(interactive = True)
    # print(arm.move_to(arm.default_pos, speed = 0.1, override_initial_joint_lims=True))
    print(arm.move_to_robost(arm.default_pos,
                             speed = 0.1,
                             override_initial_joint_lims=True,
                             require_enable = True))
    arm.set_ref_pose_to_current()
    print(arm.cartesian_ref_pose)
    # arm.capture_control_arm_ref_position()
    # print("Current Arm Position")
    # print(arm.current_joint_positions)
    # print("DAQ position:")
    # print(arm.current_daq_positions)
    # print(arm.move_to_robost(arm.default_pos,
    #                          speed = 0.1,
    #                          override_initial_joint_lims=True,
    #                          require_enable = True))
    # if 'y'==raw_input('Execute Move? (y/n)'):
    #     target_pos = arm.default_pos + daq_offset
    #     arm.move_to(target_pos, speed = 0.1, override_initial_joint_lims=False)
    pose = arm.kdl_kin.forward(arm.current_joint_positions)
    print(pose)
    raw_input("Hit enter when ready to move")
    # arm.move()
    # arm.move(capture_start_as_ref_pos=True)
    arm.run()

    arm.stop_arm()


    # target_pos = arm.default_pos
    # target_pos[5]+=1.0
    # arm.move_to(target_pos, speed = 0.1, override_initial_joint_lims=True)
