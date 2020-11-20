#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from ur5teleop.msg import jointdata, Joint
from copy import deepcopy
import time

joint_reorder = [2,1,0,3,4,5]
lower_lims = np.array([0.0, -100.0, 0.0, -180.0, -180.0, 90.0])
upper_lims = np.array([180.0, 0.0, 150.0, 0.0, 0.0, 270.0])
default_pos = np.array([90.0, -90.0, 90.0, -90.0, -90, 180.0])

current_joint_states = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
absolute_ref_pos = np.array([0.0])
ref_vel = np.array([0.0])

def joint_state_callback(data):
    # current_joint_states[joint_reorder] = data.position
    current_joint_states[0] = data.position[2]
    current_joint_states[1] = data.position[1]
    current_joint_states[2] = data.position[0]
    current_joint_states[3] = data.position[3]
    current_joint_states[4] = data.position[4]
    current_joint_states[5] = data.position[5]

def daq_callback(data):
    absolute_ref_pos[0] = data.encoder1.pos
    ref_vel[0] = data.encoder1.vel

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joint_listener', anonymous=True)

    rospy.Subscriber("joint_states", JointState, joint_state_callback)
    rospy.Subscriber("daqdata_filtered", jointdata, daq_callback)
    pub = rospy.Publisher("/joint_group_vel_controller/command",
                        Float64MultiArray,
                        queue_size=1)
    time.sleep(0.5)

    velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_ref = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_lim = 3.0
    start_positon = deepcopy(current_joint_states)
    goal_postion = np.array([0.,0.,0.,0.,0.,0.])
    feedforward_term = np.array([0.,0.,0.,0.,0.,0.])
    print(goal_postion)
    # goal_postion[5] += 2.0
    print(goal_postion)

    #get initial encoder position
    start_encoder_pos = deepcopy(absolute_ref_pos[0])



    # input("")
    # p_gain = 20.0
    p_gain = 5.0
    k_ff = 1.0
    # spin() simply keeps python from exiting until this node is stopped
    start_time = time.time()
    while time.time()-start_time < 10.0:
        # print("Goal: {}, Current {}".format(goal_postion,current_joint_states))
        relative_postion = absolute_ref_pos[0] - start_encoder_pos
        goal_postion[5] = relative_postion
        position_error = start_positon + goal_postion - current_joint_states
        # print(position_error)
        feedforward_term[5] = k_ff*ref_vel[0]
        vel_ref_temp = p_gain*position_error + feedforward_term
        np.clip(vel_ref_temp,-vel_lim,vel_lim,vel_ref_temp)
        # print(vel_ref_temp)
        vel_ref.data = vel_ref_temp
        pub.publish(vel_ref)

    velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pub.publish(velocity)
    print("Stop")

if __name__ == '__main__':
    listener()
