#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from copy import deepcopy
import time


current_joint_states = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

def joint_state_callback(data):
    current_joint_states[0] = data.position[2]
    current_joint_states[1] = data.position[1]
    current_joint_states[2] = data.position[0]
    current_joint_states[3] = data.position[3]
    current_joint_states[4] = data.position[4]
    current_joint_states[5] = data.position[5]

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joint_listener', anonymous=True)

    rospy.Subscriber("joint_states", JointState, joint_state_callback)
    pub = rospy.Publisher("/joint_group_vel_controller/command",
                        Float64MultiArray,
                        queue_size=1)
    time.sleep(0.5)

    velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_ref = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_lim = 3.0
    start_positon = current_joint_states
    goal_postion = deepcopy(start_positon)
    print(goal_postion)
    goal_postion[5] += 2.0
    print(goal_postion)
    # input("")
    p_gain = 20.0
    # spin() simply keeps python from exiting until this node is stopped
    start_time = time.time()
    while time.time()-start_time < 5.0 :
        # print("Goal: {}, Current {}".format(goal_postion,current_joint_states))
        position_error = goal_postion - current_joint_states
        # print(position_error)
        vel_ref_temp = p_gain*position_error
        np.clip(vel_ref_temp,-vel_lim,vel_lim,vel_ref_temp)
        # print(vel_ref_temp)
        vel_ref.data = vel_ref_temp
        pub.publish(vel_ref)

    velocity = Float64MultiArray(data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pub.publish(velocity)
    print("Stop")

if __name__ == '__main__':
    listener()
