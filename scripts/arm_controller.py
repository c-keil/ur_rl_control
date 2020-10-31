#! /usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
import time

from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import JointState
from ur5teleop.msg import jointdata, Joint
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
# Args: start_controllers stop_controllers strictness start_asap timeout

def switch_to_joint_group_vel_controller():
    '''calls the conntroller manager switch controller topic if the vel controller
    is not already running'''
    contollers = rosserv.call('/controller_manager/list_controllers')

#define initial state

# # TODO Arm class
# class ur5e_arm():
#     '''Defines velocity based controller for ur5e arm for use in teleop project
#     def __init__():
#     '''

if __name__ == "__main__":
    print(starting)
