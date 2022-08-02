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
