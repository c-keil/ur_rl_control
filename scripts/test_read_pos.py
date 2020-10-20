#! /usr/bin/env python

import rospy
from ur5teleop.msg import jointdata, Joint


def callback(data):
    print(data.encoder1.pos)

def main():

    rospy.init_node('daq_listener', anonymous=True)

    rospy.Subscriber("daqdata_filtered", jointdata, callback)

    # time.sleep(0.5)
    rospy.spin()

if __name__ == "__main__":
    main()
