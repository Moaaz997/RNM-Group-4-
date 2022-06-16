#!/usr/bin/python3

import rospy
import sys
from sensor_msgs.msg import JointState
import math
import numpy
from sympy import Matrix
from sympy import *

init_printing(use_unicode=True)


def callback(msg):
    pos = zeros(7, 1)
    for x in range(len(pos)):
        pos[x] = msg.position
        print(pos)
        print(pos.shape)
    mtrans(pos)


def mtrans(joint):
    a1 = Matrix([[math.cos(joint[0]), -math.sin(joint[0]), 0, 0],
                 [math.sin(joint[0]), math.cos(joint[0]), 0, 0],
                 [0, 0, 1, 0.333], [0, 0, 0, 1]])
    a2 = Matrix([[math.cos(joint[1]), 0, -math.sin(joint[1]), 0],
                 [math.sin(joint[1]), 0, math.cos(joint[1]), 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])
    a3 = Matrix([[math.cos(joint[2]), 0, math.sin(joint[2]), 0],
                 [math.sin(joint[2]), 0, -math.cos(joint[2]), 0],
                 [0, 1, 0, 0.316],
                 [0, 0, 0, 1]])
    a4 = Matrix([[math.cos(joint[3]), 0, math.sin(joint[3]), 0.0825 * math.cos(joint[3])],
                 [math.sin(joint[3]), 0, -math.cos(joint[3]), 0.0825 * math.cos(joint[3])],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    a5 = Matrix([[math.cos(joint[4]), 0, -math.sin(joint[4]), -0.0825 * math.cos(joint[4])],
                 [math.sin(joint[4]), 0, math.cos(joint[4]), -0.0825 * math.cos(joint[4])],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]])
    a6 = Matrix([[math.cos(joint[5]), 0, math.sin(joint[5]), 0],
                 [math.sin(joint[5]), 0, -math.cos(joint[5]), 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    a7 = Matrix([[math.cos(joint[6]), 0, math.sin(joint[6]), 0.088 * math.cos(joint[6])],
                 [math.sin(joint[6]), 0, -math.cos(joint[6]), 0.088 * math.cos(joint[6])],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])
    finalmatrix = a1*a2*a3*a4*a5*a6
    print(finalmatrix)


def main(argv):
    # Initialize ROS connection
    # The script with the main function needs to be executable.
    # You can make the file executable by executing 'chmod +x /path/to/file.py'
    # in the terminal (without the "'")
    rospy.init_node("panda_joint_reader_node", argv)

    # Read parameters specified in the .launch file
    # The "~" in the beginning means that it will use parameters from the node
    # e.g. /node_name/parameter_name can then be
    # read by rospy.get_param("parameter_name", default)
    topic_name = rospy.get_param("~topic_name", "/default_topic_name")
    queue_size = rospy.get_param("~queue_size", 10)

    # Register a callback function (a function that is called every time a new message arrives)
    rospy.Subscriber(name=topic_name,
                     data_class=JointState,
                     callback=callback,
                     queue_size=queue_size)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
