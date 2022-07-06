#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

published=True


def callback(data):
    global published
    target_JS=[-0.039186552054350135, 0.030171243235703894, 0.5320185192160342, -1.821231455345683, -0.9745802034125142, 2.8536811608592685, -2.2519029422323134]
    target_JS=list(map(lambda x: round(x,3),target_JS))
    data=data.position
    rounded=list(map(lambda x: round(x,3),data))
    # rospy.loginfo((rounded))
    if target_JS==rounded and published:
        pub.publish(True)
        published=False
        print(published)
def flag():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('flag', anonymous=True)
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub=rospy.Publisher("/flag_point_cloud", Bool, queue_size=0)
    flag()