#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
published=True
def callback(data):
    global published
    target_JS=[-1.266, 0.75, 2.703, -2.408, -1.98, 1.311, -0.028]
    data=data.position
    rounded=list(map(lambda x: round(x,3),data))
    rospy.loginfo((rounded))
    if target_JS==rounded and published:
        pub.publish(1)
        published=False
        print(published)
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
    pub=rospy.Publisher("flag", Int16, queue_size=0)
    flag()