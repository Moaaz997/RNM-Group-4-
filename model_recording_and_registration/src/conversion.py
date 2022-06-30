#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import open3d
import numpy as np
from ctypes import * # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

flag=True

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromRosToOpen3d(ros_cloud):
    print("ben-convert aho..")
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

# global received_ros_cloud
received_ros_cloud = None
def callback(ros_cloud):
    def func():
        print("is shudown now")

    global flag
    if flag:
        print("teezak")
        global received_ros_cloud
        received_ros_cloud=ros_cloud
        rospy.loginfo("-- Received ROS PointCloud2 message.")
        flag=False
        rospy.on_shutdown(func)


# -- Example of usage
if __name__ == "__main__":
    rospy.init_node('test_pc_conversion_between_Open3D_and_ROS', anonymous=True)
    
    # # -- Read point cloud from file
    import os
    PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"


    while not rospy.is_shutdown():
        print("taztooza")
        rospy.Subscriber('/points2', PointCloud2, callback)
        rospy.spin()
    
    # -- After subscribing the ros cloud, convert it back to open3d, and draw
    # print(received_ros_cloud)

    received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
    print(received_open3d_cloud)
    open3d.visualization.draw_geometries([received_open3d_cloud])

    # write to file

    output_filename=PYTHON_FILE_PATH+"conversion_result.pcd"
    open3d.io.write_point_cloud(output_filename, received_open3d_cloud)
    rospy.loginfo("-- Write result point cloud to: "+output_filename)

    # draw
    # open3d.draw_geometries([received_open3d_cloud])
    rospy.loginfo("-- Finish display. The program is terminating ...\n")