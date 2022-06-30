#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 

c=0 # Counter to be updated every time callback() is excecuted

def callback(data):
  global c
  c+=1
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame: %d" %c)
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data, desired_encoding='mono8') # Encoding is 'mono8' for the IR image and 'passthrough' with the RGB image
  

  # Reducing the image size to 60%, to see it fully on the screen
  shape= current_frame.shape
  dims=(int(0.6*shape[1]), int(0.6*shape[0]))
  resized = cv2.resize(current_frame, dims, interpolation = cv2.INTER_AREA)
 
   
  # Display image
  cv2.imshow("camera", resized)
  
  # Naming the frame 
  file_name="frame%d.jpg" % c

  # Save frame if the Spaace button in hit...
  if cv2.waitKey(10) == 32:
    cv2.imwrite(file_name, current_frame)     # save frame as JPEG file
    

  cv2.waitKey(1)
  
      
def receive_message():
  global c
  c+=1

  rospy.init_node('video_sub_py', anonymous=True)
   
  rospy.Subscriber('/k4a/rgb/image_rect_color', Image, callback) # '/k4a/rgb/image_raw' for the RGB image
 
  rospy.spin()

  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
