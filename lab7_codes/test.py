import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import cv2
while(True):
    rospy.init_node('webcam_puber', anonymous=True)
# queue_size should be small in order to make it 'real_time'
# or the node will pub the past_frame
    img_pub = rospy.Publisher('/republish', Image, queue_size=2)
    rate = rospy.Rate(10) # 5hz


