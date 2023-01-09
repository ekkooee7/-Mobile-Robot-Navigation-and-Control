#!/usr/bin/env python
#!coding=utf-8

# 使用ros_numpy来转换接受的图像格式

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import cv2


def callback(data):
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        count = 0
        cv_img = ros_numpy.numpify(data)
        timestr = "%.6f" %  data.header.stamp.to_sec()
        cv2.imshow("frame" , cv_img)
        cv2.waitKey(10)
        
    else:
        pass
 
def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
    print(2222)
    # make a video_object and init the video object
    global count,bridge
    count = 0
    print(3333)
    rospy.Subscriber('/raspicam_node/image_raw', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':
    print(1111)
    displayWebcam()



