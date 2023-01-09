#!/usr/bin/python2
# -*- coding: utf-8 -*-
import time
import rospy
import sys
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
# sys.path.append("/home/bo/catkin_ws/src/racecar/racecar_gazebo/scripts/")
from cv_bridge import CvBridge, CvBridgeError
image_path  = '/home/bo/data/'
count=0
def image_callback(image_data):
    global bridge,count
    #timestr = "%.6f" % image_data.header.stamp.to_sec()
    frame = bridge.imgmsg_to_cv2(image_data, "bgr8")
    image_name = str(count) + ".jpg"  # 图像命名：时间戳.jpg
    cv2.imwrite(image_path + image_name, frame)  # 保存；
    print("图片保存成功:",image_name)
    rospy.Rate(20).sleep()
    count+=1


def main():
    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            # frame = cv2.flip(frame, 0)  ##图像上下颠倒
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

            cv2.imshow('frame', frame)
            cv2.waitKey(3)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('Server_Socket', anonymous=True)  # 初始化节点
        global bridge,cap,image_pub
        bridge = CvBridge()
        # cap = cv2.VideoCapture("/dev/video0")
        # cap.set(3, 640)#weight
        # cap.set(4, 480)#height

        # image_pub = rospy.Publisher('/image', Image,queue_size=10)#发布话题
        rospy.Subscriber("/raspicam_node/image_raw", Image,image_callback,queue_size = 10)#(订阅的话题名称，数据类型，回调函数（一直监听话题是否传来消息），队列长度)
        # main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

