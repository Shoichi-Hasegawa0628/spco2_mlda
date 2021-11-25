#!/usr/bin/env python
# -*- coding: utf-8 -*-
# カメラで画像を撮影するプログラム

# Standard Library
import math
import time
import cv2
from subprocess import *
from cv_bridge import CvBridge, CvBridgeError

# Third Party
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty

# Self-made Modules
from __init__ import *



class CaptureImg():

    def __init__(self):
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        # rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.image_callback, queue_size=10)
        self.vel = Twist()
        self.cv_bridge = CvBridge()
        # self.image = 0
        #self.taking_multiple_images()

    def taking_single_image(self):
        img = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color/compressed', CompressedImage, timeout=None)
        observed_img = self.rgb_image_ros_to_opencv(img)
        cv2.imwrite(PRE_OBSERVATION + 'observed_img.jpg', observed_img)
        return

    def taking_multiple_images(self):
        rospy.wait_for_message("/next_judge", Empty, timeout=None)
        time.sleep(10)
        rotations = 13
        for rotation in range(rotations):

            # π/6 (rad/s)で回転
            self.vel.angular.z = math.pi / 6
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)
            time.sleep(1.0)
            print("The number of Rotation: ", rotation)

            # RGB画像を保存
            if not rotation == 12:
                img = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, timeout=None)
                object_image = self.rgb_image_ros_to_opencv(img)
                cv2.imwrite(SEARCH_OBSERVATION + 'object_image_{}.jpg'.format(rotation), object_image)

        time.sleep(1.0)
        # self.kill_node('take_observed_image')
        return

    # def image_callback(self, img):
    #    self.image = img

    def rgb_image_ros_to_opencv(self, img):
        observed_img = self.cv_bridge.compressed_imgmsg_to_cv2(img)
        return observed_img
        # try:
        #     object_image = img
        #     object_image = self.cv_bridge.imgmsg_to_cv2(object_image, 'passthrough')
        # except CvBridgeError as e:
        #     rospy.logerr(e)
        #
        # object_image = cv2.cvtColor(object_image, cv2.COLOR_BGR2RGB)
        # return object_image

    def kill_node(self, nodename):
        p2 = Popen(['rosnode', 'list'], stdout=PIPE)
        p2.wait()
        nodelist = p2.communicate()
        nd = nodelist[0]
        nd = nd.split("\n")
        for i in range(len(nd)):
            tmp = nd[i]
            ind = tmp.find(nodename)
            if ind == 1:
                call(['rosnode', 'kill', nd[i]])
                break


if __name__ == "__main__":
    rospy.init_node('capture_img')
    CaptureImg()
