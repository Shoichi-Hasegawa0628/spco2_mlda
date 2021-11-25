#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOとMLDAを使用して画像中の物体を推論するプログラム

# Standard Library
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Third Party
import rospy
from std_msgs.msg import String
import roslib.packages

sys.path.append(str(roslib.packages.get_pkg_dir("mlda")) + "/scripts")
import extract_img_bof
import execute_node
import extract_word_bow

mlda_extract_img_bof = extract_img_bof.ExtractImgBof()
mlda_learn = execute_node.MLDA()
mlda_extract_word_bow = extract_word_bow.ExtractWordBow()

# Self-made Modules
from modules import  (
    capture_img,
    send_img,
    connect_mlda
)
from __init__ import *

capture_img_func = capture_img.CaptureImg()
send_img_func = send_img.SendImg()
connect_mlda_func = connect_mlda.ConnectMLDA()

class ObjectRecognitionYOLO2MLDA():

    def __init__(self):
        self.cv_bridge = CvBridge()


    def selection_mode(self, mode):
        #mode = rospy.wait_for_message("/object_recog_flag", String, timeout=None)
        # if mode.data == '0': # SpCoSLAM-MLDAの事前学習

        if mode == "0":
            print("Taking a picture !")
            capture_img_func.taking_single_image()
            print("Sending a image !")
            success = send_img_func.send_img_to_yolo(mode)

            if success is True:
                return 1
            else:
                return 0

            # # 動作確認用
            # count = 0
            # observed_img_idx = 0
            # status = "estimate"
            # yolov3_image = cv2.imread("/root/HSR/catkin_ws/src/spco2_mlda/spco2_mlda/data/pre_learning/resize/0/crop_img_resize_0.jpg")
            # yolov3_image = self.cv_bridge.cv2_to_imgmsg(yolov3_image, encoding="bgr8")
            # print(type(yolov3_image))
            # connect_mlda_func.receive_mlda_result(yolov3_image, status, observed_img_idx, count, mode)

        else:              # 物体探索時
            pass


if __name__ == "__main__":
    rospy.init_node('object_recognition_yolo2mlda')
    # ObjectRecognitionYOLO2MLDA()
    yolo2mlda = ObjectRecognitionYOLO2MLDA()
    #yolo2mlda.selection_mode()
    rospy.spin()