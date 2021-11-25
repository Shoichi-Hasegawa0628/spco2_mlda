#!/usr/bin/env python
# -*- coding: utf-8 -*-
# YOLOで検出した物体から, 識別精度0.5以上の物体のみtarget_objectとして物体の画像領域だけを切り抜くプログラム

# Standard Library
import time
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
from subprocess import *

# Third Party
import rospy
import numpy as np
# from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from yolo_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image, CompressedImage

# Self-made Modules
from spco2_mlda.srv import SendImageYOLO
from spco2_mlda.srv import SendImageYOLOResponse
from __init__ import *


class CropImg():

    def __init__(self):
        rospy.Service('crop_img', SendImageYOLO, self.crop_object_img_with_yolo)
        self.cv_bridge = CvBridge()
        self.detect_objects_info = []
        self.detect_pre_img = 0

    def crop_object_img_with_yolo(self, msg):
        time.sleep(3.0)
        print('clear')
        try:
            # bb = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout=10)
            bb = rospy.wait_for_message('/yolov5_ros/output/bounding_boxes', BoundingBoxes, timeout=None)
        except:

            return SendImageYOLOResponse(success=False)
        time.sleep(3.0)
        self.detect_objects_info = bb.bounding_boxes
        img = 0
        if msg.count != 0:
            while True:
                # img = rospy.wait_for_message('/darknet_ros/detection_image', Image, timeout=15)
                img = rospy.wait_for_message('/yolov5_ros/output/image/compressed', CompressedImage, timeout=15)
                img = self.cv_bridge.compressed_imgmsg_to_cv2(img)
                status = np.array_equal(img, self.detect_pre_img)
                if status is False:
                    break

        else:
            # img = rospy.wait_for_message('/darknet_ros/detection_image', Image, timeout=15)
            img = rospy.wait_for_message('/yolov5_ros/output/image/compressed', CompressedImage, timeout=15)
            img = self.cv_bridge.compressed_imgmsg_to_cv2(img)
        self.detect_pre_img = img

        # img = rospy.wait_for_message('/darknet_ros/detection_image', Image, timeout=15)
        # img = rospy.wait_for_message('/yolov5_ros/output/image/compressed', CompressedImage, timeout=15)
        #img = self.cv_bridge.compressed_imgmsg_to_cv2(img)
        #self.detect_pre_img = img

        object_list = self.detect_objects_info
        observed_img = self.cv_bridge.compressed_imgmsg_to_cv2(msg.observed_img)
        observed_img = cv2.cvtColor(observed_img, cv2.COLOR_BGR2RGB)
        # cv2.imshow('color', img)
        # cv2.waitKey(3000)
        img_num = msg.count

        for i in range(len(object_list)):
            base_time = time.time()
            while len(self.detect_objects_info) == 0:
                if time.time() - base_time > 25:
                    return SendImageYOLOResponse(success=False)
                continue
            time.sleep(3.0)
            yolo_img = img
            if object_list[i].probability >= 0.5:

                # height_o = observed_img.shape[0]
                # width_o = observed_img.shape[1]

                crop_img = observed_img[object_list[i].ymin: object_list[i].ymax,
                           object_list[i].xmin: object_list[i].xmax]
                crop_yolo_img = yolo_img[object_list[i].ymin: object_list[i].ymax,
                                object_list[i].xmin: object_list[i].xmax]
                height_cut = crop_img.shape[0]
                width_cut = crop_img.shape[1]
                # cut_img_yolov3 = cv2.cvtColor(cut_img_yolov3, cv2.COLOR_BGR2RGB)
                # cut_img_resize = cv2.resize(cut_img , (int(width_o), int(height_o))) # observationの大きさに拡張
                crop_img_resize = cv2.resize(crop_img, (int(width_cut) * 2, int(height_cut) * 2))  # 2倍にする
                print("Saving preparation...")
                if msg.mode == "0":
                    if os.path.exists(PRE_CROP + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(PRE_CROP + "{}".format(img_num))

                    if os.path.exists(PRE_CROP_YOLO + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(PRE_CROP_YOLO + "{}".format(img_num))

                    if os.path.exists(PRE_YOLO + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(PRE_YOLO + "{}".format(img_num))

                    if os.path.exists(PRE_RESIZE + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(PRE_RESIZE + "{}".format(img_num))

                    print("Saving image...")
                    cv2.imwrite(PRE_CROP + "{}/crop_img_{}.jpg".format(img_num, i), crop_img)
                    cv2.imwrite(PRE_CROP_YOLO + "{}/crop_yolo_img_{}.jpg".format(img_num, i), crop_yolo_img)
                    cv2.imwrite(PRE_YOLO + "{}/yolo_img_{}.jpg".format(img_num, i), yolo_img)
                    cv2.imwrite(PRE_RESIZE + "{}/crop_img_resize_{}.jpg".format(img_num, i), crop_img_resize)
                    print("Displayed save data")
                    cv2.imshow('color', crop_img)
                    cv2.waitKey(3000)
                    cv2.imshow('color', crop_img_resize)
                    cv2.waitKey(3000)
                    cv2.destroyAllWindows()
                    cv2.waitKey(1)

                else:
                    if os.path.exists(SEARCH_CROP + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(SEARCH_CROP + "{}".format(img_num))

                    if os.path.exists(SEARCH_CROP_YOLO + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(SEARCH_CROP_YOLO + "{}".format(img_num))

                    if os.path.exists(SEARCH_YOLO + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(SEARCH_YOLO + "{}".format(img_num))

                    if os.path.exists(SEARCH_RESIZE + "{}".format(img_num)) is True:
                        pass
                    else:
                        os.mkdir(SEARCH_RESIZE + "{}".format(img_num))

                    cv2.imwrite(SEARCH_CROP + "{}/crop_img_{}.jpg".format(img_num, i), crop_img)
                    cv2.imwrite(SEARCH_CROP_YOLO + "{}/crop_yolo_img_{}.jpg".format(img_num, i), crop_yolo_img)
                    cv2.imwrite(SEARCH_YOLO + "{}/yolo_img_{}.jpg".format(img_num, i), yolo_img)
                    cv2.imwrite(SEARCH_RESIZE + "{}/crop_img_resize_{}.jpg".format(img_num, i), crop_img_resize)
                    cv2.imshow('color', crop_img)
                    cv2.waitKey(3000)

        print("OK")
        return SendImageYOLOResponse(success=True)

    # def image_ros_to_opencv(self, img):
    #     try:
    #         observed_img = img
    #         observed_img = self.cv_bridge.imgmsg_to_cv2(observed_img, 'passthrough')
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #
    #     # observed_img = cv2.cvtColor(observed_img, cv2.COLOR_BGR2RGB)
    #     return observed_img


if __name__ == "__main__":
    rospy.init_node('crop_img')
    CropImg()
    rospy.spin()
