#!/usr/bin/env python
# -*- coding: utf-8 -*-
# MLDAに処理対象の画像を送り、推定結果を受け取るプログラム

# Standard Library
import sys

# Third Party
import rospy
import roslib.packages

# Self-made Modules
# from spco2_mlda.srv import SendImageMLDA
# from spco2_mlda.srv import SendImageMLDAResponse
sys.path.append(str(roslib.packages.get_pkg_dir("mlda")) + "/scripts")
import extract_img_bof
import execute_node
import extract_word_bow

mlda_extract_img_bof = extract_img_bof.ExtractImgBof()
mlda_learn = execute_node.MLDA()
mlda_extract_word_bow = extract_word_bow.ExtractWordBow()

class ConnectMLDA():
    
    def __init__(self):
        #rospy.Service('judge_mlda', SendImageMLDA, self.judge_target_object_mlda)
        pass

    def receive_mlda_result(self, yolov3_image, status, observed_img_idx, count, mode):
        if status == "estimate":
            result = mlda_extract_img_bof.img_server(status, count, observed_img_idx, yolov3_image, mode)
            if result == 0:
                success = False
                return success
            mlda_learn.mlda_server(status, observed_img_idx, count, mode)
            success = True
            if mode == "0":
                return success

        #### Search用
        # if status == "estimate":
        #     result = mlda_extract_img_bof.img_server(status, count, observed_img_idx, yolov3_image, mode)
        #     if result == 0:
        #         success = False
        #         object_word = None
        #         object_prob = None
        #         return success, object_word, object_prob
        #     mlda_learn.mlda_server(status, observed_img_idx, count, mode)
        #     if mode == "0":
        #         return
        #     result = mlda_extract_word_bow.word_server(yolov3_image, status, observed_img_idx, count)
        #
        # object_word = []
        # # object_word_list = []
        # object_prob = []
        # # object_prob_list = []
        #
        # # print(result)
        # for w in result.keys():
        #     key_start = w.find('_')
        #     # key_goal = len(w)
        #     object_word.append(w[key_start + 1:])
        #     # print(object_word)
        # # object_word_list.append(object_word)
        #
        # for p in result.values():
        #     object_prob.append(p)
        #
        # # print ("Finished!!!")
        # success = True
        # # return SendImageMLDAResponse(success = True, object_word = "apple", object_word_probability = 0.1)
        # return success, object_word, object_prob

if __name__ == "__main__":
    rospy.init_node('connect_mlda')
    ConnectMLDA()
    rospy.spin()