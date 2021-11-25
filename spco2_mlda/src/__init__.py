#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os

import roslib.packages

SPCO_PARAM_PATH = str(roslib.packages.get_pkg_dir("rgiro_spco2_slam")) + "/data/output/test/max_likelihood_param/"
MLDA_PARAM_PATH = str(roslib.packages.get_pkg_dir("mlda")) + "/data/"

CODE_BOOK_SIZE = 50
ITERATION = 100
CATEGORYNUM = 3
ALPHA = 1.0
BETA = 1.0

LEARN_RESULT_FOLDER = "/root/HSR/catkin_ws/src/spco2_mlda/mlda/mlda/data/learn_result"
ESTIMATE_RESULT_FOLDER = "/root/HSR/catkin_ws/src/spco2_mlda/mlda/mlda/data/estimate_result"
PROCESSING_DATA_FOLDER = "/root/HSR/catkin_ws/src/spco2_mlda/mlda/mlda/data/processing_data"
LEARNING_DATASET_FOLDER = str(roslib.packages.get_pkg_dir("mlda_dataset_original")) + "/rsj_exp"
OBJECT_CLASS = os.listdir(LEARNING_DATASET_FOLDER)
OBJECT_NAME = []

for c in range(len(OBJECT_CLASS)):
    OBJECT_NAME.append(os.listdir(LEARNING_DATASET_FOLDER + "/" + OBJECT_CLASS[c]))

TEACHING_DATA = "word/teaching_text.txt"
WORD_DICTIONARY = "word_dic.txt"
WORD_HIST = "histgram_word.txt"
IMG_HIST = "histgram_img.txt"
CODE_BOOK = "codebook.txt"


PRE_LEARNING_OBJECT_DATA = str(roslib.packages.get_pkg_dir("spco2_mlda")) + "/data/pre_learning"
OBJECT_SEARCH_DATA = str(roslib.packages.get_pkg_dir("spco2_mlda")) + "/data/object_search"

PRE_OBSERVATION = PRE_LEARNING_OBJECT_DATA  + "/observation/"
PRE_RESIZE = PRE_LEARNING_OBJECT_DATA  + "/resize/"
PRE_CROP = PRE_LEARNING_OBJECT_DATA  + "/crop/"
PRE_YOLO = PRE_LEARNING_OBJECT_DATA + "/yolo/"
PRE_CROP_YOLO = PRE_LEARNING_OBJECT_DATA + "/crop_yolo/"

SEARCH_OBSERVATION = OBJECT_SEARCH_DATA  + "/observation/"
SEARCH_RESIZE = OBJECT_SEARCH_DATA  + "/resize/"
SEARCH_CROP = OBJECT_SEARCH_DATA  + "/crop/"
SEARCH_YOLO = OBJECT_SEARCH_DATA + "/yolo/"
SEARCH_CROP_YOLO = OBJECT_SEARCH_DATA + "/crop_yolo/"