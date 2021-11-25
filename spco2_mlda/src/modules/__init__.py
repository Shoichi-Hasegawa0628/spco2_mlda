#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
import random
import subprocess
import math

#from mlda_ros.srv import *
#from mlda_ros.msg import *

import sys
#reload(sys)
#sys.setdefaultencoding('utf-8')
import roslib.packages


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


