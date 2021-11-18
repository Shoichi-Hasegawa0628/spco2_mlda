#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import re
import glob
import random
import collections
import numpy as np
import scipy as sp
from numpy.random import multinomial  #,uniform #,dirichlet
#from scipy.stats import t             #,multivariate_normal,invwishart,rv_discrete
#from numpy.linalg import inv, cholesky
from math import pi as PI
from math import cos,sin,sqrt,exp,log,fabs,fsum,degrees,radians,atan2,gamma,lgamma
#from sklearn.cluster import KMeans
from multiprocessing import Pool
from multiprocessing import Process
import multiprocessing
from __init__ import *
from spco2_math import *
import csv # Takeshi Nakashima 2021/03/06
import rospy
from std_msgs.msg import String
import std_msgs.msg
import sys
import os.path
import time
import shutil

step = 1
trialname = "test"
datafolder   = "/root/HSR/catkin_ws/src/spco2_mlda/rgiro_spco2_slam/data/output/"
E = 365
chi0  = 0.1
D = 3
lamb = 0.1

FT = []
for s in range(step):
    for line in open( datafolder + trialname + '/img/ft' + str(s+1) + '.csv', 'r'):
    #for line in open( datasetfolder + datasetname + 'img/ft' + str(s+1) + '.csv', 'r'):
        itemList = line[:].split(',')
        FT.append( [float(itemList[i]) for i in range(DimImg)] )
print("FT: {}".format(FT))
print(type(FT[0][0]))
Nle_c = [ sum( [np.array(FT[0])] ) ]
print(Nle_c)
theta = [(np.array(Nle_c[0]) + chi0 ) / (sum(Nle_c[0]) + E*chi0)]
print(theta)

OT = []
ot = [0,0,0]
OT.append(ot)
print("OT: {}".format(OT))
Nld_c = [ sum( [np.array(OT[0])] ) ]
print(Nld_c)
Xi = [(np.array(Nld_c[0]) + lamb) / (sum(Nld_c[0]) + D*lamb)]
print(Xi)