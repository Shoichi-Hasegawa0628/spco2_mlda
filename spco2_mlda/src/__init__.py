#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib.packages

SPCO_PARAM_PATH = str(roslib.packages.get_pkg_dir("rgiro_spco2_slam")) + "/data/output/test/max_likelihood_param/"
MLDA_PARAM_PATH = str(roslib.packages.get_pkg_dir("mlda")) + "/data/"