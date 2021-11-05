#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SpCoSLAM-MLDAとMLDAの学習済みパラメータを用いて、物体の単語から場所の単語をクロスモーダル推論するコード

# Standard Library
import csv

# Third Party
from scipy.stats import dirichlet                                                                           #ディリクレ分布を使用するためのライブラリ
from scipy.stats import multinomial                                                                         #カテゴリ分布と多項分布を使用するためのライブラリ (多項は複数試行)
from scipy import stats
import numpy as np
import roslib.packages
import rospy
from std_msgs.msg import String

# Self-made Modules
from __init__ import *

class AncestralSamplingObject2Place():

    def __init__(self):
        self.object_name = 0
        pass

    def word_callback(self):
        """
        word = rospy.wait_for_message("/human_command", String, timeout=None)
        object_name = word.data
        print(object_name)
        self.object_name = object_name
        """

        # w_sの伝承サンプリング (10点サンプリング)
        target_name = ["pig", "doll"]
        result = []
        cat_ws_list = []
        samples = 10

        # 場所の単語一覧をロード
        with open(SPCO_PARAM_PATH + 'W_list.csv') as f:
            reader = csv.reader(f)
            for row in reader:
                pass
            place_name_list = row

        for name in range(len(target_name)):
            #target_name = "pig"
            
            for sample in range(samples):
                # π^oのサンプリング (ディリクレ分布)                                                                #どの物体のトピックが出現するかの確率を表現
                # (π^o)_1 ~ P( (π^o)_1 | α^o )
                alpha = np.array([1.0, 1.0, 1.0])
                pi_o = dirichlet.rvs(alpha, size=1, random_state=None)
                #print("(pi_o)_1 : {}".format(pi_o)) 



                # (C^o)_tのサンプリング式
                # P((C^o)_(t,1) | (w^o)_(t,1), θ^(o,w), (π^o)_1) ∝ P( (C^o)_(t,1) | (π^o)_1 ) P( (w^o)_(t,1) | (C^o)_(t,1), θ^(o,w))
                ## P( (C^o)_(t,1) | (π^o)_1 ) (カテゴリ分布)
                objct_topic_num = 3
                object_topic = np.arange(objct_topic_num)
                co_t_1_k = np.identity(objct_topic_num)
                cat_co_v1 = multinomial.pmf(x = co_t_1_k, n = 1, p = pi_o)                                          #物体トピックの確率分布
                co_idx_pre = stats.rv_discrete(name='co_idx_pre', values=(object_topic, cat_co_v1)).rvs(size=1)     #物体のトピック(事前)を一つサンプリング
                co_idx_pre = co_idx_pre[0]

                ## P( (w^o)_(t,1) | (C^o)_(t,1), θ^(o,w)) (カテゴリ分布) 
                #object_dic = 71
                #wo_t_1_k = np.identity(object_dic)
                theta_ow_data = np.loadtxt(MLDA_PARAM_PATH + 'learn_result/Pdw[1].txt')
                with open(MLDA_PARAM_PATH  + 'processing_data/bow/learn/word_dic.txt', 'r') as f:
                    obj_name_list = f.read().split("\n")
                cat_wo = theta_ow_data[obj_name_list.index(target_name[name]), co_idx_pre]
                co = cat_co_v1 * cat_wo
                co = [float(i)/sum(co) for i in co]                                                                  #正規化
                co_idx_pos = stats.rv_discrete(name='co_idx_pos', values=(object_topic, co)).rvs(size=1)
                co_idx_pos = co_idx_pos[0]



                # (C^s)_tのサンプリング
                # P((C^s)_t | (C^o)_(t,1), ξ, π^s) ∝ P((C^s)_t | π^s) P((C^o)_(t,1) | (C^s)_t, ξ)
                ## P((C^s)_t | π^s) (カテゴリ分布)
                with open(SPCO_PARAM_PATH + 'pi.csv', 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        pass
                pi_s_data = row
                del pi_s_data[-1]
                pi_s = np.array(pi_s_data, dtype = np.float64)
                place_topic_num = len(pi_s_data)
                place_topic = np.arange(place_topic_num)                              
                cs_k = np.identity(place_topic_num)
                cat_cs = multinomial.pmf(x = cs_k, n = 1, p = pi_s)
                cs_idx_pre = stats.rv_discrete(name='cs_idx_pre', values=(place_topic, cat_cs)).rvs(size=1)
                cs_idx_pre = cs_idx_pre[0]

                ## P((C^o)_(t,1) | (C^s)_t, ξ) (カテゴリ分布)
                #place_dic = 4
                #co_t_1_k_v2 = np.identity(place_dic)
                xi = []
                with open(SPCO_PARAM_PATH + 'Xi.csv') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        del row[-1]
                        xi.append(np.array(row, dtype = np.float64))
                        #count += 1
                #print(xi[0][0])                                                     
                cat_co_v2 = xi[cs_idx_pre][co_idx_pos] 
                #print(cat_co_v2)
                cs = cat_cs * cat_co_v2
                cs = [float(i)/sum(cs) for i in cs]
                #print(cs)  
                cs_idx_pos = stats.rv_discrete(name='cs_idx_pos', values=(place_topic, cs)).rvs(size=1)
                cs_idx_pos = cs_idx_pos[0]



                # (w^s)_tのサンプリング
                # P((w^s)_t | (C^s)_t, theta_sw) (カテゴリ分布)
                place_dic = len(place_name_list)
                ws_t_k = np.identity(place_dic)
                theta_sw = []
                with open(SPCO_PARAM_PATH + 'W.csv') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        del row[-1]
                        theta_sw.append(np.array(row, dtype = np.float64))
                cat_ws = multinomial.pmf(x = ws_t_k, n = 1, p = theta_sw[cs_idx_pos])
                cat_ws_list.append(cat_ws)
            
            sub_result = (sum(cat_ws_list)) / samples # 10点分の平均を取る
            result.append(sub_result)
            cat_ws_list = []
        final_result = (sum(result)) / len(target_name) # pig_doll, pig, doll分の確率を平均化

        print(place_name_list)
        print(final_result)
        return final_result

if __name__ == "__main__":
    #rospy.init_node('ancestral_sampling_object2place')
    object2place = AncestralSamplingObject2Place()
    object2place.word_callback()
    #rospy.spin()

