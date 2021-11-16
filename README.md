# spco2_mlda
This repository is SpCoSLAM-MLDA package.  
SpCoSLAM-MLDA learns the relationship between object and place.  
Mainly, it is a program that executes [SpCoSLAM](https://github.com/a-taniguchi/SpCoSLAM2) in Gazebo.

*   Maintainer: Shoichi Hasegawa ([hasegawa.shoichi@em.ci.ritsumei.ac.jp](mailto:hasegawa.shoichi@em.ci.ritsumei.ac.jp)).
*   Author: Shoichi Hasegawa ([hasegawa.shoichi@em.ci.ritsumei.ac.jp](mailto:hasegawa.shoichi@em.ci.ritsumei.ac.jp)).

You can do it like this below image.  
(You need to prepare room layouts personally.)
![hsr-noetic](https://user-images.githubusercontent.com/74911522/137430543-1d35d631-963c-446e-ac13-560b64926d47.png)


## Content
* [Execution Environment](#execution-environment)
* [Execute Procedure](#execute-procedure)
* [Folder](#folder)
* [To Do](#to-do)
* [Reference](#reference)
* [Acknowledgements](#acknowledgements)


## Execution environment  
- Ubuntu：20.04LTS
- ROS：Noetic
- Python：3.8.10
- C++：14
- Robot：Human Support Robot (HSR)


## Execution Procedure
1  `cd HSR/catkin_ws/src`  
2. `git clone https://github.com/Shoichi-Hasegawa0628/spco2_mlda.git`  
3. `cd spco2_mlda/mlda`  
4. `git submodule update --init --recursive`  
5. `cd ~/HSR/ && bash ./RUN-DOCKER-CONTAINER.bash`  
6. `cd catkin_ws`  
7. `catkin_make`  
8. Execute [MLDA](https://github.com/Shoichi-Hasegawa0628/mlda/tree/devel)  
9. Launch Gazebo  
10. Launch Rviz  
11. `rosnode kill /pose_integrator`   
12. `roscd rgiro_spco2_slam`  
13. `cd bash`  
14. `bash reset-spco2-slam-output.bash`  
15. `roslaunch rgiro_spco2_slam spco2_slam.launch`  
16. `roslaunch rgiro_spco2_slam spco2_word.launch`  

Teaching the place name while teleoping with `rqt`.  
If you want to use cross modal inference which infer from object word to place word,  
you can use `ancestral_sampling_object2place.py`.


## Folder
- `mlda`：Object categorization method (MLDA (Multimodal LDA))
- `rgiro_openslam_gmapping`：SpCoSLAM Wrapper of FastSLAM2.0 published on [OpenSLAM](https://openslam-org.github.io/)
- `rgiro_slam_gmapping`：SpCoSLAM Wrapper of slam_gmapping ros package (ros wrapper of openslam_gmapping)
- `rgiro_spco2_slam`：Main codes of SpCoSLAM
- `rgiro_spco2_visualization`：Visualization codes of learning spatial concepts
- `spco2_mlda`：


## To Do (Japanese)
- パラメータの追加学習には未対応
- 絶対パスが含まれている箇所があるため、変更が必要な場合あり


## Reference
- [SpCoSLAM 2.0](https://github.com/a-taniguchi/SpCoSLAM2)
- [MLDA](https://github.com/Shoichi-Hasegawa0628/mlda/tree/devel)
