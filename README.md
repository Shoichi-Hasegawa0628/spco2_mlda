# spco2_mlda
This repository is spatial concepts library.  
Mainly, it is a program that executes [SpCoSLAM](https://github.com/a-taniguchi/SpCoSLAM2) in the gazebo environment.

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


## Execution Procedure
1. Launch Gazebo
2. Launch Rviz
3. `rosnode kill /pose_integrator` 
4. `roscd rgiro_spco2_slam`
5. `cd bash`
6. `bash reset-spco2-slam-output.bash`
7. `roslaunch rgiro_spco2_slam spco2_slam.launch`
8. `roslaunch rgiro_spco2_slam spco2_word.launch`

Teaching the place name while teleoping with rqt.


## Folder  
- `rgiro_openslam_gmapping`：SpCoSLAM Wrapper of FastSLAM2.0 published on [OpenSLAM](https://openslam-org.github.io/)
- `rgiro_slam_gmapping`：SpCoSLAM Wrapper of slam_gmapping ros package (ros wrapper of openslam_gmapping)
- `rgiro_spco2_slam`：Main codes of SpCoSLAM
- `rgiro_spco2_visualization`：Visualization codes of learning spatial concepts

## To Do (Japanese)
- slam_gmappingとopenslam_gmappingの改変部分を外部のプログラムで構築
- 絶対パスを無くす
- 要らないコメントアウトなどを消す
- ハイパーパラメータなどはyamlファイルへ

## Reference
