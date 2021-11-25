cmake_minimum_required(VERSION 3.0.2)
project(spco2_mlda)

find_package(catkin REQUIRED COMPONENTS
   sensor_msgs
   message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SendImageYOLO.srv
 )

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
 )

