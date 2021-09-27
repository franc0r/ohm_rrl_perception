cmake_minimum_required(VERSION 3.0.2)
project(ohm_rrl_motiondetection2)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3 REQUIRED)    
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp -Wall -O2 -std=c++11")

 generate_dynamic_reconfigure_options(
   cfg/MotionDetection.cfg
)


catkin_package(
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ohm_motiondetection_node src/MotionDetectionNode.cpp)


 add_dependencies(ohm_motiondetection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  )


 target_link_libraries(ohm_motiondetection_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )


 install(TARGETS ohm_motiondetection_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


 install(FILES
   launch/motiondetection.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

