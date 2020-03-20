cmake_minimum_required(VERSION 2.8.3)
project(ps_slam)

set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_filters
  tf_conversions
)

#ORB_SLAM的路径
#set(ORB_SLAM_PATH /home/chen/ros/ORB_SLAM2)
set(ORB_SLAM_PATH /home/chen/slam2/ORB_SLAM2)

message("ORB_SLAM_PATH = ${ORB_SLAM_PATH} ")

LIST(APPEND CMAKE_MODULE_PATH ${ORB_SLAM_PATH}/cmake_modules)

find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package( PCL REQUIRED)#COMPONENTS )common io visualization filters)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serve_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	
	${OpenCV_INCLUDE_DIRS}
	${ORB_SLAM_PATH}
	${ORB_SLAM_PATH}/include
	${Pangolin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )


set(LIBS 
	${OpenCV_LIBS} 
#	${PCL_COMMON_LIBRARIES} 
#	${PCL_IO_LIBRARIES} 
	${PCL_LIBRARIES}
	${EIGEN3_LIBS}
	${Pangolin_LIBRARIES}
	${ORB_SLAM_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
	${ORB_SLAM_PATH}/Thirdparty/g2o/lib/libg2o.so
	${ORB_SLAM_PATH}/lib/libORB_SLAM2.so
	 -lboost_system
)

set(LIBS2 
	${OpenCV_LIBS} 
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
)




add_executable(orb_slam_node src/orb_slam_run.cpp)
target_link_libraries(orb_slam_node ${catkin_LIBRARIES} ${LIBS})

add_executable(mapper_node src/mapper.cpp)
target_link_libraries(mapper_node ${catkin_LIBRARIES} ${LIBS})

add_executable(mapper_test src/mapper_test.cpp)
target_link_libraries(mapper_test ${catkin_LIBRARIES} ${LIBS})


