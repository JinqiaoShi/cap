cmake_minimum_required(VERSION 2.8.4)
project(dji_vision_detection)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  cv_bridge
eigen_conversions
)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)

)



set(SOURCE_FILES_ROS src/main.cpp src/image_converter.cpp src/orbMatching.cpp src/detectSquares.cpp src/image_converter.cpp)# Entropy.h Entropy.cpp)
#set(SOURCE_FILES main.cpp detectSquares.cpp src/orbMatching.cpp)# Entropy.h Entropy.cpp)
add_executable(DJI_Vision_Detection_ros ${SOURCE_FILES_ROS})
target_link_libraries( DJI_Vision_Detection_ros ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_executable(grid_Detection src/grid.cpp src/image_converter.cpp src/detectSquares.cpp)
target_link_libraries(grid_Detection ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(toy_Detection src/surfMatching.cpp src/image_converter.cpp)
target_link_libraries(toy_Detection ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(q3_Detection src/main1.cpp src/image_converter.cpp src/orbMatching.cpp src/detectSquares.cpp src/image_converter.cpp)
target_link_libraries(q3_Detection ${OpenCV_LIBS} ${catkin_LIBRARIES})

