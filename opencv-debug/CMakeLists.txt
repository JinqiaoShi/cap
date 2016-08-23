cmake_minimum_required(VERSION 2.8)
project( OCVE )

# Add the EGBIS algortihm
# add_subdirectory(egbis)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(

  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)

)


find_package( OpenCV REQUIRED )

add_executable( main main.cpp )
#add_executable( main main.cpp)
target_link_libraries( main ${OpenCV_LIBS})

add_executable( main2 main2.cpp )
#add_executable( main main.cpp)
target_link_libraries( main2 ${OpenCV_LIBS})

add_executable( main3 main3.cpp )
#add_executable( main main.cpp)
target_link_libraries( main3 ${OpenCV_LIBS})

add_executable( opencv_cam opencv_cam.cpp )
#add_executable( main main.cpp)
target_link_libraries( opencv_cam ${OpenCV_LIBS})
