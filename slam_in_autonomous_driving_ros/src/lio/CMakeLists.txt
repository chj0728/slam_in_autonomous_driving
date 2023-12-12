cmake_minimum_required(VERSION 3.0.2)
#cmake_minimum_required(VERSION 2.8.3)
project(lio)
#set(CMAKE_BUILD_TYPE "Debug")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  common
  #tf
  tf_conversions
  tf2_ros
  tf2
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  #laser_geometry
)
#Ceres
find_package(Ceres REQUIRED)
# ${CERES_LIBRARIES} 

#Sophus
find_package(Sophus REQUIRED)

#PCL
find_package(PCL REQUIRED)

find_package(yaml-cpp REQUIRED)


#Find Boost LIBRARIES
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS thread )#signals)
set(Boost_INCLUDE_DIRS /usr/include/boost)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lio
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/thirdparty/sophus/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
)

## Declare extra library
# add_library(${PROJECT_NAME}_lib
#         include/kdtree.cpp
#         include/icp_2d.cpp
#         )
# target_link_libraries(${PROJECT_NAME}_lib
#         tbb
#         )

#imu_preintegration_node
#######################
add_executable(imu_preintegration_node src/imu_preintegration_node.cpp)
target_link_libraries(imu_preintegration_node
  ${catkin_LIBRARIES}
  # ${Sophus_LIBRARIES}
)

#imu_pre_node
#######################
add_library(imu_pre_lib
        include/static_imu_init.cpp
        include/imu_pre.cpp
        )
add_executable(imu_pre_node 
src/imu_pre_node.cpp)
target_link_libraries(imu_pre_node
  imu_pre_lib
  glog
  gflags
  ${catkin_LIBRARIES}
)



#kdtree_node
######################
add_executable(kdtree_node 
src/kdtree_node.cpp
include/kdtree.cpp)
target_link_libraries(kdtree_node
  ${catkin_LIBRARIES}
  glog
  gflags
  tbb
  # ${PCL_LIBRARIES}
)



#icp_node
####################
add_executable(icp_node 
src/icp_node.cpp
include/icp_2d.cpp)
target_link_libraries(icp_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES} 
  glog
  gflags
  # ${PCL_LIBRARIES}
)



# GMapping map
######################################
add_executable(gmapping_node
  src/gmapping/gmapping.cc
)

add_dependencies(gmapping_node 
${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gmapping_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



# Occupancy node
######################################
add_executable(occupancy_node
  src/occupancy_node.cpp
  include/occupancy_map.cpp

)
target_link_libraries(occupancy_node
  ${catkin_LIBRARIES}
)



# ceres example
######################################
add_executable(helloworld 
              src/test/helloworld.cc)
target_link_libraries(helloworld PRIVATE
                    ${CERES_LIBRARIES} 
                    glog)

add_executable(helloworld_numeric_diff 
              src/test/helloworld_numeric_diff.cc)
target_link_libraries(helloworld_numeric_diff PRIVATE 
                      ${CERES_LIBRARIES} 
                      glog)

add_executable(helloworld_analytic_diff 
              src/test/helloworld_analytic_diff.cc)
target_link_libraries(helloworld_analytic_diff PRIVATE 
                      ${CERES_LIBRARIES} 
                      glog)



# test node
######################################
# file(GLOB_RECURSE ALL_SRCS "include/lio/sensors/imu.cpp")
# add_executable(imu_node 
#               src/nodes/imu_node.cpp
#               ${ALL_SRCS})
# target_link_libraries(imu_node PRIVATE 
#                       glog
#                       gflags
#                       ${catkin_LIBRARIES})


#将include/lio下的所有*.cpp文件编译成库 lio_lib
file(GLOB_RECURSE SRCS "include/lio/*.cpp")
add_library(lio_lib
            ${SRCS})
target_link_libraries(lio_lib PRIVATE 
                      yaml-cpp
                      glog
                      gflags
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})

#imu_node                       
add_executable(imu_node 
              src/nodes/imu_node.cpp)
target_link_libraries(imu_node  PRIVATE 
                      glog
                      gflags
                      ${catkin_LIBRARIES}
                      lio_lib)
#cloud_node
add_executable(cloud_node
              src/nodes/cloud_node.cpp)
target_link_libraries(cloud_node PRIVATE 
                      glog
                      gflags
                      ${catkin_LIBRARIES}
                      yaml-cpp
                      lio_lib)
message(STATUS "SRCS" ${SRCS} )


# CMAKE related message
######################
message(STATUS "PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR} )
message(STATUS "PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR} )
message(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR} )
message(STATUS "CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR} )
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR} )
message(STATUS "CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR} )
message(STATUS "CMAKE_CURRENT_LIST_FILE: " ${CMAKE_CURRENT_LIST_FILE} )
message(STATUS "CMAKE_CURRENT_LIST_LINE: " ${CMAKE_CURRENT_LIST_LINE} )
message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )
message(STATUS "Sophus_INCLUDE_DIRS: " ${Sophus_INCLUDE_DIRS} )
######################