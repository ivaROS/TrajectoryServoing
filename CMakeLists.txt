cmake_minimum_required(VERSION 2.8.3)
project(trajectory_servoing)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  navfn
  tf
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
  cv_bridge
  sensor_msgs
  std_msgs
  pips_trajectory_msgs
  image_transport
  message_filters
  message_generation
  image_geometry
  dynamic_reconfigure
  turtlebot_trajectory_functions
  sparse_stereo
  sparse_stereo_msgs
)

find_package(Boost    REQUIRED COMPONENTS thread system filesystem)
find_package(OpenCV 3 REQUIRED)
find_package(OpenMP)
# find_package(Eigen3   REQUIRED)


MESSAGE(STATUS "opencv_version='${OpenCV_VERSION}'")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -std=c++11 ${OpenMP_CXX_FLAGS} -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")

# srvs
add_service_files(
  DIRECTORY srv
  FILES OnlineTrajService.srv
)

generate_messages(
  DEPENDENCIES
    pips_trajectory_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cubic_spline_interpolator trajectory_servo_core
  CATKIN_DEPENDS dynamic_reconfigure tf2_ros turtlebot_trajectory_functions pips_trajectory_msgs sparse_stereo sparse_stereo_msgs message_runtime #Note: the generic parts of the last 2 should probably be moved to a separate package
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../stereoFeatureTracking/stvo/include/
  ${CMAKE_CURRENT_LIST_DIR}/../stereoFeatureTracking/stvo/3rdparty/DBoW2/DBoW2/
  ${CMAKE_CURRENT_LIST_DIR}/../stereoFeatureTracking/stvo/3rdparty/line_descriptor/include/
)
#message(${catkin_INCLUDE_DIRS})
message(${CMAKE_CURRENT_LIST_DIR})

## Declare a C++ executable
add_library(trajectory_servo_core 
            src/trajectory_servo_core.cpp
            src/trajectory_servo_feedforward.cpp)
add_dependencies(trajectory_servo_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(trajectory_servo_no_tracking
            src/trajectory_servo_no_tracking.cpp)
add_dependencies(trajectory_servo_no_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(trajectory_servo_navigator src/trajectory_servo_navigator.cpp)
add_dependencies(trajectory_servo_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(trajectory_servo_no_tracking_navigator src/trajectory_servo_no_tracking_navigator.cpp)
add_dependencies(trajectory_servo_no_tracking_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(cubic_spline_interpolator src/cubic_spline_interpolator.cpp)
add_dependencies(cubic_spline_interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_servo_navigator_node src/trajectory_servo_navigator_node.cpp)

add_executable(trajectory_servo_no_tracking_navigator_node src/trajectory_servo_no_tracking_navigator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cubic_spline_interpolator
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_core
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_no_tracking
  trajectory_servo_core
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_navigator
  trajectory_servo_core
  cubic_spline_interpolator
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_no_tracking_navigator
  trajectory_servo_core
  trajectory_servo_no_tracking
  trajectory_servo_navigator
  cubic_spline_interpolator
  ${CMAKE_CURRENT_LIST_DIR}/../stereoFeatureTracking/stvo/lib/libstvo.so
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_navigator_node
  trajectory_servo_navigator
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_servo_no_tracking_navigator_node
  trajectory_servo_no_tracking_navigator
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(send_online_trajectory
  src/send_online_trajectory.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(send_online_trajectory
  cubic_spline_interpolator
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(gt_odom_generator
  src/gt_odom_generator.cpp)

target_link_libraries(gt_odom_generator
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(gt_odom_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
