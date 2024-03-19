cmake_minimum_required(VERSION 2.8.3)
project(dynamic)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  pcl_conversions
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

find_package(PCL REQUIRED QUIET)

add_message_files(
	DIRECTORY msg
	FILES
	information.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  message_runtime
  message_generation
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

# preprocessing
add_executable(${PROJECT_NAME}_preProcessing src/preProcessing.cpp)
add_dependencies(${PROJECT_NAME}_preProcessing ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_preProcessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# dynamic feature
add_executable(${PROJECT_NAME}_dynamicFeature src/dynamicFeature.cpp)
add_dependencies(${PROJECT_NAME}_dynamicFeature ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_dynamicFeature ${catkin_LIBRARIES} ${PCL_LIBRARIES})
