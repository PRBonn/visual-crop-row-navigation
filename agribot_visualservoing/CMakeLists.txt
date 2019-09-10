cmake_minimum_required(VERSION 2.8.3)
project(agribot_visualservoing)

# c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  cmake_modules 
  message_generation
  dynamic_reconfigure 
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  move_base
  robot_localization
  
)

find_package( PkgConfig )

find_package( OpenCV REQUIRED )

add_message_files(FILES vs_msg.msg)

generate_messages(DEPENDENCIES std_msgs)

set (CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
    message_runtime
#  DEPENDS
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
   src/agribot_vs.cpp
)

## Declare cpp executables
add_executable(agribot_vs_node
  src/agribot_vs_node.cpp
  src/agribot_vs_nodehandler.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(agribot_vs_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(agribot_vs_node 
	${OpenCV_LIBS} 
	Eigen3::Eigen
)

add_dependencies(agribot_vs_node ${PROJECT_NAME}_gencfg)
add_dependencies(agribot_vs_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS})


#add_subdirectory("src")

