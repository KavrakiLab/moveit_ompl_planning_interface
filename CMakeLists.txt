cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planning_interface)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  ompl
  moveit_core
  moveit_ros_planning
  moveit_msgs
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  eigen_conversions
  )

generate_dynamic_reconfigure_options("ompl_interface/cfg/OMPLDynamicReconfigure.cfg")

catkin_package(
  LIBRARIES
    moveit_ompl_planning_interface
  INCLUDE_DIRS
    ompl_interface/include
  CATKIN_DEPENDS
    ompl
    moveit_core
    moveit_ros_planning
    moveit_msgs
    roscpp
    rosconsole
    pluginlib
    tf
    dynamic_reconfigure
    eigen_conversions
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(ompl_interface)

install(FILES moveit_ompl_interface_plugin_description.xml ompl_geometric_planning_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
