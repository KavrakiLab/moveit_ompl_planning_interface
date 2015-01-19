cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planning_interface) # Added "planning" to avoid name conflict with MoveIt library of the same name.  Change this!

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MOVEIT_LIB_NAME moveit_ompl_planning_interface)

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  dynamic_reconfigure
  )

find_package(OMPL REQUIRED)

generate_dynamic_reconfigure_options("cfg/OMPLDynamicReconfigure.cfg")

catkin_package(
  LIBRARIES
    ${MOVEIT_LIB_NAME}
    ${OMPL_LIBRARIES}
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_planners_ompl
    moveit_core
    moveit_ros_planning
    dynamic_reconfigure
)

include_directories(include
                    ${OMPL_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    SYSTEM)

link_directories(${OMPL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


# Create the PlannerManager Plugin
add_library(${MOVEIT_LIB_NAME}
            src/OMPLPlannerManager.cpp
            src/GeometricPlanningContext.cpp
            src/ModelBasedStateSpace.cpp
            src/JointModelStateSpace.cpp
            src/StateValidityChecker.cpp
            src/ProjectionEvaluators.cpp
            src/ConstrainedGoalSampler.cpp
            src/GoalMux.cpp
            src/OMPLConsole.cpp
            src/PoseModelStateSpace.cpp
            src/ConstrainedSampler.cpp)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# make sure configure headers are built before the library tries to compile them
add_dependencies(${MOVEIT_LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install (TARGETS ${MOVEIT_LIB_NAME}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(FILES ompl_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
