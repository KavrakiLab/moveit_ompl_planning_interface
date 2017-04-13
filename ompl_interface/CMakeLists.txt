set(MOVEIT_LIB_NAME moveit_ompl_planning_interface) # added "planning" to avoid conflict with existing MoveIt library

add_library(${MOVEIT_LIB_NAME}
  src/ompl_planning_context_manager.cpp
  src/geometric_planning_context.cpp
  src/parameterization/model_based_state_space.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constrained_goal_sampler.cpp
  src/detail/ompl_console.cpp
  src/detail/threadsafe_state_storage.cpp
)

#find_package(OpenMP)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

#catkin_add_gtest(test_state_space test/test_state_space.cpp)
#target_link_libraries(test_state_space ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

#add_executable(moveit_ompl_planner src/ompl_planner.cpp)
#target_link_libraries(moveit_ompl_planner ${MOVEIT_LIB_NAME})
#set_target_properties(moveit_ompl_planner PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

#add_executable(moveit_demo_construct_constraints_database src/demo_construct_state_database.cpp)
#target_link_libraries(moveit_demo_construct_constraints_database ${MOVEIT_LIB_NAME})
#set_target_properties(moveit_demo_construct_constraints_database PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

#add_library(moveit_ompl_planner_plugin src/ompl_planner_manager.cpp)
#target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner moveit_ompl_planner_plugin
install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
