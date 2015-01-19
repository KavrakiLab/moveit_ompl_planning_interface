The moveit_ompl_interface supersedes the existing ompl_interface plugin that is packaged with moveit.  The moveit_ompl_interface interface provides a plugin framework to easily incorporate new OMPL-based planners into the framework without having to recompile the ompl_interface.

-- USAGE --
The new interface works with the existing ompl_planning.yaml files that are output with the MoveIt setup wizard.  You can simply extend the yaml planner specification to specify the plugin where the PlannerManager can create the planning context for the given planner.

Example (ompl_planning.yaml):

planner_configs:
  RRTkConfigDefault:
    plugin: moveit_ompl_interface/GeometricPlanningContext  # <-- This line was added
    type: geometric::RRT
    range: 0.0
    goal_bias: 0.05
  RRTConnectkConfigDefault:
    plugin: moveit_ompl_interface/GeometricPlanningContext  # <-- This line was added
    type: geometric::RRTConnect
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
...

To load the plugin, you will need to modify move_group.launch to specify the rice_moveit_ompl_interface planning pipeline instead of the existing ompl planning pipeline.

-- Design --
The interface is plugin centric, where the planning context is responsible for the majority of its own configuration.  New plugins must implement an OMPLPlanningContext class.  For standard geometric planning, it is possible to derive from the existing GeometricPlanningContext class and simply configure your new planner.
