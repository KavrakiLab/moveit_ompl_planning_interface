The moveit\_ompl\_planning\_interface supersedes the existing ompl\_interface
plugin that is packaged with moveit.  The moveit\_ompl\_planning\_interface
interface provides a plugin framework to easily incorporate new OMPL-based
planners into the framework without having to recompile the ompl\_interface.

## USAGE
The new interface works with the existing `ompl_planning.yaml` files that are
output with the MoveIt setup wizard.  You can simply extend the yaml planner
specification to specify the plugin where the PlannerManager can create the
planning context for the given planner.

Example (`ompl_planning.yaml`):
```
planner_configs:
  RRTkConfigDefault:
    plugin: ompl_interface/GeometricPlanningContext  # <-- This line was added.  This is the fully-qualified name of the plugin you create
    type: geometric::RRT
    range: 0.0
    goal_bias: 0.05
  RRTConnectkConfigDefault:
    plugin: ompl_interface/GeometricPlanningContext  # <-- This line was added.  This is the fully-qualified name of the plugin you create
    type: geometric::RRTConnect
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
...
```

To load the plugin, you will need to modify `move_group.launch` to specify the
moveit\_ompl\_planning\_interface pipeline instead of the existing ompl planning
pipeline. Usually, this code is factored out into a
`planning_pipeline.launch.xml` or a `ompl_planning_pipeline.launch.xml` file
instead. In `ompl_planning_pipeline.launch.xml`, make sure the following exists.
```
<!-- OMPL Plugin for MoveIt! -->
<arg name="planning_plugin" value="ompl_interface/OMPLPlanningContextManager" />
```

## Design
The interface is plugin centric, where the planning context is responsible for
the majority of its own configuration.  New plugins must implement an
`OMPLPlanningContext` class.  For standard geometric planning, it is possible to
derive from the existing `GeometricPlanningContext` class and simply configure
your new planner.

If you do add a new plugin, you must make a `$NAME_plugin_description.xml` and
add to the `package.xml`
```
<moveit_ompl_planning_interface plugin="${prefix}/$NAME_plugin_description.xml</moveit_ompl_planning_interface>
```
