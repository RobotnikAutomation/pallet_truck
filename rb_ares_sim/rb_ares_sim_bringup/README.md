# rb_ares_sim_bringup

Launch and config files that launch the complete simulation of the robot, being simulator independent.
Launch files are splitted so one can run the simulation environment, a robot and rviz independently.

This configuration allows to run one or several robots, in one or several simulator environments (either different instances of the same simulator, or instances of different simulators) and an easy switching from simulation to real robot.

It has to be remembered the concept of running and spawning a robot. Spawning if the process of creating a robot in the simulation environment, but not running it's low-level software (i.e. control). Running a robot consist of spawning a robot and executing its low-level software (i.e. control).

It has to be noted that this launch package does not launch any configuration for localization, navigation, perception, etc. as this has to be done at a higher level so integration with real robots is easier.

This README file is splitted in the following sections:

1. Rationale, explains the meaning of each launch file.
1. Launch files explained, details the arguments of each launch file.
1. Environment variables, lists which environtment variables are used.
1. Examples, shows how to use this package.
1. How to extend, gives the rationale of how this package and related packages should be extended.
1. TODO, future work, with a brief explanation of what and why is needed and a hint for the implementation.

## Rationale

There are four launch files. They are explained from high-level to low-level.

1. rb_ares_complete.launch
1. run_simulation.launch
1. run_robot.launch
1. run_rviz.launch

### rb_ares_complete.launch

Single file that can run the simulation environment, a single robot and an RViz instance to visualize that robot. What is runned is controlled through arguments, and uses the launch files described in this list.

### run_simulation.launch

runs the simulation environment.

### run_robot.launch

Runs a robot in an already running simulation environment. That means it spawns a robot and runs it's low-level software.

It can also run an RViz instance connected to this robot.

### run_rviz.launch

Runs an RViz instance to visualize an already running robot.

## Launch files explained.

### Common arguments

These arguments are common to all launch files in this package.

` simulation_package, default="$(optenv SIMULATION_PACKAGE rb_ares_gazebo)" `

Defines which package has the launch files for the simulation. Possibilities:

1. rb_ares_gazebo, has a configuration for Gazebo simulator
1. rb_ares_stage, has a configuration for Stage simulator.

` config_package, default="$(optenv SIMULATION_CONFIG_PACKAGE rb_ares_gazebo)" `

Defines which package has the launch configuration files for the simulation. Possibilities:

1. rb_ares_gazebo, has standard configuration for Gazebo simulator.
1. rb_ares_gazebo_fancy, has the configuration for Gazebo simulator related to the Fancy project.

` use_gpu_for_simulation, default="$(optenv SIMULATION_USE_GPU true)" `

Defines if simulation use GPU accelerated plugins or not. If true, simulation should run faster but with problems sometimes. At the time of this writing this only affects the XACRO/URDF, which is the one that specifies which plugins are loaded (so you can say if they are GPU based or not).

### rb_ares_complete.launch

This launch file receives and forwards the arguments for the low-level launch files, which are described later, as well as the following:

* ` run_robot, default="true" `

If robot should be runed

* ` run_simulation, default="true" `

If simulation environment should be runed

* ` run_rviz, default="true" `

If RViz should be runed

### run_simulation.launch

This launch file received arguments to set up the simulation environment.

* ` world, default="worlds/rb_ares_epal.world" `

World file to be loaded, as relative path to config_package argument

* ` world_file, default="$(eval find(config_package) + '/' + world)"/>" `

World file to be loaded, as an absolute path in the file system, used to override `world` argument

* ` debug, default="false" `

If simulator should output debug information

* ` paused, default="false" `

If simulator should start paused

* ` headless, default="false" `

If simulator should start in headless mode

* ` gui, default="true" `

If simulator should show its GUI.

### run_robot.launch

* ` robot_id, default="$(optenv ROBOT_ID robot)"  `

Robot id.

* ` prefix, default="$(arg robot_id)_"  `

Prefix of runed robot.

* ` robot_model, default="$(optenv ROBOT_MODEL rb_ares)"  `

If different models should exist (rb_ares, rb_ares_steel...)

* ` robot_xacro, default="$(optenv ROBOT_XACRO rb_ares_std.urdf.xacro)"  `

Specifies XACRO to be loaded as robot_description

* ` x_init_pose, default="0"  `

Initial X pose in world

* ` y_init_pose, default="0"  `

Initial Y pose in world

* ` z_init_pose, default="0"  `

Initial Z pose in world

* ` launch_base_hw_sim, default="false"  `

If base_hw simulator should be loaded

* ` launch_battery_estimator, default="false"  `

If battery estimator should be loaded

* ` run_rviz", default="false" `

If RViz should be runed for this robot. See run_rviz section on the parameters

### run_rviz.launch

* ` rviz_config", default="rviz/rviz.rviz" `

RViz config file to be loaded, as relative path to config_package argument

* ` rviz_config_file, default="$(eval find(config_package) + '/' + rviz_config) `

RViz config file to be loaded, as an absolute path in the file system, used to override `rviz_config` argument

## Environment variables

Lists which environtment variables are used by this configuration package.

* `SIMULATION_PACKAGE`, which package has the basic launch files for each simulator
* `SIMULATION_CONFIG_PACKAGE`, specfies the package with the configuration to be used, normally would have the same value as `SIMULATION_PACKAGE`.
* `SIMULATION_USE_GPU`, specifies if simulation use GPU accelerated plugins or not. If true, simulation should run faster but with problems sometimes.

* `ROBOT_ID`, id of the robot to be runed.
* `ROBOT_MODEL`, robot model in case several exist (e.g. Pallet Truck, Pallet Truck Steel).
* `ROBOT_XACRO`, XACRO robot definition.

## Examples

#### Launch standard simulation with one robot and its visualization:

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch`

#### Launch standard simulation with one robot and its visualization and run another one called "robot_2" in start pose (5, 0, 0) without visualization

First:

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch`

Then:

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch robot_id:=robot_2 x_init_pose:=5 run_simulation:=false run_rviz:=false`

or:

* `roslaunch rb_ares_sim_bringup run_robot.launch robot_id:=robot_2 x_init_pose:=5`

#### Launch simulation environment and two robots independently, with rviz for only one robot:

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch run_robot:=false run_rviz:=false`

Then:

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch run_simulation:=false`

* `roslaunch rb_ares_sim_bringup rb_ares_complete.launch robot_id:=robot_2 x_init_pose:=5 run_simulation:=false`

or:

* `roslaunch rb_ares_sim_bringup run_robot.launch robot_id:=robot x_init_pose:=0 run_rviz:=true`

* `roslaunch rb_ares_sim_bringup run_robot.launch robot_id:=robot_2 x_init_pose:=5 run_rviz:=false`

## How to extend

The most important thing is to maintain compatibility and reuse existing structures.

### New simulator

If a new simulator is used (VREP, Stage, etc), a `rb_ares_SIMULATOR` package must be created with at least two low-level launch files:

1. spawn_simulation.launch, which will load the simulation environment.
2. spawn_robot.launch, which will spawn a robot into that simulation environemt.

Remember that this package should only spawn a low-level robot, without running it's software (i.e. control), which is done at a higher-level, in this case, in the `rb_ares_sim_bringup/run_robot.launch`.

Arguments for simulation should match and be remapped to existing arguments. For example, if new simulator receives world file through an argument called `environment`, then `spawn_simulation.launch` should receive it through the already existing argument `world`, but internally call the simulator with the `environment` argument.

Update README files accordingly.

Current existing arguments are derived from Gazebo, because it is the simulator we use now.

### New configuration

If an specific configuration for a project is required, then a `rb_ares_SIMULATOR_PROJECT` package must be created with the required simulation files, such as `world, models, etc.` folders. Launch files are not required.

Update README files accordingly.

## TODO

1. ~~Resolve some inconsistency between ROBOT_XX (optenv ROBOT_MODEL, translated to argument robot_model) and XX_ROBOT (optenv ROBOT_ID, translated to argument id_robot)~~
1. RViz configuration for different robots. Difficult to have due to frame names, topics and parameters are stored in the configuration. Likely a WON'T DO.
