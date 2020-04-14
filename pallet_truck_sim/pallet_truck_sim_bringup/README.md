# pallet_truck_sim_bringup

Launch and config files that launch the complete simulation of the robot, being simulator independent.
Launch files are splitted so one can spawn the simulation environment, a robot and rviz independently.

This configuration allows to spawn one or several robots, in one or several simulator environments (either different instances of the same simulator, or instances of different simulators) and an easy switching from simulation to real robot.

It has to be noted that this launch package does not launch any configuration for localization, navigation, perception, etc. as this has to be done at a higher level so integration with real robots is easier.

This README file is splitted in the following sections:
1. Rationale, explains the meaning of each launch file.
1. Launch files explained, details the arguments of each launch file.
1. Examples, shows how to use this package.
1. How to extend, gives the rationale of how this package and related packages should be extended.
1. TODO, future work, with a brief explanation of what and why is needed and a hint for the implementation.

## Rationale

There are four launch files. They are explained from high-level to low-level.

1. pallet_truck_complete.launch
1. spawn_simulation.launch
1. spawn_robot.launch
1. spawn_rviz.launch

### pallet_truck_complete.launch

Single file that can spawn the simulation environment, a single robot and an RViz instance to visualize that robot. What is spawned is controlled through arguments, and uses the launch files described in this list.

### spawn_simulation.launch

Spawns the simulation environment. 

### spawn_robot.launch

Spawns a robot in an already running simulation environment. This file spawns the robot related software, as well as spawns an instance of the robot in the simulator. It can also spawn an RViz instance connected to this robot.

### spawn_rviz.launch

Spawns an RViz instance to visualize an already spawned robot.

## Launch files explained.

### Common arguments

These arguments are common to all launch files in this package.

``` simulation_package, default="$(optenv SIMULATION_PACKAGE pallet_truck_gazebo)" ```

Defines which package has the launch files for the simulation. Possibilities:

1. pallet_truck_gazebo, has a configuration for Gazebo simulator
1. pallet_truck_stage, has a configuration for Stage simulator.

``` config_package, default="$(optenv SIMULATION_CONFIG_PACKAGE pallet_truck_gazebo)" ```

Defines which package has the launch configuration files for the simulation. Possibilities:
    
1. pallet_truck_gazebo, has standard configuration for Gazebo simulator.
1. pallet_truck_gazebo_fancy, has the configuration for Gazebo simulator related to the Fancy project.

### pallet_truck_complete.launch

This launch file receives and forwards the arguments for the low-level launch files, which are described later, as well as the following:

* ``` spawn_robot, default="true" ```

If robot should be spawned

* ``` spawn_simulation, default="true" ```

If simulation environment should be spawned

* ``` spawn_rviz, default="true" ```

If RViz should be spawned

### spawn_simulation.launch

This launch file received arguments to set up the simulation environment.

* ``` world, default="worlds/pallet_truck_epal.world" ```

World file to be loaded, as relative path to config_package argument 

* ``` world_file, default="$(eval find(config_package) + '/' + world)"/>" ```

World file to be loaded, as an absolute path in the file system, used to override `world` argument

* ``` debug, default="false" ```

If simulator should output debug information

* ``` paused, default="false" ```

If simulator should start paused

* ``` headless, default="false" ```

If simulator should start in headless mode

* ``` gui, default="true" ```

If simulator should show its GUI.

### spawn_robot.launch

* ``` id_robot, default="$(optenv ROBOT_ID robot)"  ```

Robot id.

* ``` prefix, default="$(arg id_robot)_"  ```

Prefix of spawned robot.

* ``` robot_model, default="$(optenv ROBOT_MODEL pallet_truck)"  ```

If different models should exist (pallet_truck, pallet_truck_steel...)

* ``` xacro_robot, default="$(optenv ROBOT_XACRO pallet_truck_std.urdf.xacro)"  ```

Specifies XACRO to be loaded as robot_description

* ``` x_init_pose, default="0"  ```

Initial X pose in world 

* ``` y_init_pose, default="0"  ```

Initial Y pose in world 

* ``` z_init_pose, default="0"  ```

Initial Z pose in world 

* ``` launch_base_hw_sim, default="false"  ```

If base_hw simulator should be loaded

* ``` launch_battery_estimator, default="false"  ```

If battery estimator should be loaded

* ``` spawn_rviz", default="false" ```

If RViz should be spawned for this robot. See spawn_rviz section on the parameters

### spawn_rviz.launch

* ``` rviz_config", default="rviz/rviz.rviz" ```

RViz config file to be loaded, as relative path to config_package argument 

* ``` rviz_config_file, default="$(eval find(config_package) + '/' + rviz_config) ```

RViz config file to be loaded, as an absolute path in the file system, used to override `rviz_config` argument

## Examples

#### Launch standard simulation with one robot and its visualization:

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch`

#### Launch standard simulation with one robot and its visualization and spawn another one called "robot_2" in start pose (5, 0, 0) without visualization

First:

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch`

Then: 

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch id_robot:=robot_2 x_init_pose:=5 spawn_simulation:=false spawn_rviz:=false`
    
or:
    
* `roslaunch pallet_truck_sim_bringup spawn_robot.launch id_robot:=robot_2 x_init_pose:=5`

#### Launch simulation environment and two robots independently, with rviz for only one robot:

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch spawn_robot:=false spawn_rviz:=false`

Then:

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch spawn_simulation:=false`

* `roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch id_robot:=robot_2 x_init_pose:=5 spawn_simulation:=false`

or:

* `roslaunch pallet_truck_sim_bringup spawn_robot.launch id_robot:=robot x_init_pose:=0 spawn_rviz:=true`

* `roslaunch pallet_truck_sim_bringup spawn_robot.launch id_robot:=robot_2 x_init_pose:=5 spawn_rviz:=false`

## How to extend

The most important thing is to maintain compatibility and reuse existing structures.

### New simulator

If a new simulator is used (VREP, Stage, etc), a `pallet_truck_SIMULATOR` package must be created with at least two launch files:

1. spawn_simulation.launch, which will load the simulation environment.
2. spawn_robot.launch, which will spawn a robot into that simulation environemt.

Arguments for simulation should match and be remapped to existing arguments. For example, if new simulator receives world file through an argument called `environment`, then `spawn_simulation.launch` should receive it through the already existing argument `world`, but internally call the simulator with the `environment` argument.

Current existing arguments are derived from Gazebo, because it is the simulator we use now.

### New configuration

If an specific configuration for a project is required, then a `pallet_truck_SIMULATOR_PROJECT` package must be created with the required simulation files, such as `world, models, etc.` folders. Launch files are not required.

## TODO

RViz configuration for different robots. Difficult to have due to frame names, topics and parameters are stored in the configuration. Likely a WON'T DO.
