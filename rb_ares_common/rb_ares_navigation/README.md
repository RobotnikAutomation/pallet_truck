# rb_ares_navigation

This package contains the configuration for running the Navigation stack of the rb_ares robot (ROS Navigation, Robotnik Navigation, etc)

# Structure

- Everything should be stored as parameters in YAML files, only reasonable exceptions are accepted.
- Launch arguments should load the proper YAML config file, not overwrite parameters.
- New launch files can be created for default loading of some YAML config files, only reasonable exceptions are accepted.
- Add new arguments and parameters to main launch file, and just add arguments to derived ones, only reasonable exceptions are accepted.
- Follow similar rules when creating files, folders, parameters and arguments to the ones already existing.
- Use argument _pass_all_args="true"_ with _include_ launch files and pass all existing arguments and avoid verbosity.

# Common arguments

### Robot configuration

* robot_id (string, default: robot)

Name of spawned robot to connect

* prefix (string, default: $(arg robot_id)_)

Prefix of spawned robot to connect

* kinematics (string, default: ROBOT_KINEMATICS diff)

Kinematics of robot.

* has_safety_controller (bool, default: HAS_SAFETY_CONTROLLER false)

If robot has software-based safety controller. If true "_unsafe" is appended to cmd_vel_topic output

### Topics

* cmd_vel_topic (string, default: move_base/cmd_vel)

Topic for command velocity.

* odom_topic (string, default: robotnik_base_control/odom)

Topic to read odometry.

### Frame overwriting (should not be used)

* overwrite_frames (bool, default: false)

If frames from YAML files must be overwritten (which by the way is a bad practice)

* odom_frame (string, default: $(arg prefix)odom)

Odom frame to overwrite (which by the way is a bad practice)

* base_frame (string, default: $(arg prefix)base_footprint)

Base frame to overwrite (which by the way is a bad practice)

* odom_frame_param_name (string, default: "")

Name of parameter for odom_frame that has to be overwritten. Should not be used though.

* base_frame_param_name (string, default: "")

Name of parameter for base_frame that has to be overwritten. Should not be used though.

# move_base

There is one is a basic configuration of move_base, with some standard additional configurations for each planner.

## arguments

### Topics

* front_scan_topic (string, default: $(optenv ROBOT_NAVIGATION_FRONT_2D_SCAN front_laser/scan_filtered))

Topic for front laser scan (if exists)

* rear_scan_topic (string, default: $(optenv ROBOT_NAVIGATION_REAR_2D_SCAN rear_laser/scan_filtered))

Topic for rear laser scan (if exists)

* front_rgbd_to_scan_topic (string, default: $(optenv ROBOT_NAVIGATION_FRONT_RGBD_TO_SCAN front_rgbd_camera/point_cloud_scan_filtered))

Topic for projected pointcloud from rgbd camera (if exists)

### Configuration

* navigation_package (string, default: $(optenv NAVIGATION_PACKAGE rb_ares_navigation)

* config_folder (string, default: $(find navigation_package)/config)

Folder where configuration is stored. Can be used to store different configurations. No need to be in the same package.

* costmap_folder (string, default: costmaps)

* costmap_common_params (string, default: costmap_common_params.yaml)

Default config file for common costmap parameters

* global_costmap_params (string, default: global_costmap_params_nomap.yaml)

Default config file for global costmap parameters

* local_costmap_params (string, default: local_costmap_params.yaml)

Default config file for local costmap parameters

* global_planner (string, default: $(optenv ROBOT_MOVE_BASE_GLOBAL_PLANNER global))

Default global planner (yes, global is a global planner).

* global_planner_params (string, default: $(arg global_planner)_global_planner_params.yaml)

Config file for global planner parameters. Should not be overwritten unless you want load a config file does not follow rules of PLANNERNAME_planner_params.yaml

* local_planner (string, default: $(optenv ROBOT_MOVE_BASE_LOCAL_PLANNER teb))

Default local planner.

* local_planner_params (string, default: $(arg local_planner)_local_planner_$(arg kinematics)_params.yaml)

Config file for local planner parameters. Should not be overwritten unless you want load a config file does not follow rules of PLANNERNAME_planner_KINEMATICS_params.yaml

## dockers

There is one basic configuration of docker algorithms

### Configuration

* navigation_package (string, default: $(optenv NAVIGATION_PACKAGE rb_ares_navigation)

* config_folder (string, default: $(find navigation_package)/config/dockers)

Folder where configuration is stored. Can be used to store different configurations. No need to be in the same package.

* config_file (string, default: $(arg kinematics)_docker.yaml)

Config file for docker. Should not be overwritten unless you want load a config file does not follow rules of KINEMATICS_docker.yaml

## move

There is one basic configuration of move algorithms

### Configuration

* navigation_package (string, default: $(optenv NAVIGATION_PACKAGE rb_ares_navigation)

* config_folder (string, default: $(find navigation_package)/config/move)

Folder where configuration is stored. Can be used to store different configurations. No need to be in the same package.

* config_file (string, default: $(arg kinematics)_move.yaml)

Config file for move. Should not be overwritten unless you want load a config file does not follow rules of KINEMATICS_move.yaml

## scan_filter

There is one basic configuration of filter scan algorithms
### LaserScan

* laser_scan (string, default: front_laser))

LaserScan to be filtered. Different filtered scans should be children of this namespace

* node_name (string, default: $(arg laser_scan)_filter)

Name of filtering node

### Topics

* input_scan (string, default: $(arg laser_scan)/scan)

Input topic for scan to be filtered (could be an already filtered scan)

* output_scan (string, default: $(arg laser_scan)/scan_filtered)

Output topic for result of scan filtering

### Configuration

* navigation_package (string, default: $(optenv NAVIGATION_PACKAGE rb_ares_navigation)

* config_folder (string, default: $(find navigation_package)/config/dockers)

Folder where configuration is stored. Can be used to store different configurations. No need to be in the same package.

* config_file (string, default: $(arg kinematics)_move.yaml)

Config file for move. Should not be overwritten unless you want load a config file does not follow rules of KINEMATICS_move.yaml
