# pallet_truck_navigation

This package contains the configuration for running the Navigation stack of the pallet_truck robot (ROS Navigation, Robotnik Navigation, etc)

# Structure

- Everything should be stored as parameters in YAML files, with only reasonable exceptions.
- Launch arguments should load the proper YAML config file.
- New launch files can be created for default loading of some YAML config files.
- Use argument _pass_all_args="true"_ to include launch files and pass all existing arguments

# Nodes

## move_base

There is one basic configuration of move_base, with some standard additional configurations for each planner.

### arguments

* id_robot (string, default: robot)
Name of spawned robot to connect

* prefix (string, default: (id_robot)_)
Prefix of spawned robot to connect

* overwrite_frames (bool, default: false))
If frames from YAML files must be overwritten (which by the way is a bad practice)

* odom_frame (string, default: (arg prefix)odom)
Odom frame to overwrite (which by the way is a bad practice)

* base_frame (string, default: (arg prefix)base_footprint)
Base frame to overwrite (which by the way is a bad practice)

* cmd_vel_topic (string, default: move_base/cmd_vel)
Topic for command velocity.

* odom_topic (string, default: robotnik_base_control/odom)
Topic to read odometry

* config_folder (string, default: (find pallet_truck_navigation)/config)
Folder where configuration is stored.

* costmap_common_params (string, default: costmap_common_params.yaml)
Default config file for common costmap parameters

* global_costmap_params (string, default: global_costmap_params_nomap.yaml)
Default config file for global costmap parameters

* local_costmap_params (string, default: local_costmap_params.yaml)
Default config file for local costmap parameters

* global_planner_params (string, default: global_planner_params.yaml)
Default config file for global planner parameters

* local_planner_params (string, default: teb_local_planner_diff_params.yaml)
Default config file for local planner parameters

* front_scan_topic (string, default: (optenv ROBOT_NAVIGATION_FRONT_2D_SCAN front_laser/filtered_scan))
Topic for front laser scan (if exists)

* rear_scan_topic (string, default: (optenv ROBOT_NAVIGATION_REAR_2D_SCAN rear_laser/scan))
Topic for rear laser scan (if exists)

* front_rgbd_to_scan_topic (string, default: (optenv ROBOT_NAVIGATION_FRONT_RGBD_TO_SCAN front_rgbd_camera/point_cloud_scan_filtered))
Topic for projected pointcloud from rgbd camera (if exists)

## dockers

## move



