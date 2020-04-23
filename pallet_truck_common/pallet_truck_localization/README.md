# pallet_truck_localization

This package contains the configuration for running the Localization stack of the pallet_truck robot (Only AMCL and Map_server for the moment).

## Structure

- Everything should be stored as parameters in YAML files, only reasonable exceptions are accepted.
- Launch arguments should load the proper YAML config file, not overwrite parameters.
- New launch files can be created for default loading of some YAML config files, only reasonable exceptions are accepted.
- Add new arguments and parameters to main launch file, and just add arguments to derived ones, only reasonable exceptions are accepted.
- Follow similar rules when creating files, folders, parameters and arguments to the ones already existing.

## Common arguments

### Robot configurations

* robot_id (string, default: robot)

Name of spawned robot to connect

* prefix (string, default: $(arg robot_id)_)

Prefix of spawned robot to connect

* kinematics (string, default: ROBOT_KINEMATICS diff)

Kinematics of robot.

* localization_system (string, default: amcl)

Localization system of robot.

### Configuration

* localization_package (string, default: $(optenv LOCALIZATION_PACKAGE pallet_truck_localization)

* config_folder (string, default: $(find localization_package)/config)

Folder where configuration is stored. Can be used to store different configurations.

### Robot initial position

* x_init_pose (float, default: 0)

* y_init_pose (float, default: 0)

* z_init_pose (float, default: 0)

## AMCL

### Files

* amcl_params (string, default: amcl.yaml)

Config file for amcl localization system.

### Topics

* scan_topic (string, default: $(optenv ROBOT_LOCALIZATION_SCAN_TOPIC front_laser/scan))

Topic of the laser scan for the amcl

* map_topic (string, default: map)

When the use_map_topic parameter is set, AMCL subscribes to the map's topic to retrieve the map used for laser-based localization

## Map Map_server

### Maps

* map_file (string, default= empty/map_empty.yaml)

* frame_id (string, default= $(arg prefix)map)
