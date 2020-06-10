^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rb_ares_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2020-03-5)
------------------
* navigation: adding dependencies with move_base & costmap_prohibition_layer
* merging with kinetic-multirobot-devel
* updating mantainers
* Add Steel teb parameters file
* Update teb config and remove unused files
* Add costmap_prohibition_layer dependency
* Add parameters to change navigation and localization mode
* Added teb_local_planner and costmap_prohibition_layer as Required Components
* [rb_ares_navigation] odom topic changed to robotnik_base_control/odom
* Add missing dependencies
* removed maps subfolder of CMakeLists
* navigation: added docking launch for steel
* moving amcl,map_server from nav to loc pkg. Creating arguments for most of launch files
* navigation: adding default arguments to slam_gmapping launch file
* move_base_nomap modified to parametrize the params of yaml
* deleted move_base.launch
* [rb_ares_navigation]:params of move_base yaml parametrized to work with multirobot
* rb_ares_localization/navigation: changed topics and services to make work rl_utils
* [rb_ares_navigation]:added prefix to launch file
* [rb_ares_navigation]: frame updated with prefix
* [rb_ares_navigation]:prefix frame added to launch files and frames
* xacro updated to multirobot
* Merge branch 'kinetic-devel' into kinetic-multirobot-devel
* Merge branch 'indigo-devel-rc' into kinetic-devel
* Merge branch 'indigo-devel' into indigo-devel-rc
* Merge branch 'indigo-devel-rc' into kinetic-devel
* rb_ares_navigation map model added
* yaml,launch and xacro modified to multirobot
* rb_ares_navigation: corrected docking launch files
* rb_ares_navigation: added docking launch files
* updated navigation files
* corrected map and laser link according to new name in robotnik_sensors
* rb_ares_navigation: fixes map publish_frequency
* rb_ares_navigation: adds configuration file for teb_local_planner in holonomic configuration
* rb_ares_navigation: updating package.xml
* rb_ares_navigation: using teb_local_planner as default planner
* rb_ares_navigation: fix inflation_radius
* Adds rbk_warehouse map
