# pallet_truck_gazebo

Package for the simulation of the Autonomous Pallet Truck in the Gazebo simulator

### launch

#### spawn_simulation.launch

Spawns the simulation environment, without robots. Has the following arguments:

* ``` world, default="$(find pallet_truck_gazebo)/worlds/pallet_truck_epal.world" ```

World file to be loaded, as an absolute path in the file system.

* ``` debug, default="false" ```

If simulator should output debug information

* ``` paused, default="false" ```

If simulator should start paused

* ``` headless, default="false" ```

If simulator should start in headless mode

* ``` gui, default="true" ```

If simulator should show its GUI.

#### spawn_robot.launch

Spawns a robot in a running simulation environment. It only spawns the robot in the environement, it does not spawn any of the required software for the robot (e.g. control, navigation, etc.). This is done in the pallet_truck_sim_bringup package.

This launch has the following arguments:

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
