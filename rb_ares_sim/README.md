# rb_ares_sim

Packages for the simulation of the RB Ares.

## Packages

### rb_ares_sim_bringup

Launch files that launch the complete simulation of the robot.

Launch sample simulation with:

```
roslaunch rb_ares_sim_bringup rb_ares_complete.launch
```

### rb_ares_gazebo

Launch files and world files to start the simulation in Gazebo. Should not be used directly but throught the `rb_ares_sim_bringup` package, unless you want to do something non-standard.
