# SparseStereo

Note: stvo library is partially modified from [pl-slam public repository](https://github.com/rubengooj/pl-slam).

## Instructions for using
1. Before `catkin build` or `catkin_make` this package, please run `build_stvo.sh` to build all related libraries.
2. `catkin build` or `catkin_make`

## Running simulation
1. Open simulation environment
```
roslaunch nav_configs gazebo_turtlebot_stereo_sector_extra_world.launch
```

2. Run the feature matching
```
roslaunch sparse_stereo sparse_stereo.launch
```

The configuration file is in `config/config.yaml`.
