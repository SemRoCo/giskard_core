# giskard [![Build Status](https://travis-ci.org/SemRoCo/giskard.svg?branch=master)](https://travis-ci.org/SemRoCo/giskard)
A library and some ROS nodes for robot motion control.

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Indigo```:
```
source /opt/ros/indigo/setup.bash          # start using ROS Indigo
mkdir -p ~/giskard_ws/src                  # create directory for workspace
cd ~/giskard_ws                            # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/SemRoCo/giskard/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/giskard_ws/devel/setup.bash       # source new overlay
```

## Parsing urdfs into yamls
`rosrun giskard extract_expression <start_link> <end_link> <urdf_file> (optional <output_file>)`

Example:
`rosrun giskard extract_expression torso_lift_link l_wrist_roll_link test_data/pr2.urdf asd.yaml`
