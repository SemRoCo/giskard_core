# giskard
A library and some ROS nodes for robot motion control.

## Installation
`.rosinstall`:
```
- git: {local-name: src/giskard, uri: 'git@github.com:airballking/giskard.git'}
- git: {local-name: src/expressiongraph, uri: 'git@github.com:airballking/expressiongraph.git'}
- git: {local-name: src/qpOASES, uri: 'git@github.com:airballking/qpOASES.git'}
```

* roscd expressiongraph
* git checkout catkin
* catkin_make run_tests    (2 fails: expressiongraph.Frame, expressiongraph.Rotation)

## Parsing urdfs into yamls
`rosrun giskard extract_expression <start_link> <end_link> <urdf_file> (optional <output_file>)`

Example:
`rosrun giskard extract_expression torso_lift_link l_wrist_roll_link test_data/pr2.urdf asd.yaml`
