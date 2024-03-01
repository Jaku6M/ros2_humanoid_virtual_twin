# ROS2-Humanoid-Virtual-Twin

**Branch Melmanruchynogi**

Branch is responsible for creating humanoid robot Leg in Rviz2 and move it in a sinusoidal way.

_MOVING LEG_

*source the repository: source install/local_setup.bash 

Launchig the moving leg: ```ros2 launch ros2_humanoid_virtual_twin rviz_legmoves.launch.py```

User can change frequency of the movement by typing: ```ros2 run ros2_humanoid_virtual_twin client <here type your frequency>```

example of setting frequency to 2.5: ```ros2 run ros2_humanoid_virtual_twin client 2.5```

The frequency and amplitude can be changed in rviz_legmoves.launch.py directly because both of them are parameters.

_STATIC LEG ACTIVATED BY ACTION_

Downlad action package from https://github.com/Jaku6M/swing_action and perform instructions in Readme

Build the repository ```colcon build --packages-up-to ros2_humanoid_virtual_twin```

*source the repository: source install/local_setup.bash 

Launchig the leg swinging in the Hip joint for 3 seconds: ```ros2 launch ros2_humanoid_virtual_twin rviz_swingaction.launch.py```

If willing user can repeat action of leg swing by typing: ```ros2 run ros2_humanoid_virtual_twin swing_client```

Time of swing can be changed in file: swing_client.cpp
