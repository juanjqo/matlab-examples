# Constrained Kinematic Control

This example performs a constrained kinematic control using the Kuka LBR4 serial manipulator on a dynamics-enabled CoppeliaSim scene. 
The goal is to minimize the distance between the end effector and a 
target plane (green plane) while preventing collisions between the end-effector and other planes (red planes) in the workspace. The constraints are defined using the [VFI framework](https://arxiv.org/pdf/1804.11270).


![ezgif com-video-to-gif (1)](https://user-images.githubusercontent.com/23158313/234257783-44fc4539-f745-4a6b-91d9-fe5b40754668.gif)


1. Open the [constrained_kinematic_control.ttt](https://github.com/dqrobotics/coppeliasim-scenes/tree/main) scene in CoppeliaSim

2. Ensure you have installed or defined the path for the DQ Robotics library, and the [ZMQ client for Matlab](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/coppeliasim-v4.7.0-rev2/clients/matlab)
