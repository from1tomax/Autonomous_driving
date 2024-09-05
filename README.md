# Team15_Cars_GroupRepository
This is a autonomus dirving project. The goal is enable a vehicle driving autonomusly in simulation. Obeying traffic signal and achieve less time consumtion. 

## Getting Started
0. get all dependencies listed below.
1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. source the setup.bash
  a.) roslaunch path_planner movebase_teb.launch
Different windows will be launched for monitoring. Desired velocity will be generated which obeys the traffic signal.  
Vehicle is still controllable manually with w-a-s-d from Unity.

5. checking velocity commands:
  rostopci echo /cmd_mutipller_node/ackermann_cmd_mux
  


# dependencies install
  sudo apt-get install ros-noetic-ackermann-msgs
  sudo apt-get install ros-noetic-vision-opencv
  sudo apt-get install ros-noetic-teb-local-planner
  sudo apt-get install ros-noetic-octomap-ros
  sudo apt-get install ros-noetic-cv-bridge
  sudo apt-get install ros-noetic-octomap-server
  sudo apt-get install ros-noetic-sensor-msgs
  sudo apt-get install ros-noetic-eigen-conversions
  sudo apt-get install ros-noetic-tf-conversions
  sudo apt-get install ros-noetic-move-base
  sudo apt-get install ros-noetic-move-base-msgs
  sudo apt-get install ros-noetic-geometry-msgs
  sudo apt-get install ros-noetic-tf2-ros
  sudo apt-get install ros-noetic-actionlib-msgs
  sudo apt-get install ros-noetic-message-generation
  sudo apt-get install ros-noetic-nav-msgs

