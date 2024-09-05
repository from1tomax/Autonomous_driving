## Getting Started


1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. source the setup.bash
  a.) roslaunch path_planner movebase_teb.launch
  
Different windows will be launched for monitoring. Desired velocity will be generated which obeys the traffic signal.  
Vehicle is still controllable manually with w-a-s-d from Unity.




# dependencies
  ackermann_msgs; vision_opencv; teb_local_planner;octomap_ros; cv_bridge;octomap_server; sensor_msgs;eigen_conversions;tf_conversions;move_base;move_base_msgs; geometry_msgs; tf2_ros; actionlib_msgs;message_generation; nav_msgs
