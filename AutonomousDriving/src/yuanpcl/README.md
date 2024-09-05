

## STEPS: generate the poindcloud2

//open Terminal and

    cd /AutonomousDriving

1. Terminal
```
source devel/setup.bash
catkin build
source devel/setup.bash
roslaunch simulation simulation.launch
```

//then,

2. Terminal 
```
source devel/setup.bash
roslaunch yuanpcl openni_record_player.launch
```

//THEN the PointCloud2_msg will be publish to the rostopic: 
    
    /unity_ros/OurCar/Sensors/DepthCamera/points.


//!!! if you wanna see the map with rviz, then,

3. Terminal
```
rviz
```
// "add" - "PointCloud2" - choice the topic - point cloud will be showed on the screen.
