# ROS2-PurePursuitControl-PathPlanning-Tracking
Route Planning and Tracking Application Developed Using ROS2, Turtlebot3, A*, and PurePursuit


![Screenshot_5](https://user-images.githubusercontent.com/87595266/205762696-91c48af3-617d-4784-a1d9-ebe66400df4c.png)

# V1.1 Version Update | 02/09/2023

![1-min](https://user-images.githubusercontent.com/87595266/217926638-2232239a-5f35-469e-829c-a2883f835bdc.gif)



## Innovations / New Features


- Pure Pursuit algorithm has been optimized, providing route tracking optimization.
- B-Spline algorithm has been added for path waypoints, providing path smoothing and improving route tracking optimization.

 ![Screenshot from 2023-01-31 21-28-34](https://user-images.githubusercontent.com/87595266/217913980-c0ec9e54-0f9c-4488-8a21-2d258873a409.png)
 
 - The Costmap algorithm has been optimized.

# How it works?

Start a Gazebo simulation world. For example;

`export TURTLEBOT3_MODEL=burger`


`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`



Then, run the mapping package to perform mapping.

`ros2 launch slam_toolbox online_async_launch.py`

Run the PathPlanning-Tracking package.
  
`ros2 run nav_controller control`

Then, set the target point via rviz2.

# YouTube Preview & Usage Video
https://youtu.be/r_2mMyaLLaI

## Requirements

- ROS2 - Humble
- Slam Toolbox
- Turtlebot3 Package
- Gazebo Simulator

