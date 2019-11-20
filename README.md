### ar_vision marker detection (based on ar_tracker_alvar)

``` sudo apt-get install ros-melodic-ar-track-alvar ```
``` sudo apt-get install ros-melodic-rgbd-launch ```
``` roslaunch realsense2_camera rs_rdbd.launch ``` to launch realsense camera [github](https://github.com/IntelRealSense/realsense-ros)
``` roslaunch ar_vision ar_vision.launch ``` 
``` rosrun rviz rviz ```
In Rviz, change the frame id to ``` camera_link ```
In Rviz, add ``` By_topic ``` ```visualization_markers --> Marker ```
In Rviz, add ``` TF ``` to see the frames
Print out the ``` table_8_9_10.png``` and place it infront of the realsense camera
