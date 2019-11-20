### ar_vision marker detection (based on ar_tracker_alvar)

1. ``` sudo apt-get install ros-melodic-ar-track-alvar ```
2. ``` roslaunch realsense2_camera rs_camera.launch ``` to launch realsense camera [github](https://github.com/IntelRealSense/realsense-ros)
3. ``` roslaunch ar_vision ar_vision.launch ``` 
4. ``` rosrun rviz rviz ```
5. In Rviz, change the frame id to ``` camera_link ```
6. In Rviz, add ``` By_topic ``` ```visualization_markers --> Marker ```
7. In Rviz, add ``` TF ``` to see the frames
