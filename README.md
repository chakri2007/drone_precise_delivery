## Usage

### Setup the Workspace

./install_opencv.sh 
```
Install the submoduls
```
git submodule update --init --recursive
```
Build the ros2_ws
```
colcon build
```
After this runs, we do not need to build the whole workspace again, you can just build the individual packages you have modified

```
colcon build --packages-select precision_land
```
Source the workspace
```
source install/setup.bash 
```

#### Run the simulation environment
Launch PX4 sim
```
make px4_sitl gz_x500_mono_cam_down_aruco
```
Launch micro dds
```
MicroXRCEAgent udp4 -p 8888
```

Launch the ros_gz_bridge for getting the camera topic
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Launch the ros_gz_bridge for getting the camera info topic (this is how we get camera intrinsics)
```
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

Launch the ros2 nodes (aruco_tracker)
```
source install/setup.bash 
ros2 run aruco_tracker aruco_tracker 
```
View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```

Launch the ros2 nodes (precision_land)
```
source install/setup.bash 
ros2 run precision_land precision_land
```
Once the nodes are running
1. Take off the drone to a height
2.Then select precision land mode in qgc
