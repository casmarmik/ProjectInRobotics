Install openni-launch and openni-camera
```
sudo apt install ros-noetic-openni-camera
sudo apt install ros-noetic-openni-launch
```
To start gathering feed from camera first start a roscore then run:
```
roslaunch openni_launch openni.launch
```
To save a point cloud as a .pcd run:
```
rosrun pointnet pointcloud_to_pcd input:=/camera/depth/points
```
And then call a service when you want to save a point cloud:
```
rosservice call /pointcloud_to_pcd
```


To run the point cloud processing program first install pcl_ros
```
sudo apt install ros-noetic-pcl-ros
```
Then run
```
rosrun pointnet pointnet
```

This migth be useful for finding the trasnformation between the robot and the camera
https://github.com/opencv/opencv/blob/b5a9a6793b3622b01fe6c7b025f85074fec99491/modules/calib3d/test/test_calibration_hand_eye.cpp#L145