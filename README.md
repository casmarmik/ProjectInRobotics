To start gathering feed from camera run:
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
