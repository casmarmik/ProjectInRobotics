import numpy as np 
import open3d as o3d
import matplotlib.pyplot as plt
import itertools

pcd = o3d.io.read_point_cloud("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/1669206976261120.pcd")
pcd.remove_non_finite_points()
out_arr = np.asarray(pcd.points)  
print("points before down sampling: " + str(len(out_arr)))
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# pcd = pcd.voxel_down_sample(voxel_size=0.005)
bounds = [[-0.15, 0.2], [-0.136, 1], [-0.95, 0]]  # set the bounds
bounding_box_points = list(itertools.product(*bounds))  # create limit points
bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object
pcd = pcd.crop(bounding_box)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
# pcd.estimate_normals()
# pcd.orient_normals_towards_camera_location()
out_arr = np.asarray(pcd.points)  
outliers_pcd = pcd.select_by_index(inliers, invert=True)
out_arr = np.asarray(pcd.points)  
print("points after down sampling: " + str(len(out_arr)))
print("inliers: " + str(len(inliers)))

#figure out how to do spacial filtering - maybe just look at z-value and if it is above a threshold, then remove the point

o3d.visualization.draw([pcd])
# o3d.io.write_point_cloud("/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/plug_segmented.pcd", pcd)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# ax.scatter(x, y, z, c=z)
# plt.show()