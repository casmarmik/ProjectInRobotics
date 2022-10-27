import numpy as np 
import open3d as o3d
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("/home/marcus/pir/ros_ws/src/pointnet/data/p1onboxangle.pcd")
pcd.remove_non_finite_points()
out_arr = np.asarray(pcd.points)  
print("points before down sampling: " + str(len(out_arr)))
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
pcd = pcd.voxel_down_sample(voxel_size=0.005)
out_arr = np.asarray(pcd.points)  
print("points after down sampling: " + str(len(out_arr)))

#figure out how to do spacial filtering - maybe just look at z-value and if it is above a threshold, then remove the point

o3d.visualization.draw([pcd])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# ax.scatter(x, y, z, c=z)
# plt.show()