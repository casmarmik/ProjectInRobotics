clc; clear; close all; 

% Collect camera poses
camera_poses_file = importdata('/home/mads/project_in_robotics/project_in_robotics/vision/data/calibration/camera_poses.txt');
camera_poses = zeros(4,4,20);
idx_counter = 1;
for i=1:20
    camera_poses(:,:,i) = camera_poses_file(idx_counter:idx_counter+3,1:4);
    idx_counter = idx_counter + 4;
end

% Calculate x, y and z means
meanx = mean(camera_poses(1,4,:));
meany = mean(camera_poses(2,4,:));
meanz = mean(camera_poses(3,4,:));

% Calculate mean rotations, based on
% https://se.mathworks.com/help/nav/ref/quaternion.meanrot.html matlab and 
quaternions = rotm2quat(camera_poses(1:3,1:3,:));
quaternions = quaternion(quaternions);
mean_rot = meanrot(quaternions);
mean_rot_matrix = quat2rotm(mean_rot);

% Construct average camera pose
camera_pose = eye(4,4);
camera_pose(1:3,1:3) = mean_rot_matrix;
camera_pose(1,4) = meanx;
camera_pose(2,4) = meany;
camera_pose(3,4) = meanz;
camera_pose
rotm2eul(camera_pose(1:3,1:3), 'XYZ')

figure(1)
poseplot(mean_rot, camera_pose(1:3,4), "ENU");