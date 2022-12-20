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
meany = mean(camera_poses(2,4,:));e
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

% Plot all the poses and also the average
axes = 0:0.0001:0.01;
figure("Name", "Camera_poses")
for i=1:20
    %poseplot(quaternions(i), camera_poses(1:3,4, i), "ENU", ScaleFactor=0.1);
    pointsx = zeros(4, length(axes));
    pointsy = zeros(4, length(axes));
    pointsz = zeros(4, length(axes));
    pointsx(1,:) = axes;
    pointsy(2,:) = axes;
    pointsz(3,:) = axes;
    pointsx(4,:) = 1;
    pointsy(4,:) = 1;
    pointsz(4,:) = 1;
    x = zeros(4, length(axes));
    y = zeros(4, length(axes));
    z = zeros(4, length(axes));
    for j=1:length(axes)
        x(:,j) = camera_poses(:,:,i)*pointsx(:,j);
        y(:,j) = camera_poses(:,:,i)*pointsy(:,j);
        z(:,j) = camera_poses(:,:,i)*pointsz(:,j);
    end

    plot3(x(1,:), x(2,:), x(3,:), 'o', color="red");
    hold on;
    plot3(y(1,:), y(2,:), y(3,:),'o', color="green");
    plot3(z(1,:), z(2,:), z(3,:),'o' ,color="blue");
end
%xlim([-0.5 0.6])
%ylim([-0.94 -0.84])
%zlim([0.34 0.44])


% Plot the average pose
axes = 0:0.0001:0.03;
pointsx = zeros(4, length(axes));
pointsy = zeros(4, length(axes));
pointsz = zeros(4, length(axes));
pointsx(1,:) = axes;
pointsy(2,:) = axes;
pointsz(3,:) = axes;
pointsx(4,:) = 1;
pointsy(4,:) = 1;
pointsz(4,:) = 1;
x = zeros(4, length(axes));
y = zeros(4, length(axes));
z = zeros(4, length(axes));
for j=1:length(axes)
    x(:,j) = camera_pose*pointsx(:,j);
    y(:,j) = camera_pose*pointsy(:,j);
    z(:,j) = camera_pose*pointsz(:,j);
end
plot3(x(1,:), x(2,:), x(3,:), 'o', color="cyan");
hold on;
plot3(y(1,:), y(2,:), y(3,:),'o', color="yellow");
plot3(z(1,:), z(2,:), z(3,:),'o' ,color="black");
hold off;
grid on;
title("Estiamted camera poses and average camera pose")
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")

figure("name", "average pose and base frame")
% Plot the average pose
axes = 0:0.0001:0.3;
yaxes = 0:0.0001:0.2;
pointsx = zeros(4, length(axes));
pointsy = zeros(4, length(axes));
pointsz = zeros(4, length(axes));
pointsx(1,:) = axes;
pointsy(2,:) = axes;
pointsz(3,:) = axes;
pointsx(4,:) = 1;
pointsy(4,:) = 1;
pointsz(4,:) = 1;
x = zeros(4, length(axes));
y = zeros(4, length(axes));
z = zeros(4, length(axes));
% Convert points to be in reference of robot coordinate system instead
% of matlab coordinate system 
matlab_to_base = zeros(4,4);
matlab_to_base(4,4) = 1;
matlab_to_base(1:3,1:3) = rotz(90);
for j=1:length(axes)
    x(:,j) = matlab_to_base * camera_pose*pointsx(:,j);
    y(:,j) = matlab_to_base * camera_pose*pointsy(:,j);
    z(:,j) = matlab_to_base * camera_pose*pointsz(:,j);
end
x0 = matlab_to_base * camera_pose * [0.0, 0.0, 0.0 1.0]';
plot3(x(1,:), x(2,:), x(3,:), 'o', color="red");
hold on;
plot3(y(1,:), y(2,:), y(3,:),'o', color="green");
plot3(z(1,:), z(2,:), z(3,:),'o' ,color="blue");
txt = 'Camera frame';
text(x0(1), x0(2), x0(3), txt,'FontSize',18);

% Plot the average pose
pointsx = zeros(4, length(axes));
pointsy = zeros(4, length(axes));
pointsz = zeros(4, length(axes));
pointsx(1,:) = -1.*axes;
pointsy(2,:) = axes;
pointsz(3,:) = axes;
title("Estimated camera frame")
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
plot3(pointsx(1,:), pointsx(2,:), pointsx(3,:), 'o', color="green");
plot3(pointsy(1,:), pointsy(2,:), pointsy(3,:),'o', color="red");
plot3(pointsz(1,:), pointsz(2,:), pointsz(3,:),'o' ,color="blue");
txt = 'Base frame';
text(0,0, 0,txt, 'FontSize',16);
grid on;
hold off;

