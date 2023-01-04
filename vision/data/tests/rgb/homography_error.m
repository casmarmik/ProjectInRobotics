clc; clear; close all;

plug_error = importdata('/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/plug_error.txt');
screw_error =  importdata('/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/screw_error.txt');

% Plug
plug_error_x = plug_error(:,1);
plug_error_y = plug_error(:,2);
plug_error_ang = plug_error(:,3);
plug_computation_time = plug_error(:,4);

% Screw
screw_error_x = screw_error(:,1);
screw_error_y = screw_error(:,2);
screw_error_ang = screw_error(:,3);
screw_computation_time = screw_error(:,4);

mean(screw_error_x);
std(screw_error_x);
mean(screw_error_y);
std(screw_error_y);
mean(screw_error_ang);
std(screw_error_ang);

mean(plug_error_x);
std(plug_error_x);
mean(plug_error_y);
std(plug_error_y);
mean(plug_error_ang);
std(plug_error_ang);

absolute_error_plug = sqrt(plug_error_x.^2+ plug_error_y.^2);
absolute_error_screw = sqrt(screw_error_x.^2+ screw_error_y.^2);
(mean(absolute_error_screw) + mean(absolute_error_plug)) / 2
(mean(screw_error_ang) + mean(plug_error_ang)) / 2



% Grasping area analysis
mean_screw1 = mean(absolute_error_screw(1:4));
mean_screw2 = mean(absolute_error_screw(5:8));
mean_screw3 = mean(absolute_error_screw(9:12));
mean_screw4 = mean(absolute_error_screw(13:16));
mean_screw5 = mean(absolute_error_screw(17:20));
mean_screw6 = mean(absolute_error_screw(21:24));
mean_screw7 = mean(absolute_error_screw(25:28));
mean_screw8 = mean(absolute_error_screw(29:32));
mean_screw9 = mean(absolute_error_screw(33:36));
std_screw1 = std(absolute_error_screw(1:4));
std_screw2 = std(absolute_error_screw(5:8));
std_screw3 = std(absolute_error_screw(9:12));
std_screw4 = std(absolute_error_screw(13:16));
std_screw5 = std(absolute_error_screw(17:20));
std_screw6 = std(absolute_error_screw(21:24));
std_screw7 = std(absolute_error_screw(25:28));
std_screw8 = std(absolute_error_screw(29:32));
std_screw9 = std(absolute_error_screw(33:36));

mean_angle1 = mean(screw_error_ang(1:4));
mean_angle2 = mean(screw_error_ang(5:8));
mean_angle3 = mean(screw_error_ang(9:12));
mean_angle4 = mean(screw_error_ang(13:16));
mean_angle5 = mean(screw_error_ang(17:20));
mean_angle6 = mean(screw_error_ang(21:24));
mean_angle7 = mean(screw_error_ang(25:28));
mean_angle8 = mean(screw_error_ang(29:32));
mean_angle9 = mean(screw_error_ang(33:36));
std_angle1 = std(screw_error_ang(1:4));
std_angle2 = std(screw_error_ang(5:8));
std_angle3 = std(screw_error_ang(9:12));
std_angle4 = std(screw_error_ang(13:16));
std_angle5 = std(screw_error_ang(17:20));
std_angle6 = std(screw_error_ang(21:24));
std_angle7 = std(screw_error_ang(25:28));
std_angle8 = std(screw_error_ang(29:32));
std_angle9 = std(screw_error_ang(33:36));

mean_plug1 = mean(absolute_error_plug(1:4));
mean_plug2 = mean(absolute_error_plug(5:8));
mean_plug3 = mean(absolute_error_plug(9:12));
mean_plug4 = mean(absolute_error_plug(13:16));
mean_plug5 = mean(absolute_error_plug(17:20));
mean_plug6 = mean(absolute_error_plug(21:24));
mean_plug7 = mean(absolute_error_plug(25:28));
mean_plug8 = mean(absolute_error_plug(29:32));
mean_plug9 = mean(absolute_error_plug(33:36));
std_plug1 = std(absolute_error_plug(1:4));
std_plug2 = std(absolute_error_plug(5:8));
std_plug3 = std(absolute_error_plug(9:12));
std_plug4 = std(absolute_error_plug(13:16));
std_plug5 = std(absolute_error_plug(17:20));
std_plug6 = std(absolute_error_plug(21:24));
std_plug7 = std(absolute_error_plug(25:28));
std_plug8 = std(absolute_error_plug(29:32));
std_plug9 = std(absolute_error_plug(33:36));

mean_plug_angle1 = mean(plug_error_ang(1:4));
mean_plug_angle2 = mean(plug_error_ang(5:8));
mean_plug_angle3 = mean(plug_error_ang(9:12));
mean_plug_angle4 = mean(plug_error_ang(13:16));
mean_plug_angle5 = mean(plug_error_ang(17:20));
mean_plug_angle6 = mean(plug_error_ang(21:24));
mean_plug_angle7 = mean(plug_error_ang(25:28));
mean_plug_angle8 = mean(plug_error_ang(29:32));
mean_plug_angle9 = mean(plug_error_ang(33:36));
std_plug_angle1 = std(plug_error_ang(1:4));
std_plug_angle2 = std(plug_error_ang(5:8));
std_plug_angle3 = std(plug_error_ang(9:12));
std_plug_angle4 = std(plug_error_ang(13:16));
std_plug_angle5 = std(plug_error_ang(17:20));
std_plug_angle6 = std(plug_error_ang(21:24));
std_plug_angle7 = std(plug_error_ang(25:28));
std_plug_angle8 = std(plug_error_ang(29:32));
std_plug_angle9 = std(plug_error_ang(33:36));

figure(1)
bar_data = [mean_screw1 mean_plug1; mean_screw2 mean_plug2; mean_screw3 mean_plug3; mean_screw4 mean_plug4; ...
            mean_screw5 mean_plug5; mean_screw6 mean_plug6; mean_screw7 mean_plug7; mean_screw8 mean_plug8; ...
            mean_screw9 mean_plug9];
positions = {'(329.03, -355.87)', '(429.03, -355.87)', '(529.03, -355.87)', ...
          '(329.03, -455.87)', '(429.03, -455.87)', '(529.03, -455.87)', ...
          '(329.03, -555.87)', '(429.03, -555.87)', '(529.03 -555.87)'};
bar(bar_data)
hold on
ngroups = size(bar_data, 1);
nbars = size(bar_data, 2);
% Calculating the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
err = [std_screw1 std_plug1; std_screw2 std_plug2; std_screw3 std_plug3; std_screw4 std_plug4; 
       std_screw5 std_plug5; std_screw6 std_plug6; std_screw7 std_plug7; std_screw8 std_plug8; 
       std_screw9 std_plug9];
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, bar_data(:,i), err(:,i), '.');
end

ylabel("Error [mm]")
xlabel("Position [mm]")
title("Euclidian distance error at different positions")
legend({'Screw', 'Plug'})
xticklabels(positions);

figure(2)
bar_data = [mean_angle1 mean_plug_angle1; mean_angle2 mean_plug_angle2; mean_angle3 mean_plug_angle3;
            mean_angle4 mean_plug_angle4; mean_angle5 mean_plug_angle5; mean_angle6 mean_plug_angle6;
            mean_angle7 mean_plug_angle7; mean_angle8 mean_plug_angle8; mean_angle9 mean_plug_angle9];
positions = {'(329.03, -355.87)', '(429.03, -355.87)', '(529.03, -355.87)', ...
          '(329.03, -455.87)', '(429.03, -455.87)', '(529.03, -455.87)', ...
          '(329.03, -555.87)', '(429.03, -555.87)', '(529.03 -555.87)'};
bar(bar_data)
hold on
ngroups = size(bar_data, 1);
nbars = size(bar_data, 2);
% Calculating the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
err = [std_angle1 std_plug_angle1; std_angle2 std_plug_angle2; std_angle3 std_plug_angle3; 
       std_angle4 std_plug_angle4; std_angle5 std_plug_angle5; std_angle6 std_plug_angle6; 
       std_angle7 std_plug_angle7; std_angle8 std_plug_angle8; std_angle9 std_plug_angle9];
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, bar_data(:,i), err(:,i), '.');
end

ylabel("Error [degrees]")
xlabel("Position [mm]")
title("Error in angle at different positions")
legend({'Screw', 'Plug'})
xticklabels(positions);

