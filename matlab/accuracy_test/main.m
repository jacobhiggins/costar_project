% This file compares GPS data and odom data from IMU

%% Get raw data from bag file
bagfilename = "bebop_record_2022-01-19-11-58-21.bag"; % Name of bag file
bagfilepath = "./data/"+bagfilename;
bag = rosbag(bagfilepath);
odom_topic_name = '/bebop/odom'; gps_topic_name = '/bebop/fix';
odom_topic_msgs = select(bag,'Topic',odom_topic_name); gps_topic_msgs = select(bag,'Topic',gps_topic_name);
odom_raw_data = readMessages(odom_topic_msgs,'DataFormat','struct'); gps_raw_data = readMessages(gps_topic_msgs,'DataFormat','struct');

%% Process Odom Data
odom_data = struct("x",[],"y",[],"theta",[]); % Structure that contains x and y position
for i = 1:size(odom_raw_data,1)
    msg = odom_raw_data{i};
    % XY position
    odom_data.x(end+1) = msg.Pose.Pose.Position.X; odom_data.y(end+1) = 1*msg.Pose.Pose.Position.Y;
    % Heading (theta)
    eul = quat2eul([msg.Pose.Pose.Orientation.W msg.Pose.Pose.Orientation.X msg.Pose.Pose.Orientation.Y msg.Pose.Pose.Orientation.Z]);
    odom_data.theta(end+1) = 180*eul(1)/pi;
end

%% Process GPS Data
EARTH_RAD = 111139;
gps_data = struct("x",[],"y",[]);
msg_start = gps_raw_data{1};
coord_start = [msg_start.Latitude;msg_start.Longitude];
for i = 1:size(gps_raw_data)
    msg = gps_raw_data{i};
    coord = [msg.Latitude;msg.Longitude] - coord_start;
    xy_pos = EARTH_RAD*coord;
    gps_data.x(end+1) = xy_pos(1); gps_data.y(end+1) = xy_pos(2);
end

%% Plot Trajectories
% Plot odom trajectory
close all; figure(1); hold on; axis equal;
plot(odom_data.x,odom_data.y);
plot(gps_data.x,gps_data.y);

figure(2); hold on;
plot(odom_data.theta);

%% Animate Trajectories

%%%%VECTORIZE AND SAVE AS EPS (use cloudconvert.com to change .eps to .emf
%%%%for powerpoint usage

% axis off
% set(gcf,'Color','none')
% set(gca,'Color','none')
% exportgraphics(gca,'files/traj8.eps','BackgroundColor','none')
