close all
clear
clc

% Initialization: Data
%   '../../data/tricyc/free/140120211611/140120211611',
filenameCenter = '../../data/tricyc/free/140120211611/140120211611_robot-center.csv';
filenameRobot  = '../../data/tricyc/free/140120211611/140120211611_robot-tricyc.csv';
[filePath,~,~] = fileparts(filenameRobot);
dataUniqueID   = '140120211611';
TypeEulerAngle = "YXZ";

% Initialization: Robot Initial Parameters
RobotParam.type   = 'tricyc';
RobotParam.ngear  = 1;
RobotParam.encRes = 1600;
RobotParam.L = 0.15;
RobotParam.D = 0.065;
RobotParam.ThOff = 0;
N = 1;

% Initialization: Options
visualize = true;

% Data Processing: Compute the transformation betweeen the centre and the body rigid bodies
[HRob,ThRobFrame,iMarkers] = computeRobotCenterTricyc(filenameCenter,filenameRobot,TypeEulerAngle,visualize);
fprintf('Press Enter to continue...\n');
pause
close all

% Data Processing: Odometry and Ground-truth
iTimeSyncAll = [];
for i=1:N
  filename = sprintf('%s/%s_run-%02d_odo.csv',filePath,dataUniqueID,i);
  [TimeOdo,Odo,XOdo] = processOdoData_tricyc(filename,RobotParam);
  filename = sprintf('%s/%s_run-%02d_opti.csv',filePath,dataUniqueID,i);
  [TimeData,XGt,Odo,XOdo,iTimeSync] = processOptiTrackData(filename,TypeEulerAngle,HRob,ThRobFrame,Odo,XOdo,TimeOdo,visualize);
  iTimeSyncAll = [ iTimeSyncAll ; iTimeSync ];

  % Write the data into a csv file
  filename = sprintf('%s/%s_run-%02d.csv',filePath,dataUniqueID,i);
  writematrix([ TimeData , XGt , Odo ],filename);
  fprintf('Press Enter to continue...\n');
  pause
  close all
end

% Data Processing: Metadata
metadata = {};
metadata{ 1,1} = 'type';     metadata{ 1,2} = RobotParam.type;
metadata{ 2,1} = 'ngear';    metadata{ 2,2} = RobotParam.ngear;
metadata{ 3,1} = 'encRes';   metadata{ 3,2} = RobotParam.encRes;
metadata{ 4,1} = 'Li';       metadata{ 4,2} = RobotParam.L;
metadata{ 5,1} = 'Di';
for i=1:length(RobotParam.D)
  metadata{5,i+1} = RobotParam.D(i);
end
metadata{ 6,1} = 'Thi';      metadata{ 6,2} = RobotParam.ThOff;
metadata{ 7,1} = 'N';        metadata{ 7,2} = N;
metadata{ 8,1} = 'L';        %metadata{ 8,2} = 1.7;
metadata{ 9,1} = 'imarkers';
for i=1:length(iMarkers)
  metadata{9,i+1} = iMarkers(i);
end
metadata{10,1} = 'gt_ti';    metadata{11,1} = 'gt_tf';    metadata{12,1} = 'odo_ti';    metadata{13,1} = 'odo_tf';
for i=1:N
  metadata{10,i+1} = iTimeSyncAll(i,1);
  metadata{11,i+1} = iTimeSyncAll(i,2);
  metadata{12,i+1} = iTimeSyncAll(i,3);
  metadata{13,i+1} = iTimeSyncAll(i,4);
end
filename = sprintf('%s/%s_metadata.csv',filePath,dataUniqueID);
writecell(metadata,filename);