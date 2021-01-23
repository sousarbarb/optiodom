function [TimeOdo,Odo,XOdo] = processOdoData_omni4(filename,RobotParam)
  
  % Retrieve data from the CSV file
  M = readmatrix(filename);

  % Odometry data: Timestamps
  TimeOdo = M(:,1);
  TimeOdo = TimeOdo - TimeOdo(1);

  % Odometry data: Thicks per cycle
  Odo = M(:,2:end);
  %Odo = -M(:,2:end);

  % Odometry data: Robot Position and Orientation
  [XOdo] = simulateRobot_omni4([0,0,0],Odo,TimeOdo(2)-TimeOdo(1),RobotParam);
end