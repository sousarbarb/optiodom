function [TimeOdo,Odo,XOdo] = processOdoData_diff(filename,RobotParam)
  
  % Retrieve data from the CSV file
  M = readmatrix(filename);

  % Odometry data: Timestamps
  TimeOdo = M(:,1);
  TimeOdo = TimeOdo - TimeOdo(1);

  % Odometry data: Thicks per cycle
  %Odo = M(:,2:end);
  Odo = [ M(:,3) , M(:,2) ];

  % Odometry data: Robot Position and Orientation
  [XOdo] = simulateRobot_diff([0,0,0],Odo,TimeOdo(2)-TimeOdo(1),RobotParam);
end