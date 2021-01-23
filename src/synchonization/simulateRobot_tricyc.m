function [XOdo] = simulateRobot_tricyc(X0,Odo,TOdo,RobotParam)

  % Initialization: Data
  [itf,numWh] = size(Odo);
  XOdo = zeros(itf,3);
  
  % Initialization: Robot parameters
  ngear  = RobotParam.ngear;
  encRes = RobotParam.encRes;
  L      = RobotParam.L;
  D      = RobotParam.D;
  ThOff  = RobotParam.ThOff;

  % Simulate robot
  XOdo(1,:) = X0(1,:);
  for i=2:itf
    % Linear and angular displacements
    ThWh     = Odo(i,2);
    deltaWh  = pi*D(1)*Odo(i,1)/(ngear*encRes);
    deltaRob = deltaWh * [ cos(ThWh+ThOff) , 0 , sin(ThWh+ThOff)/L(1) ];

    % Odometry pose
    XOdo(i,1) = XOdo(i-1,1) + deltaRob(1)*cos(XOdo(i-1,3) + deltaRob(3)/2);
    XOdo(i,2) = XOdo(i-1,2) + deltaRob(1)*sin(XOdo(i-1,3) + deltaRob(3)/2);
    XOdo(i,3) = XOdo(i-1,3) + deltaRob(3);
  end
end