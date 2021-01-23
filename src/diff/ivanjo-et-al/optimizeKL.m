function cost = optimizeKL(x,XGt,Odo,RobotParam)

  % Initialization: Data
  NumRuns = length(XGt);
  
  % Initialization: Robot parameters
  ngear  = RobotParam.ngear;
  encRes = RobotParam.encRes;
  L      = RobotParam.L;
  D      = RobotParam.D;
  
  % Initialization: Adjustment parameters
  KL = x(1);

  % Initialization: Cost function
  cost = zeros(1,NumRuns);

  % Data processment
  for i=1:NumRuns
    XOdo = XGt{i}(1,:);
    [itf,~] = size(XGt{i});

    for k=2:itf
      % Linear and angular displacements
      deltaWh  = pi.*D.*Odo{i}(k,:)./(ngear*encRes);

      % Odometry pose
      deltaRob = [ (deltaWh(1) + deltaWh(2))/2 , 0 , (deltaWh(1) - deltaWh(2))/(KL*L(1)) ];

      % Odometry pose
      XOdo(1) = XOdo(1) + deltaRob(1)*cos(XOdo(3) + deltaRob(3)/2);
      XOdo(2) = XOdo(2) + deltaRob(1)*sin(XOdo(3) + deltaRob(3)/2);
      XOdo(3) = XOdo(3) + deltaRob(3);
    end

    cost(i) = XOdo(3)-XGt{i}(end,3);
  end
end