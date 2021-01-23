function cost = optimizeKD(x,XGt,Odo,RobotParam)

  % Initialization: Data
  NumRuns = length(XGt);
  
  % Initialization: Robot parameters
  ngear  = RobotParam.ngear;
  encRes = RobotParam.encRes;
  L      = RobotParam.L;
  D      = RobotParam.D;
  
  % Initialization: Adjustment parameters
  KD = [ x(1) , x(2) ];
  %KDR = x(1);  % Adjusment parameter of the left wheel diameter
  %KDL = x(2);  % Adjusment parameter of the right wheel diameter

  % Initialization: Cost function
  cost = zeros(1,NumRuns);

  % Data processment
  for i=1:NumRuns
    XOdo = XGt{i}(1,:);
    [itf,~] = size(XGt{i});

    for k=2:itf
      % Linear and angular displacements
      deltaWh  = pi.*KD.*D.*Odo{i}(k,:)./(ngear*encRes);

      % Odometry pose
      deltaRob = [ (deltaWh(1) + deltaWh(2))/2 , 0 , (deltaWh(1) - deltaWh(2))/L(1) ];

      % Odometry pose
      XOdo(1) = XOdo(1) + deltaRob(1)*cos(XOdo(3) + deltaRob(3)/2);
      XOdo(2) = XOdo(2) + deltaRob(1)*sin(XOdo(3) + deltaRob(3)/2);
      XOdo(3) = XOdo(3) + deltaRob(3);
    end

    cost(i) = ((XOdo(1)-XGt{i}(end,1)).^2 + (XOdo(2)-XGt{i}(end,2)).^2).^(1/2);
  end
end