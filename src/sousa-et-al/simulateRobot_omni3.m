function [XOdo,iSamples] = simulateRobot_omni3(X0,Odo,TOdo,RobotParam,sampleDist)

  % Initialization: Data
  [itf,numWh] = size(Odo);
  XOdo = zeros(itf,3);
  
  % Initialization: Robot parameters
  ngear  = RobotParam.ngear;
  encRes = RobotParam.encRes;
  L      = RobotParam.L;
  D      = RobotParam.D;
  if (~RobotParam.useJ)
    J = [
      -(3^(1/2))/3 , (3^(1/2))/3 ,        0 ; ...
              -1/3 ,        -1/3 ,      2/3 ; ...
          -1/(3*L) ,    -1/(3*L) , -1/(3*L)   ...
    ];
    Jinv = eye(3)/J;
  else
    J = RobotParam.J;
    Jinv = RobotParam.Jinv;
  end

  % Initialization: define samples given a certain distance
  if (~isempty(sampleDist))
    iSamples  = [];
    deltaRobD = 0;
    sampling  = true;
  else
    iSamples  = NaN;
    sampling  = false;
  end

  % Simulate robot
  XOdo(1,:) = X0(1,:);
  for i=2:itf
    % Linear and angular displacements
    deltaWh  = pi.*D.*Odo(i,:)./(ngear*encRes);
    deltaRob = (J * deltaWh')';

    % Odometry pose
    if ( abs(deltaRob(3))/TOdo < 1e-5 )
      XOdo(i,1) =                      ...
        XOdo(i-1,1)                  + ...
        deltaRob(1)*cos(XOdo(i-1,3)) - ...
        deltaRob(2)*sin(XOdo(i-1,3));
      XOdo(i,2) =                      ...
        XOdo(i-1,2)                  + ...
        deltaRob(1)*sin(XOdo(i-1,3)) + ...
        deltaRob(2)*cos(XOdo(i-1,3));
    else
      XOdo(i,1) =                                                                                                    ...
        XOdo(i-1,1)                                                                                                + ...
        (deltaRob(1)*sin(deltaRob(3))+deltaRob(2)*(cos(deltaRob(3))-1))*cos(XOdo(i-1,3)+deltaRob(3)/2)/deltaRob(3) - ...
        (deltaRob(1)*(1-cos(deltaRob(3)))+deltaRob(2)*sin(deltaRob(3)))*sin(XOdo(i-1,3)+deltaRob(3)/2)/deltaRob(3);
      XOdo(i,2) =                                                                                                    ...
        XOdo(i-1,2)                                                                                                + ...
        (deltaRob(1)*sin(deltaRob(3))+deltaRob(2)*(cos(deltaRob(3))-1))*sin(XOdo(i-1,3)+deltaRob(3)/2)/deltaRob(3) + ...
        (deltaRob(1)*(1-cos(deltaRob(3)))+deltaRob(2)*sin(deltaRob(3)))*cos(XOdo(i-1,3)+deltaRob(3)/2)/deltaRob(3);
    end
    XOdo(i,3) = XOdo(i-1,3) + deltaRob(3);

    % Sampling data given a certain distance
    if (sampling)
      deltaRobD = deltaRobD + sqrt(deltaRob(1)^2 + deltaRob(2)^2);

      if (deltaRobD >= sampleDist)
        deltaRobD = 0;
        iSamples  = [iSamples , i];
      end
    end
  end
  iSamples = [iSamples , i];
end