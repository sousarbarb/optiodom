function [RobotEstParam] = LinEtAl_omni3(RobotParam,XGt,Odo,iSamples)

  % Initialization: Robot parameters
  n  = RobotParam.ngear;
  Ce = RobotParam.encRes;
  L  = RobotParam.L;
  D  = RobotParam.D;

  % Initialization: Input data
  Nsim = length(XGt);

  % Process data: orientation data
  KTH = [];
  TH  = [];
  isegAll = 1;
  for i=1:Nsim
    Nseg = length(iSamples{i});
    for k=1:Nseg
      k1 = 1;
      k2 = iSamples{i}(k);
      KTH(isegAll,1) = sum( Odo{i}(1+k1:k2,1) )*pi*D(1)/(n*Ce);
      KTH(isegAll,2) = sum( Odo{i}(1+k1:k2,2) )*pi*D(2)/(n*Ce);
      KTH(isegAll,3) = sum( Odo{i}(1+k1:k2,3) )*pi*D(3)/(n*Ce);
      TH(isegAll)    = XGt{i}(k2,3) - XGt{i}(k1,3);
      isegAll = isegAll + 1;
    end
  end
  TH  = TH';
  JTH = (((KTH'*KTH)\KTH')*TH)';
  NsegAll = length(TH);

  % Process data: compute orientation of the robot
  RTH = cell(Nsim,1);
  for i=1:Nsim
    itf = length(XGt{i});
    RTH{i} = zeros(itf,1);
    RTH{i}(1) = XGt{i}(1,3);
    for k=2:itf
      RTH{i}(k) = RTH{i}(k-1) + JTH * ([ Odo{i}(k,1)*D(1) , Odo{i}(k,2)*D(2) , Odo{i}(k,3)*D(3) ]') * pi/(n*Ce);
    end
  end

  % Process data: position data
  KXY = [];
  XY  = [];
  for i=1:Nsim
    Nseg = length(iSamples{i});
    for k=1:Nseg
      k1 = 1;
      k2 = iSamples{i}(k);
      % X data
      KXY(isegAll,1) = sum(  cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll,2) = sum(  cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll,3) = sum(  cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      KXY(isegAll,4) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll,5) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll,6) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      XY(isegAll)    = XGt{i}(k2,1) - XGt{i}(k1,1);
      % Y data
      KXY(isegAll + NsegAll,1) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll + NsegAll,2) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll + NsegAll,3) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      KXY(isegAll + NsegAll,4) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll + NsegAll,5) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll + NsegAll,6) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      XY(isegAll + NsegAll)    = XGt{i}(k2,2) - XGt{i}(k1,2);
      % Update index
      isegAll = isegAll + 1;
    end
  end
  XY  = XY';
  KXY = [ (sqrt(3)*KXY(:,1)+KXY(:,4)).*D(1) , (-sqrt(3)*KXY(:,2)+KXY(:,5)).*D(2) , (0*KXY(:,3)+KXY(:,6)).*D(3) ] .*pi./(n.*Ce);
  JXY = (((KXY'*KXY)\KXY')*XY)';
  
  % Output arguments
  RobotEstParam = RobotParam;
  RobotEstParam.J = [ sqrt(3)*JXY(1) , -sqrt(3)*JXY(2) ,      0 ; ...
                              JXY(1) ,          JXY(2) , JXY(3) ; ...
                              JTH(1) ,          JTH(2) , JTH(3) ];
  RobotEstParam.Jinv = eye(3)/RobotEstParam.J;
end