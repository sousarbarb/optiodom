function [RobotEstParam] = LinEtAl_omni4(RobotParam,XGt,Odo,iSamples)

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
      KTH(isegAll,4) = sum( Odo{i}(1+k1:k2,4) )*pi*D(4)/(n*Ce);
      TH(isegAll)    = XGt{i}(k2,3) - XGt{i}(k1,3);
      isegAll = isegAll + 1;
    end
  end
  TH  = TH';
  %KTH = KTH(:,1) + KTH(:,2) + KTH(:,3) + KTH(:,4);
  JTH = (((KTH'*KTH)\KTH')*TH)';
  %JTH = [ JTH , JTH , JTH , JTH ];
  NsegAll = length(TH);

  % Process data: compute orientation of the robot
  RTH = cell(Nsim,1);
  for i=1:Nsim
    itf = length(XGt{i});
    RTH{i} = zeros(itf,1);
    RTH{i}(1) = XGt{i}(1,3);
    for k=2:itf
      RTH{i}(k) = RTH{i}(k-1) + JTH * ([ Odo{i}(k,1)*D(1) , Odo{i}(k,2)*D(2) , Odo{i}(k,3)*D(3) , Odo{i}(k,4)*D(4) ]') * pi/(n*Ce);
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
      KXY(isegAll,4) = sum(  cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,4) );
      KXY(isegAll,5) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll,6) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll,7) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      KXY(isegAll,8) = sum( -sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,4) );
      XY(isegAll)    = XGt{i}(k2,1) - XGt{i}(k1,1);
      % Y data
      KXY(isegAll + NsegAll,1) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll + NsegAll,2) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll + NsegAll,3) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      KXY(isegAll + NsegAll,4) = sum( sin(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,4) );
      KXY(isegAll + NsegAll,5) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,1) );
      KXY(isegAll + NsegAll,6) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,2) );
      KXY(isegAll + NsegAll,7) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,3) );
      KXY(isegAll + NsegAll,8) = sum( cos(RTH{i}(1+k1:k2)) .* Odo{i}(1+k1:k2,4) );
      XY(isegAll + NsegAll)    = XGt{i}(k2,2) - XGt{i}(k1,2);
      % Update index
      isegAll = isegAll + 1;
    end
  end
  XY  = XY';
  KXY = [ (KXY(:,1)-KXY(:,5)).*D(1) , (KXY(:,2)+KXY(:,6)).*D(2) , (KXY(:,3)+KXY(:,7)).*D(3) , (KXY(:,4)-KXY(:,8)).*D(4) ] .*pi./(n.*Ce);
  JXY = (((KXY'*KXY)\KXY')*XY)';
  
  % Output arguments
  RobotEstParam = RobotParam;
  RobotEstParam.J = [  JXY(1) , JXY(2) , JXY(3) ,  JXY(4) ; ...
                      -JXY(1) , JXY(2) , JXY(3) , -JXY(4) ; ...
                       JTH(1) , JTH(2) , JTH(3) ,  JTH(4) ];
  RobotEstParam.Jinv = RobotEstParam.J'/(RobotEstParam.J*RobotEstParam.J');
end