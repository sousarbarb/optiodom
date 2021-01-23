function [RobotEstParam] = KallasiEtAl(RobotParam,XGt,Odo,iSamples)

  % Initialization: Input data
  Nsim = length(XGt);

  % Process data
  A  = [];
  TH = [];
  isegAll = 1;
  for i=1:Nsim
    Nseg = length(iSamples{i});
    for k=1:Nseg
      if k ~= 1
        k1 = iSamples{i}(k-1);
      else
        k1 = 1;
      end
      k2 = iSamples{i}(k);
      A(isegAll,1) = sum( Odo{i}(1+k1:k2,1) .* sin(Odo{i}(1+k1:k2,2)) );
      A(isegAll,2) = sum( Odo{i}(1+k1:k2,1) .* cos(Odo{i}(1+k1:k2,2)) );
      TH(isegAll)  = XGt{i}(k2,3) - XGt{i}(k1,3);
      isegAll = isegAll + 1;
    end
  end
  TH = TH';

  % Monroe-Penrose pseudoinverse
  P = ((A'*A)\A')*TH;

  % Output arguments
  RobotEstParam = RobotParam;
  RobotEstParam.ThOff = atan2(P(2),P(1));
  RobotEstParam.D     = RobotEstParam.L * sqrt( P(1)^2 + P(2)^2 ) * RobotEstParam.ngear * RobotEstParam.encRes / pi;
end