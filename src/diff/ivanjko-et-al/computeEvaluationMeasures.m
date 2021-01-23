function [measures] = computeEvaluationMeasures(XErr)

  % Initialisation
  N = length(XErr);
  measures.cgX  = 0;          % Center of gravity of the final X error
  measures.cgY  = 0;          % Center of gravity of the final Y error
  measures.cgTH = zeros(1,2); % Center of gravity of the final orientation error
  measures.cgR  = 0;          % Distance of the final X and Y error's center of gravities
  measures.R    = 0;          % Maximum distance variation on the final pose error (gives an estimation of the nonsystematic errors' influence [UMBmark])
  measures.maxXErr    = zeros(N,2); % Maximum absolute error over the dataset
  measures.maxFinXErr = zeros(N,2); % Final absolute pose (distance and orientation) error over the dataset
  for i=1:N
    measures.cgX    = measures.cgX  + XErr{i}(end,1);
    measures.cgY    = measures.cgY  + XErr{i}(end,2);
    measures.cgTH   = measures.cgTH + [ cos(XErr{i}(end,3)) , sin(XErr{i}(end,3)) ];
    measures.R(i,:) = sqrt( (XErr{i}(end,1) - measures.cgX).^2 + (XErr{i}(end,2) - measures.cgY).^2 );
    measures.maxXErr(i,:)    = [ max(sqrt( XErr{i}(:,1).^2 + XErr{i}(:,2).^2 )) , max(abs(wrapToPi(XErr{i}(:,3)))) ];
    measures.maxFinXErr(i,:) = [ sqrt( XErr{i}(end,1).^2 + XErr{i}(end,2).^2 )  , abs(wrapToPi(XErr{i}(end,3))) ];
  end
  measures.cgX  = measures.cgX / N;
  measures.cgY  = measures.cgY / N;
  measures.cgTH = wrapToPi( atan2(measures.cgTH(2),measures.cgTH(1)) / N );
  measures.cgR  = sqrt(measures.cgX^2 + measures.cgY^2);
  measures.R    = max(measures.R); % Indicates the non-systematic error!!! (a "perspective" of this error)
  measures.maxXErrRun = measures.maxXErr;
  measures.maxXErr    = [ max(measures.maxXErr(:,1))    , max(measures.maxXErr(:,2)) ];
  measures.maxFinXErrRun = measures.maxFinXErr;
  measures.maxFinXErr    = [ max(measures.maxFinXErr(:,1)) , max(measures.maxFinXErr(:,2)) ];
end