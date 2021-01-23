function [DataMatrix] = removeRotationDegDataOcclusionsByLinearApproximation(DataMatrix)

  % Initialization
  boolMissing = false;        % Assumes that the initial value is not missing
  [itf,~] = size(DataMatrix); % Final frame
  
  % Linearization of the data occlusions
  for i=1:itf
    % Detect the first sample of the data occlusion (in OptiTrack, the data
    % occlusion retrieves 0 for all fields when using CSVREAD)
    if sum((DataMatrix(i,:) == 0))  && (~boolMissing)
      boolMissing = true;
      iDataOkIni  = i-1; 
    end
    % Detect the first valid sample after the data occlusion has occurred
    if prod((DataMatrix(i,:) ~= 0)) && (boolMissing)
      boolMissing = false;
      iDataOkFin  = i;
      DataOkIni   = DataMatrix(iDataOkIni,:);
      DataOkFin   = wrapTo180(DataOkIni) + wrapTo180(DataMatrix(iDataOkFin,:)-DataOkIni);
      % Linearisation
      for k=iDataOkIni+1:iDataOkFin-1
        DataMatrix(k,:) = wrapTo180(wrapTo180(DataMatrix(k-1,:)) + wrapTo180((DataOkFin-DataOkIni)/(iDataOkFin-iDataOkIni)));
      end
    end
  end
end

