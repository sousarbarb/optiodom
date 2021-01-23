function [Tsampling,numSamples,time,XGt,Odo] = loadData(filename,RobotParam)

  % Get data from csv file
  dataMat = readmatrix(filename);

  % Data processing
  % - sampling period:
  Tsampling   = dataMat(2,1) - dataMat(1,1);
  % - number of samples:
  [numSamples,~] = size(dataMat);
  % - time:
  time = [0:numSamples-1] * Tsampling;
  % - ground-truth data:
  XGt = dataMat(:,2:4);
  % - odometry data:
  Odo = dataMat(:,5:end);
end