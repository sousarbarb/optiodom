close all
clear
clc

%% INITIALIZATION

visualize = true;

% Dataset filenames:
% - UMBmark and Jung&Chung: only 1 dataset of square paths.
% - Ivanjko               : only 1 dataset of Ivanjko's path (straight line + 180º on-the-spot rotation).
% - Sousa et al           : any dataset.
Dataset.filenames = {
  %'../../../data/diff/ivanjko/231220200057/231220200057',
  %'../../../data/diff/ivanjko/231220200102/231220200102',
  %'../../../data/diff/ivanjko/231220200104/231220200104',
  %'../../../data/diff/ivanjko/231220200107/231220200107',
  %'../../../data/diff/ivanjko/250620201618/250620201618',
  %'../../../data/diff/ivanjko/250620201636/250620201636',
  %'../../../data/diff/ivanjko/250620201655/250620201655',
  '../../../data/diff/ivanjko/250620201738/250620201738'
};
Dataset.metadata  = '../../../data/diff/ivanjko/231220200057/231220200057_metadata.csv';
Dataset.N    = length(Dataset.filenames);
Dataset.data = cell(1,Dataset.N);

% Method parameters:
% some methods require specific parameters for their execution
Method.name       = 'ivanjko';
Method.sampleDist = [];
Method.options = optimoptions(@lsqnonlin,  ... Non-linear least-squares
  'Algorithm','levenberg-marquardt',... Levenberg-Marquardt Algorithm
  'MaxFunctionEvaluations',100000,  ... Adjust these two parameters to 
  'MaxIterations',10000             ... obtain performance @ accuracy
);
% Method. ...

% Robot parameters:
[RobotParam] = readRobotParametersMetadata(Dataset.metadata);
% ... you can change the robot parameters after reading them from a metadata csv file

% Agregated data
t    = {};
Odo  = {};
XOdo = {};
XGt  = {};
XErr = {};
XOdoCal = {};
XErrCal = {};
Filenames = {};

%% DATA PROCESSMENT
k = 1;
for i=1:Dataset.N
  Dataset.data{i}.parameters = readDatasetParameters(strcat(Dataset.filenames{i},'_metadata.csv'));

  for j=1:Dataset.data{i}.parameters.N
    Filenames{k} = strcat(Dataset.filenames{i},sprintf('_run-%02d.csv', j));

    [ Dataset.data{i}.parameters.Tsampling , ...
      Dataset.data{i}.numSamples{j}        , ...
      Dataset.data{i}.time{j} , ...
      Dataset.data{i}.XGt{j}  , ...
      Dataset.data{i}.Odo{j}  ] = loadData(Filenames{k},RobotParam);
    
    [Dataset.data{i}.XOdo{j},~] = simulateRobot_diff( ...
      Dataset.data{i}.XGt{j}(1,:)                   , ...
      Dataset.data{i}.Odo{j}                        , ...
      Dataset.data{i}.parameters.Tsampling          , ...
      RobotParam                                    , ...
      Method.sampleDist);
    
    Dataset.data{i}.XErr{j} = Dataset.data{i}.XGt{j} - Dataset.data{i}.XOdo{j};
    
    % Agregated data
    t{k}    = Dataset.data{i}.time{j};
    XGt{k}  = Dataset.data{i}.XGt{j};
    Odo{k}  = Dataset.data{i}.Odo{j};
    XOdo{k} = Dataset.data{i}.XOdo{j};
    XErr{k} = Dataset.data{i}.XErr{j};
    k = k+1;
  end
end

%% METHOD: IVANJKO ET AL.
% - UMBmark and Jung&Chung: assumed that it the only dataset it is first CW (clockwise) and then CCW (counterclockwise).
% - Ivanjko               : any dataset of Ivanjko's path (straight line + 180º on-the-spot rotation).
% - Sousa et al           : any dataset.

% Evaluation measures
N = length(t)/3;                                 % N runs straight-line, clockwise (CW) and N runs counterclockwise (CCW)
XErrLin = cellarray2cellarray(XErr,1    ,N  );
XErrCw  = cellarray2cellarray(XErr,1+N  ,N*2);
XErrCcw = cellarray2cellarray(XErr,1+N*2,N*3);
Method.results.uncalibrated     = computeEvaluationMeasures(XErr);
Method.results.uncalibrated.lin = computeEvaluationMeasures(XErrLin);
Method.results.uncalibrated.cw  = computeEvaluationMeasures(XErrCw);
Method.results.uncalibrated.ccw = computeEvaluationMeasures(XErrCcw);

% Calibration procedure
RobotEstParam = RobotParam;
OdoLin = cellarray2cellarray(Odo,1    ,N  );
OdoCw  = cellarray2cellarray(Odo,1+N  ,N*2);
OdoCcw = cellarray2cellarray(Odo,1+N*2,N*3);
XGtLin = cellarray2cellarray(XGt,1    ,N  );
XGtCw  = cellarray2cellarray(XGt,1+N  ,N*2);
XGtCcw = cellarray2cellarray(XGt,1+N*2,N*3);
while true
  % Initial estimations
  KD0 = [1 1]; %
  KL0 = [1];

  % 1: Optimise wheels parameters
  KD = lsqnonlin(@optimizeKD,KD0,[],[],Method.options,XGtLin,OdoLin,RobotEstParam);
  RobotEstParam.D = KD .* RobotEstParam.D;
  if (abs(KD(1)-1) < 0.001) && (abs(KD(2)-1) < 0.001)
    conditionKD = true;
  else
    conditionKD = false;
  end

  % 2: Optimise wheelbase/distance between wheels
  KLCw  = lsqnonlin(@optimizeKL,KL0,[],[],Method.options,XGtCw ,OdoCw ,RobotEstParam);
  KLCcw = lsqnonlin(@optimizeKL,KL0,[],[],Method.options,XGtCcw,OdoCcw,RobotEstParam);
  KL    = (KLCw + KLCcw)/2;
  RobotEstParam.L(1) = KL * RobotEstParam.L(1);
  if (abs(KL-1) < 0.001)
    conditionKL = true;
  else
    conditionKL = false;
  end

  % Exit condition
  if conditionKD && conditionKL
    break;
  end
end

%% SIMULATE CALIBRATED ROBOT

% Odometry data
k = 1;
for i=1:Dataset.N
  for j=1:Dataset.data{i}.parameters.N    
    [Dataset.data{i}.XOdoCal{j},~] = simulateRobot_diff( ...
      Dataset.data{i}.XGt{j}(1,:)                      , ...
      Dataset.data{i}.Odo{j}                           , ...
      Dataset.data{i}.parameters.Tsampling             , ...
      RobotEstParam                                    , ...
      Method.sampleDist);
    
    Dataset.data{i}.XErrCal{j} = Dataset.data{i}.XGt{j} - Dataset.data{i}.XOdoCal{j};
    
    % Agregated data
    XOdoCal{k} = Dataset.data{i}.XOdoCal{j};
    XErrCal{k} = Dataset.data{i}.XErrCal{j};
    k = k+1;
  end
end

% Evaluation measures
XErrCalLin = cellarray2cellarray(XErrCal,1    ,N  );
XErrCalCw  = cellarray2cellarray(XErrCal,1+N  ,N*2);
XErrCalCcw = cellarray2cellarray(XErrCal,1+N*2,N*3);
Method.results.calibrated     = computeEvaluationMeasures(XErrCal);
Method.results.calibrated.lin = computeEvaluationMeasures(XErrCalLin);
Method.results.calibrated.cw  = computeEvaluationMeasures(XErrCalCw);
Method.results.calibrated.ccw = computeEvaluationMeasures(XErrCalCcw);


%% VISUALIZATION
main_visualizationSimplex

if (visualize)
  main_visualization
end