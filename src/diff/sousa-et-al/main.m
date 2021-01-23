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
  '../../../data/diff/square/230620202042/230620202042',
  '../../../data/diff/square/230620202144/230620202144',
  '../../../data/diff/square/230620202258/230620202258',
  '../../../data/diff/square/230620202317/230620202317',
  '../../../data/diff/square/231220200029/231220200029',
  '../../../data/diff/square/231220200040/231220200040',
  '../../../data/diff/square/231220200045/231220200045',
  '../../../data/diff/square/231220200048/231220200048',
  '../../../data/diff/circular/231220200121/231220200121',
  '../../../data/diff/circular/231220200134/231220200134',
  '../../../data/diff/circular/231220200141/231220200141',
  '../../../data/diff/circular/231220200146/231220200146',
  '../../../data/diff/circular/231220200150/231220200150',
  '../../../data/diff/circular/231220200154/231220200154',
  '../../../data/diff/circular/231220200157/231220200157',
  '../../../data/diff/circular/250620202104/250620202104',
  '../../../data/diff/circular/250620202119/250620202119',
  '../../../data/diff/circular/250620202236/250620202236',
  '../../../data/diff/circular/250620202251/250620202251',
  '../../../data/diff/circular/250620202317/250620202317',
  '../../../data/diff/circular/250620202345/250620202345',
  '../../../data/diff/free/020120212354/020120212354',
  '../../../data/diff/free/030120210001/030120210001',
  '../../../data/diff/free/030120210006/030120210006',
  '../../../data/diff/ivanjko/231220200057/231220200057',
  '../../../data/diff/ivanjko/231220200102/231220200102',
  '../../../data/diff/ivanjko/231220200104/231220200104',
  '../../../data/diff/ivanjko/231220200107/231220200107',
  '../../../data/diff/ivanjko/250620201618/250620201618',
  '../../../data/diff/ivanjko/250620201636/250620201636',
  '../../../data/diff/ivanjko/250620201655/250620201655',
  '../../../data/diff/ivanjko/250620201738/250620201738'
};
Dataset.metadata  = '../../../data/diff/square/230620202042/230620202042_metadata.csv';
Dataset.N    = length(Dataset.filenames);
Dataset.data = cell(1,Dataset.N);

% Method parameters:
% some methods require specific parameters for their execution
Method.name       = 'sousa';
Method.sampleDist = [0.5];
% - options:
%   > type         : 'rprop+', 'rprop-', 'irprop+', 'irprop-'
%   > nplus        : rprop acceleration parameter
%   > nminus       : rprop deceleration parameter
%   > maxvar       : maximum parameters variation
%   > minvar       : minimum parameters variation
%   > maxiter      : maximum iterations
%   > miniter      : minimum iterations
%   > minvarbetiter: minimum variation to break algorithm
%   > numiterminavg: number of iterations to compute the average
Method.options.type   = 'irprop-';
Method.options.nplus  = [ 1.005 , 1.005 , 1.005 ];
Method.options.nminus = [ 0.500 , 0.500 , 0.500 ];
Method.options.maxvar = [ 0.00025 , 0.00025 , 0.00025 ];
Method.options.minvar = [ 0.00001 , 0.00001 , 0.00001 ];
Method.options.maxiter = 1000;
Method.options.miniter = 30;
Method.options.minvarbetiter = 0.001;
Method.options.numiterminavg = 10;
% - exclude worse runs
Method.options.excludeRuns = [];
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
iSamples = {};
Filenames = {};

%% DATA PROCESSMENT
k = 1;
auxExcludeRuns = Method.options.excludeRuns;
for i=1:Dataset.N
  Dataset.data{i}.parameters = readDatasetParameters(strcat(Dataset.filenames{i},'_metadata.csv'));

  for j=1:Dataset.data{i}.parameters.N

    [ Dataset.data{i}.parameters.Tsampling , ...
      Dataset.data{i}.numSamples{j}        , ...
      Dataset.data{i}.time{j} , ...
      Dataset.data{i}.XGt{j}  , ...
      Dataset.data{i}.Odo{j}  ] = loadData(strcat(Dataset.filenames{i},sprintf('_run-%02d.csv', j)),RobotParam);
    
    [Dataset.data{i}.XOdo{j},Dataset.data{i}.iSamples{j}] = simulateRobot_diff( ...
      Dataset.data{i}.XGt{j}(1,:)                   , ...
      Dataset.data{i}.Odo{j}                        , ...
      Dataset.data{i}.parameters.Tsampling          , ...
      RobotParam                                    , ...
      Method.sampleDist);
    
    Dataset.data{i}.XErr{j} = Dataset.data{i}.XGt{j} - Dataset.data{i}.XOdo{j};
    
    % Agregated data
    if (~isempty(auxExcludeRuns))
      if (sum(k == auxExcludeRuns) > 0)
        auxExcludeRuns (k == auxExcludeRuns) = [];
        auxExcludeRuns = auxExcludeRuns - 1;
        runok = false;
      else
        runok = true;
      end
    else
      runok = true;
    end
    if (runok)
      Filenames{k} = strcat(Dataset.filenames{i},sprintf('_run-%02d.csv', j));
      t{k}    = Dataset.data{i}.time{j};
      XGt{k}  = Dataset.data{i}.XGt{j};
      Odo{k}  = Dataset.data{i}.Odo{j};
      XOdo{k} = Dataset.data{i}.XOdo{j};
      XErr{k} = Dataset.data{i}.XErr{j};
      iSamples{k} = Dataset.data{i}.iSamples{j};
      k = k+1;
    end
  end
end

%% METHOD: SOUSA ET AL.
% - UMBmark and Jung&Chung: assumed that it the only dataset it is first CW (clockwise) and then CCW (counterclockwise).
% - Ivanjko               : any dataset of Ivanjko's path (straight line + 180º on-the-spot rotation).
% - Sousa et al           : any dataset.

% Evaluation measures
Method.results.uncalibrated = computeEvaluationMeasures(XErr);

% Calibration procedure
[ RobotEstParam                       , ...
  Method.results.optimization.cost    , ...
  Method.results.optimization.costSim , ...
  Method.results.optimization.numIterations    , ...
  Method.results.optimization.historyRobotParam] = Rprop_diff( ...
    RobotParam     , ...
    XGt            , ...
    Odo            , ...
    iSamples       , ...
    Method.options   ...
  );

%% SIMULATE CALIBRATED ROBOT

% Odometry data
k = 1;
auxExcludeRuns = Method.options.excludeRuns;
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
    if (~isempty(auxExcludeRuns))
      if (sum(k == auxExcludeRuns) > 0)
        auxExcludeRuns (k == auxExcludeRuns) = [];
        auxExcludeRuns = auxExcludeRuns - 1;
        runok = false;
      else
        runok = true;
      end
    else
      runok = true;
    end
    if (runok)
      XOdoCal{k} = Dataset.data{i}.XOdoCal{j};
      XErrCal{k} = Dataset.data{i}.XErrCal{j};
      k = k+1;
    end
  end
end

% Evaluation measures
Method.results.calibrated = computeEvaluationMeasures(XErrCal);


%% VISUALIZATION
main_visualizationSimplex

if (visualize)
  main_visualization
end