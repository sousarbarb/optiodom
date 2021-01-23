close all
clear
clc

%% INITIALIZATION

visualize = true;

% Dataset filenames:
% - Kallasi et al: any dataset.
% - Sousa et al  : any dataset.
Dataset.filenames = {
   '../../../data/tricyc/circular/140120211415/140120211415',
   '../../../data/tricyc/circular/140120211440/140120211440',
   '../../../data/tricyc/circular/140120211454/140120211454',
   '../../../data/tricyc/circular/140120211513/140120211513',
   '../../../data/tricyc/circular/140120211519/140120211519',
   '../../../data/tricyc/circular/140120211530/140120211530',
   '../../../data/tricyc/circular/140120211617/140120211617',
   '../../../data/tricyc/square/140120211430/140120211430',
   '../../../data/tricyc/square/140120211446/140120211446',
   '../../../data/tricyc/square/140120211500/140120211500',
   '../../../data/tricyc/square/140120211536/140120211536',
   '../../../data/tricyc/square/140120211543/140120211543',
   '../../../data/tricyc/square/140120211554/140120211554',
   '../../../data/tricyc/square/140120211622/140120211622',
   '../../../data/tricyc/free/140120211508/140120211508',
   '../../../data/tricyc/free/140120211525/140120211525',
   '../../../data/tricyc/free/140120211606/140120211606',
   '../../../data/tricyc/free/140120211611/140120211611'
};
Dataset.metadata  = '../../../data/tricyc/circular/140120211415/140120211415_metadata.csv';
Dataset.N    = length(Dataset.filenames);
Dataset.data = cell(1,Dataset.N);

% Method parameters:
% some methods require specific parameters for their execution
Method.name       = 'kallasi';
Method.sampleDist = [0.5];
% - exclude worse runs
Method.options.excludeRuns = [];
% Method. ...

% Robot parameters:
[RobotParam] = readRobotParametersMetadata(Dataset.metadata);
% ... you can change the robot parameters after reading them from a metadata csv file
RobotParam.L = 0.15;

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
    
    [Dataset.data{i}.XOdo{j},Dataset.data{i}.iSamples{j}] = simulateRobot_tricyc( ...
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

%% METHOD: KALLASI ET AL.
% - Kallasi et al: any dataset.
% - Sousa et al  : any dataset.

% Evaluation measures
Method.results.uncalibrated = computeEvaluationMeasures(XErr);

% Calibration procedure
[ RobotEstParam ] = KallasiEtAl( ...
    RobotParam     , ...
    XGt            , ...
    Odo            , ...
    iSamples         ...
  );

%% SIMULATE CALIBRATED ROBOT

% Odometry data
k = 1;
auxExcludeRuns = Method.options.excludeRuns;
for i=1:Dataset.N
  for j=1:Dataset.data{i}.parameters.N    
    [Dataset.data{i}.XOdoCal{j},~] = simulateRobot_tricyc( ...
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