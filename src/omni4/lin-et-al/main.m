close all
clear
clc

%% INITIALIZATION

visualize = true;

% Dataset filenames:
% - Lin et al  : any dataset.
% - Sousa et al: any dataset.
Dataset.filenames = {
  '../../../data/omni4/circular/080920201015/080920201015',
  '../../../data/omni4/circular/080920201547/080920201547',
  '../../../data/omni4/circular/080920201911/080920201911',
  '../../../data/omni4/circular/231220200310/231220200310',
  '../../../data/omni4/circular/231220200346/231220200346',
  '../../../data/omni4/circular/231220200422/231220200422',
  '../../../data/omni4/circular/231220200440/231220200440',
  '../../../data/omni4/circular/231220200453/231220200453',
  '../../../data/omni4/circular/231220200502/231220200502',
  '../../../data/omni4/circular/231220200510/231220200510',
  '../../../data/omni4/square/080920201205/080920201205',
  '../../../data/omni4/square/080920201715/080920201715',
  '../../../data/omni4/square/080920201810/080920201810',
  '../../../data/omni4/square/231220200521/231220200521',
  '../../../data/omni4/square/231220200550/231220200550'
};
Dataset.metadata  = '../../../data/omni4/circular/231220200310/231220200310_metadata.csv';
Dataset.N    = length(Dataset.filenames);
Dataset.data = cell(1,Dataset.N);

% Method parameters:
% some methods require specific parameters for their execution
Method.name       = 'lin';
Method.sampleDist = [0.5];
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
    
    [Dataset.data{i}.XOdo{j},Dataset.data{i}.iSamples{j}] = simulateRobot_omni4( ...
      Dataset.data{i}.XGt{j}(1,:)                   , ...
      Dataset.data{i}.Odo{j}                        , ...
      Dataset.data{i}.parameters.Tsampling          , ...
      RobotParam                                    , ...
      Method.sampleDist,false);
    
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

%% METHOD: LIN ET AL.
% - Lin et al  : any dataset.
% - Sousa et al: any dataset.

% Evaluation measures
Method.results.uncalibrated = computeEvaluationMeasures(XErr);

% Calibration procedure
[ RobotEstParam ] = LinEtAl_omni4( ...
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
    [Dataset.data{i}.XOdoCal{j},~] = simulateRobot_omni4( ...
      Dataset.data{i}.XGt{j}(1,:)                      , ...
      Dataset.data{i}.Odo{j}                           , ...
      Dataset.data{i}.parameters.Tsampling             , ...
      RobotEstParam                                    , ...
      Method.sampleDist,true);
    
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