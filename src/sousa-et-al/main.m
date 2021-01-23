close all
clear
clc

%% INITIALIZATION

visualize = true;

% Dataset filenames:
Dataset.filenames = {

  %%%%% >>>>>  DIFF  <<<<< %%%%%

  '../../data/diff/square/230620202042/230620202042',
  '../../data/diff/square/230620202144/230620202144',
  '../../data/diff/square/230620202258/230620202258',
  '../../data/diff/square/230620202317/230620202317',
  '../../data/diff/square/231220200029/231220200029',
  '../../data/diff/square/231220200040/231220200040',
  '../../data/diff/square/231220200045/231220200045',
  '../../data/diff/square/231220200048/231220200048',
  '../../data/diff/circular/231220200121/231220200121',
  '../../data/diff/circular/231220200134/231220200134',
  '../../data/diff/circular/231220200141/231220200141',
  '../../data/diff/circular/231220200146/231220200146',
  '../../data/diff/circular/231220200150/231220200150',
  '../../data/diff/circular/231220200154/231220200154',
  '../../data/diff/circular/231220200157/231220200157',
  '../../data/diff/circular/250620202104/250620202104',
  '../../data/diff/circular/250620202119/250620202119',
  '../../data/diff/circular/250620202236/250620202236',
  '../../data/diff/circular/250620202251/250620202251',
  '../../data/diff/circular/250620202317/250620202317',
  '../../data/diff/circular/250620202345/250620202345',
  '../../data/diff/free/020120212354/020120212354',
  '../../data/diff/free/030120210001/030120210001',
  '../../data/diff/free/030120210006/030120210006',
  '../../data/diff/ivanjko/231220200057/231220200057',
  '../../data/diff/ivanjko/231220200102/231220200102',
  '../../data/diff/ivanjko/231220200104/231220200104',
  '../../data/diff/ivanjko/231220200107/231220200107',
  '../../data/diff/ivanjko/250620201618/250620201618',
  '../../data/diff/ivanjko/250620201636/250620201636',
  '../../data/diff/ivanjko/250620201655/250620201655',
  '../../data/diff/ivanjko/250620201738/250620201738',

%   %%%%% >>>>> TRICYC <<<<< %%%%%
%
%   '../../data/tricyc/circular/140120211415/140120211415',
%   '../../data/tricyc/circular/140120211440/140120211440',
%   '../../data/tricyc/circular/140120211454/140120211454',
%   '../../data/tricyc/circular/140120211513/140120211513',
%   '../../data/tricyc/circular/140120211519/140120211519',
%   '../../data/tricyc/circular/140120211530/140120211530',
%   '../../data/tricyc/circular/140120211617/140120211617',
%   '../../data/tricyc/square/140120211430/140120211430',
%   '../../data/tricyc/square/140120211446/140120211446',
%   '../../data/tricyc/square/140120211500/140120211500',
%   '../../data/tricyc/square/140120211536/140120211536',
%   '../../data/tricyc/square/140120211543/140120211543',
%   '../../data/tricyc/square/140120211554/140120211554',
%   '../../data/tricyc/square/140120211622/140120211622',
%   '../../data/tricyc/free/140120211508/140120211508',
%   '../../data/tricyc/free/140120211525/140120211525',
%   '../../data/tricyc/free/140120211606/140120211606',
%   '../../data/tricyc/free/140120211611/140120211611',
% 
%   %%%%% >>>>> OMNI3  <<<<< %%%%%
% 
%   '../../data/omni3/circular/221220201643/221220201643',
%   '../../data/omni3/circular/221220201701/221220201701',
%   '../../data/omni3/circular/221220201716/221220201716',
%   '../../data/omni3/circular/221220201722/221220201722',
%   '../../data/omni3/circular/221220201726/221220201726',
%   '../../data/omni3/circular/221220201730/221220201730',
%   '../../data/omni3/circular/221220201750/221220201750',
%   '../../data/omni3/square/221220201934/221220201934',
%   '../../data/omni3/square/221220201953/221220201953',
%   '../../data/omni3/joystick/211220201842/211220201842',
%   '../../data/omni3/joystick/221220202228/221220202228',
%   '../../data/omni3/joystick/221220202235/221220202235',
% 
%   %%%%% >>>>> OMNI4  <<<<< %%%%%
% 
%   '../../data/omni4/circular/080920201015/080920201015',
%   '../../data/omni4/circular/080920201547/080920201547',
%   '../../data/omni4/circular/080920201911/080920201911',
%   '../../data/omni4/circular/231220200310/231220200310',
%   '../../data/omni4/circular/231220200346/231220200346',
%   '../../data/omni4/circular/231220200422/231220200422',
%   '../../data/omni4/circular/231220200440/231220200440',
%   '../../data/omni4/circular/231220200453/231220200453',
%   '../../data/omni4/circular/231220200502/231220200502',
%   '../../data/omni4/circular/231220200510/231220200510',
%   '../../data/omni4/square/080920201205/080920201205',
%   '../../data/omni4/square/080920201715/080920201715',
%   '../../data/omni4/square/080920201810/080920201810',
%   '../../data/omni4/square/231220200521/231220200521',
%   '../../data/omni4/square/231220200550/231220200550',
};
Dataset.metadata  = '../../data/diff/square/230620202042/230620202042_metadata.csv';
% Dataset.metadata  = '../../data/tricyc/circular/140120211415/140120211415_metadata.csv';
% Dataset.metadata  = '../../data/omni3/circular/221220201643/221220201643_metadata.csv';
% Dataset.metadata  = '../../data/omni4/circular/231220200310/231220200310_metadata.csv';
Dataset.N    = length(Dataset.filenames);
Dataset.data = cell(1,Dataset.N);

% Robot parameters:
[RobotParam] = readRobotParametersMetadata(Dataset.metadata);
% ... you can change the robot parameters after reading them from a metadata csv file
RobotParam.useJ = false;
% RobotParam.L = [];
% RobotParam.D = [];
% RobotParam.ThOff = [];

% Method parameters:
% some methods require specific parameters for their execution
Method.name       = 'sousa';
Method.sampleDist = [0.5];
% - rprop parameters
[Method] = setRpropParameters(Method,RobotParam);
% - exclude worse runs
Method.options.excludeRuns = [30];
% Method. ...


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
    
    [Dataset.data{i}.XOdo{j},Dataset.data{i}.iSamples{j}] = simulateRobot( ...
      Dataset.data{i}.XGt{j}(1,:)          , ...
      Dataset.data{i}.Odo{j}               , ...
      Dataset.data{i}.parameters.Tsampling , ...
      RobotParam                           , ...
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

% Evaluation measures
Method.results.uncalibrated = computeEvaluationMeasures(XErr);

% Calibration procedure
[ RobotEstParam                       , ...
  Method.results.optimization.cost    , ...
  Method.results.optimization.costSim , ...
  Method.results.optimization.numIterations    , ...
  Method.results.optimization.historyRobotParam] = Rprop( ...
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
    [Dataset.data{i}.XOdoCal{j},~] = simulateRobot( ...
      Dataset.data{i}.XGt{j}(1,:)                 , ...
      Dataset.data{i}.Odo{j}                      , ...
      Dataset.data{i}.parameters.Tsampling        , ...
      RobotEstParam                               , ...
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