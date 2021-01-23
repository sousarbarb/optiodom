function [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop(RobotParam,XGt,Odo,iSamples,options)

  % Select the appropriate function given a robot's type
  if     (strcmp(RobotParam.type,'diff'))

    [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_diff(RobotParam,XGt,Odo,iSamples,options);

  elseif (strcmp(RobotParam.type,'tricyc'))

    [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_tricyc(RobotParam,XGt,Odo,iSamples,options);

  elseif (strcmp(RobotParam.type,'omni3'))

    [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_omni3(RobotParam,XGt,Odo,iSamples,options);

  elseif (strcmp(RobotParam.type,'omni4'))

    [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_omni4(RobotParam,XGt,Odo,iSamples,options);

  end
end