function [XOdo,iSamples] = simulateRobot(X0,Odo,TOdo,RobotParam,sampleDist)

  % Select the appropriate function given a robot's type
  if     (strcmp(RobotParam.type,'diff'))

    [XOdo,iSamples] = simulateRobot_diff(X0,Odo,TOdo,RobotParam,sampleDist);

  elseif (strcmp(RobotParam.type,'tricyc'))

    [XOdo,iSamples] = simulateRobot_tricyc(X0,Odo,TOdo,RobotParam,sampleDist);

  elseif (strcmp(RobotParam.type,'omni3'))

    [XOdo,iSamples] = simulateRobot_omni3(X0,Odo,TOdo,RobotParam,sampleDist);

  elseif (strcmp(RobotParam.type,'omni4'))

    [XOdo,iSamples] = simulateRobot_omni4(X0,Odo,TOdo,RobotParam,sampleDist);

  end
end