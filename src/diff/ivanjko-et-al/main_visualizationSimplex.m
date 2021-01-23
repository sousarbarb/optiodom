RobotEstParam
Method.results.uncalibrated
Method.results.calibrated

if     (strcmp(RobotParam.type,'diff'))

  fprintf('UNCALIBRATED:\n  param: %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\nCALIBRATED:\n  param: %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\n',RobotParam.L,RobotParam.D(1),RobotParam.D(2),Method.results.uncalibrated.maxXErr(1),rad2deg(Method.results.uncalibrated.maxXErr(2)),Method.results.uncalibrated.maxFinXErr(1),rad2deg(Method.results.uncalibrated.maxFinXErr(2)),RobotEstParam.L,RobotEstParam.D(1),RobotEstParam.D(2),Method.results.calibrated.maxXErr(1),rad2deg(Method.results.calibrated.maxXErr(2)),Method.results.calibrated.maxFinXErr(1),rad2deg(Method.results.calibrated.maxFinXErr(2)));

elseif (strcmp(RobotParam.type,'tricyc'))

  fprintf('UNCALIBRATED:\n  param: %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\nCALIBRATED:\n  param: %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\n',RobotParam.L,RobotParam.D,rad2deg(RobotParam.ThOff),Method.results.uncalibrated.maxXErr(1),rad2deg(Method.results.uncalibrated.maxXErr(2)),Method.results.uncalibrated.maxFinXErr(1),rad2deg(Method.results.uncalibrated.maxFinXErr(2)),RobotEstParam.L,RobotEstParam.D,rad2deg(RobotEstParam.ThOff),Method.results.calibrated.maxXErr(1),rad2deg(Method.results.calibrated.maxXErr(2)),Method.results.calibrated.maxFinXErr(1),rad2deg(Method.results.calibrated.maxFinXErr(2)));

elseif (strcmp(RobotParam.type,'omni3'))

  fprintf('UNCALIBRATED:\n  param: %.6f %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\nCALIBRATED:\n  param: %.6f %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\n',RobotParam.L,RobotParam.D(1),RobotParam.D(2),RobotParam.D(3),Method.results.uncalibrated.maxXErr(1),rad2deg(Method.results.uncalibrated.maxXErr(2)),Method.results.uncalibrated.maxFinXErr(1),rad2deg(Method.results.uncalibrated.maxFinXErr(2)),RobotEstParam.L,RobotEstParam.D(1),RobotEstParam.D(2),RobotEstParam.D(3),Method.results.calibrated.maxXErr(1),rad2deg(Method.results.calibrated.maxXErr(2)),Method.results.calibrated.maxFinXErr(1),rad2deg(Method.results.calibrated.maxFinXErr(2)));

elseif (strcmp(RobotParam.type,'omni4'))

  fprintf('UNCALIBRATED:\n  param: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\nCALIBRATED:\n  param: %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n  max    : [ %.6f %.6f ]\n  max fin: [ %.6f %.6f ]\n',RobotParam.L(1),RobotParam.L(2),RobotParam.L(1)+RobotParam.L(2),RobotParam.D(1),RobotParam.D(2),RobotParam.D(3),RobotParam.D(4),Method.results.uncalibrated.maxXErr(1),rad2deg(Method.results.uncalibrated.maxXErr(2)),Method.results.uncalibrated.maxFinXErr(1),rad2deg(Method.results.uncalibrated.maxFinXErr(2)),RobotEstParam.L(1),RobotEstParam.L(2),RobotEstParam.L(1)+RobotEstParam.L(2),RobotEstParam.D(1),RobotEstParam.D(2),RobotEstParam.D(3),RobotEstParam.D(4),Method.results.calibrated.maxXErr(1),rad2deg(Method.results.calibrated.maxXErr(2)),Method.results.calibrated.maxFinXErr(1),rad2deg(Method.results.calibrated.maxFinXErr(2)));

end

% Visualization simplified (maximum errors per run)
[Nruns,~] = size(Method.results.uncalibrated.maxFinXErrRun);

figure
subplot(1,2,1)
hold on
bar([ Method.results.uncalibrated.maxXErrRun(:,1) , Method.results.calibrated.maxXErrRun(:,1) ])
grid on
xlim([0,Nruns+1])
xlabel('irun \rightarrow')
ylabel('max \epsilon_d (m)')
legend('max \epsilon_{d,unc}','max \epsilon_{d,cal}')
for i=1:Nruns
  %text(i,0,
  text(i,max([ Method.results.uncalibrated.maxXErrRun(:,1) ; Method.results.calibrated.maxXErrRun(:,1)])*1.5,...
    extractAfter(Filenames{i},RobotParam.type),...
    'HorizontalAlignment','left',              ...
    'VerticalAlignment','middle',              ...
    'Rotation',45,                             ...
    'Interpreter','none',                      ...
    'FontSize',9)
end
title('Maximum Distance Error')
subplot(1,2,2)
hold on
bar([ rad2deg(Method.results.uncalibrated.maxXErrRun(:,2)) , rad2deg(Method.results.calibrated.maxXErrRun(:,2)) ])
grid on
xlim([0,Nruns+1])
xlabel('irun \rightarrow')
ylabel('max \epsilon_{\theta} (º)')
legend('max \epsilon_{\theta,unc}','max \epsilon_{\theta,cal}')
for i=1:Nruns
  %text(i,0,
  text(i,max([ rad2deg(Method.results.uncalibrated.maxXErrRun(:,2)) ; rad2deg(Method.results.calibrated.maxXErrRun(:,2)) ])*1.5,...
    extractAfter(Filenames{i},RobotParam.type),...
    'HorizontalAlignment','left',              ...
    'VerticalAlignment','middle',              ...
    'Rotation',45,                             ...
    'Interpreter','none',                      ...
    'FontSize',9)
end
title('Maximum Absolute Orientation Error')

figure
subplot(1,2,1)
hold on
bar([ Method.results.uncalibrated.maxFinXErrRun(:,1) , Method.results.calibrated.maxFinXErrRun(:,1) ])
grid on
xlim([0,Nruns+1])
xlabel('irun \rightarrow')
ylabel('max \epsilon_d (m)')
legend('max \epsilon_{d,unc}','max \epsilon_{d,cal}')
for i=1:Nruns
  %text(i,0,
  text(i,max([ Method.results.uncalibrated.maxFinXErrRun(:,1) ; Method.results.calibrated.maxFinXErrRun(:,1)])*1.5,...
    extractAfter(Filenames{i},RobotParam.type),...
    'HorizontalAlignment','left',              ...
    'VerticalAlignment','middle',              ...
    'Rotation',45,                             ...
    'Interpreter','none',                      ...
    'FontSize',9)
end
title('Maximum Final Distance Error')
subplot(1,2,2)
hold on
bar([ rad2deg(Method.results.uncalibrated.maxFinXErrRun(:,2)) , rad2deg(Method.results.calibrated.maxFinXErrRun(:,2)) ])
grid on
xlim([0,Nruns+1])
xlabel('irun \rightarrow')
ylabel('max \epsilon_{\theta} (º)')
legend('max \epsilon_{\theta,unc}','max \epsilon_{\theta,cal}')
for i=1:Nruns
  %text(i,0,
  text(i,max([ rad2deg(Method.results.uncalibrated.maxFinXErrRun(:,2)) ; rad2deg(Method.results.calibrated.maxFinXErrRun(:,2)) ])*1.5,...
    extractAfter(Filenames{i},RobotParam.type),...
    'HorizontalAlignment','left',              ...
    'VerticalAlignment','middle',              ...
    'Rotation',45,                             ...
    'Interpreter','none',                      ...
    'FontSize',9)
end
title('Maximum Final Absolute Orientation Error')