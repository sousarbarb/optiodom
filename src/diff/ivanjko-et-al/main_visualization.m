% Suptitle parameters
suptitleParam.use = true;
suptitleParam.useFilename = true;
suptitleParam.height      = 0.04; % 0..1
suptitleParam.letterSize  = 11;



%% COMPARISON PATH : UNCALIBRATED VS CALIBRATED

% Position
for i=1:length(t)
  figure
  subplot(1,2,1)
  initialColorOrder = get(gca,'ColorOrder');
  xmin = min([ XOdo{i}(:,1) ; XGt{i}(:,1) ]);
  xmax = max([ XOdo{i}(:,1) ; XGt{i}(:,1) ]);
  ymin = min([ XOdo{i}(:,2) ; XGt{i}(:,2) ]);
  ymax = max([ XOdo{i}(:,2) ; XGt{i}(:,2) ]);
  xmin = xmin - 0.05*abs(xmax - xmin);
  xmax = xmax + 0.05*abs(xmax - xmin);
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(XOdo{i}(:,1),XOdo{i}(:,2))
  plot(XGt{i}( :,1),XGt{i}( :,2))
  if (~isempty(Method.sampleDist))
    plot(XOdo{i}( iSamples{i} ,1),XOdo{i}( iSamples{i} ,2),'x','Color',initialColorOrder(1,:))
    plot(XGt{i}(  iSamples{i} ,1),XGt{i}(  iSamples{i} ,2),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ xmin xmax ])
  ylim([ ymin ymax ])
  axis equal
  xlabel('x (m) \rightarrow')
  ylabel('y (m) \rightarrow')
  legend('(x_{o},y{o})','(x_{gt},y{gt})')
  title('uncalibrated')

  subplot(1,2,2)
  initialColorOrder = get(gca,'ColorOrder');
  xmin = min([ XOdoCal{i}(:,1) ; XGt{i}(:,1) ]);
  xmax = max([ XOdoCal{i}(:,1) ; XGt{i}(:,1) ]);
  ymin = min([ XOdoCal{i}(:,2) ; XGt{i}(:,2) ]);
  ymax = max([ XOdoCal{i}(:,2) ; XGt{i}(:,2) ]);
  xmin = xmin - 0.05*abs(xmax - xmin);
  xmax = xmax + 0.05*abs(xmax - xmin);
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(XOdoCal{i}(:,1),XOdoCal{i}(:,2))
  plot(XGt{i}(    :,1),XGt{i}(    :,2))
  if (~isempty(Method.sampleDist))
    plot(XOdoCal{i}( iSamples{i} ,1),XOdoCal{i}( iSamples{i} ,2),'x','Color',initialColorOrder(1,:))
    plot(XGt{i}(     iSamples{i} ,1),XGt{i}(     iSamples{i} ,2),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ xmin xmax ])
  ylim([ ymin ymax ])
  axis equal
  xlabel('x (m) \rightarrow')
  ylabel('y (m) \rightarrow')
  legend('(x_{o},y{o})','(x_{gt},y{gt})')
  title('calibrated')

  if (suptitleParam.use)
    axes( 'Position', [0,1-suptitleParam.height,1,suptitleParam.height] )
    set( gca, 'Color', 'None', 'XColor', 'None', 'YColor', 'None' )
    if (suptitleParam.useFilename)
      text( 0.5, 0, Filenames{i} , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    else
      text( 0.5, 0, sprintf('Experiment %02d',i) , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    end
  end
end

% Orientation
for i=1:length(t)
  figure
  subplot(1,2,1)
  initialColorOrder = get(gca,'ColorOrder');
  ymin = rad2deg(min(wrapToPi([ XOdo{i}(:,3) ; XGt{i}(:,3) ])));
  ymax = rad2deg(max(wrapToPi([ XOdo{i}(:,3) ; XGt{i}(:,3) ])));
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(t{i},rad2deg(wrapToPi(XOdo{i}(:,3))))
  plot(t{i},rad2deg(wrapToPi(XGt{i}( :,3))))
  if (~isempty(Method.sampleDist))
    plot(t{i}( iSamples{i} ),rad2deg(wrapToPi(XOdo{i}( iSamples{i} ,3))),'x','Color',initialColorOrder(1,:))
    plot(t{i}( iSamples{i} ),rad2deg(wrapToPi(XGt{i}(  iSamples{i} ,3))),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ 0    t{i}(end) ])
  ylim([ ymin ymax      ])
  xlabel('time (s) \rightarrow')
  ylabel('\theta (º) \rightarrow')
  legend('\theta_{o}','\theta_{gt}')
  title('uncalibrated')

  subplot(1,2,2)
  initialColorOrder = get(gca,'ColorOrder');
  ymin = rad2deg(min(wrapToPi([ XOdoCal{i}(:,3) ; XGt{i}(:,3) ])));
  ymax = rad2deg(max(wrapToPi([ XOdoCal{i}(:,3) ; XGt{i}(:,3) ])));
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(t{i},rad2deg(wrapToPi(XOdoCal{i}(:,3))))
  plot(t{i},rad2deg(wrapToPi(XGt{i}(:,3))))
  if (~isempty(Method.sampleDist))
    plot(t{i}( iSamples{i} ),rad2deg(wrapToPi(XOdoCal{i}( iSamples{i} ,3))),'x','Color',initialColorOrder(1,:))
    plot(t{i}( iSamples{i} ),rad2deg(wrapToPi(XGt{i}(     iSamples{i} ,3))),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ 0    t{i}(end) ])
  ylim([ ymin ymax      ])
  xlabel('time (s) \rightarrow')
  ylabel('\theta (º) \rightarrow')
  legend('\theta_{o}','\theta_{gt}')
  title('calibrated')

  if (suptitleParam.use)
    axes( 'Position', [0,1-suptitleParam.height,1,suptitleParam.height] )
    set( gca, 'Color', 'None', 'XColor', 'None', 'YColor', 'None' )
    if (suptitleParam.useFilename)
      text( 0.5, 0, Filenames{i} , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    else
      text( 0.5, 0, sprintf('Experiment %02d',i) , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    end
  end
end

%% COMPARISON ERROR: UNCALIBRATED VS CALIBRATED

for i=1:length(t)

  % Distance
  
  figure
  subplot(1,2,1)
  initialColorOrder = get(gca,'ColorOrder');
  ymin = min([ sqrt( XErr{i}(:,1).^2 + XErr{i}(:,2).^2 ) ; sqrt( XErrCal{i}(:,1).^2 + XErrCal{i}(:,2).^2 ) ]);
  ymax = max([ sqrt( XErr{i}(:,1).^2 + XErr{i}(:,2).^2 ) ; sqrt( XErrCal{i}(:,1).^2 + XErrCal{i}(:,2).^2 ) ]);
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(t{i},sqrt( XErr{i}(:,1).^2 + XErr{i}(:,2).^2 ))
  plot(t{i},sqrt( XErrCal{i}(:,1).^2 + XErrCal{i}(:,2).^2 ))
  if (~isempty(Method.sampleDist))
    plot(t{i}( iSamples{i} ),sqrt( XErr{i}(    iSamples{i} ,1).^2 + XErr{i}(    iSamples{i} ,2).^2 ),'x','Color',initialColorOrder(1,:))
    plot(t{i}( iSamples{i} ),sqrt( XErrCal{i}( iSamples{i} ,1).^2 + XErrCal{i}( iSamples{i} ,2).^2 ),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ 0    t{i}(end) ])
  ylim([ ymin ymax      ])
  xlabel('time (s) \rightarrow')
  ylabel('\epsilon_d (m) \rightarrow')
  legend('\epsilon_{d,unc}','\epsilon_{d,cal}')
  title('Distance error: uncalibrated vs calibrated')

  % Orientation

  subplot(1,2,2)
  initialColorOrder = get(gca,'ColorOrder');
  ymin = rad2deg(min(wrapToPi([ XErr{i}(:,3) ; XErrCal{i}(:,3) ])));
  ymax = rad2deg(max(wrapToPi([ XErr{i}(:,3) ; XErrCal{i}(:,3) ])));
  ymin = ymin - 0.05*abs(ymax - ymin);
  ymax = ymax + 0.05*abs(ymax - ymin);
  hold on
  plot(t{i},rad2deg(XErr{i}(:,3)))
  plot(t{i},rad2deg(XErrCal{i}(:,3)))
  if (~isempty(Method.sampleDist))
    plot(t{i}( iSamples{i} ),rad2deg(XErr{i}(    iSamples{i} ,3)),'x','Color',initialColorOrder(1,:))
    plot(t{i}( iSamples{i} ),rad2deg(XErrCal{i}( iSamples{i} ,3)),'x','Color',initialColorOrder(2,:))
  end
  grid on
  xlim([ 0    t{i}(end) ])
  ylim([ ymin ymax      ])
  xlabel('time (s) \rightarrow')
  ylabel('\epsilon_{\theta} (º) \rightarrow')
  legend('\epsilon_{\theta,unc}','\epsilon_{\theta,cal}')
  title('Orientation error: uncalibrated vs calibrated')

  if (suptitleParam.use)
    axes( 'Position', [0,1-suptitleParam.height,1,suptitleParam.height] )
    set( gca, 'Color', 'None', 'XColor', 'None', 'YColor', 'None' )
    if (suptitleParam.useFilename)
      text( 0.5, 0, Filenames{i} , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    else
      text( 0.5, 0, sprintf('Experiment %02d',i) , ...
      'FontSize', suptitleParam.letterSize', 'FontWeight', 'Bold', ...
      'HorizontalAlignment', 'Center', 'VerticalAlignment', 'Bottom', 'Interpreter', 'none' )
    end
  end
end

%% OPTIMIZATION PROCEDURE

if (strcmp(Method.name,'sousa'))
  figure
  subplot(1,2,1)
  plot(Method.results.optimization.cost,'r')
  grid on
  xlim([ 1 , Method.results.optimization.numIterations ])
  xlabel('iteration \rightarrow')
  ylabel('\epsilon_{RSS} \rightarrow')
  title('Cost function')
  subplot(1,2,2)
  plot(Method.results.optimization.costSim)
  grid on
  xlim([ 1 , Method.results.optimization.numIterations ])
  xlabel('iteration \rightarrow')
  ylabel('\epsilon_{RSS,s} \rightarrow')
  title('Cost of each path segment')
end