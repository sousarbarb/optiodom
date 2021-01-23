function [TimeData,XGt,Odo,XOdo,iTimeSync] = processOptiTrackData(filename,TypeEulerAngle,HRob,ThRobFrame,Odo,XOdo,TimeOdo,visualize)

  % Initialisation: Indexes
  ir = 7; ic = 0;        % Indexes for the data to retrive from the file
  icrot  = 2; icpos = 5; % Indexes for the rotation and position data of the rigid body in the file
  itime  = 1;            % Indexes for the timestamp
  icrot  = icrot  - ic + 1;
  icpos  = icpos  - ic + 1;
  itime  = itime  - ic + 1;

  % Initialisation: Evaluate the type of the Euler Angles
  if strcmp(TypeEulerAngle,'YXZ')
    irotX = 2;
    irotY = 1;
    irotZ = 3;
  elseif strcmp(TypeEulerAngle,'ZXY')
    irotX = 2;
    irotY = 3;
    irotZ = 1;
  end

  % Data Processing: Read CSV file
  M = csvread(filename,ir,ic);  % Data matrix retrieved from the csv file
  BodyRot = M(:,icrot:icrot+2); % Rotation data (Euler angles)
  BodyPos = M(:,icpos:icpos+2); % Position data
  [itf,~] = size(BodyRot);

  % Data Processing: Evaluate data occlusions
  BodyPosOld  = BodyPos;
  BodyRotOld  = BodyRot;
  [BodyPos] = removeDataOcclusionsByLinearApproximation(BodyPos);
  [BodyRot] = removeRotationDegDataOcclusionsByLinearApproximation(BodyRot);
  if (visualize)
    figure
    for i=1:3
      subplot(2,3,i)
      hold on
      plot(0:itf-1,BodyRotOld(:,i),'.-')
      plot(0:itf-1,BodyRot   (:,i),'.-')
      grid
      xlabel('time (frames) \rightarrow')
      if (i == irotX)
        ylabel('Rot. X (deg) \rightarrow')
      end
      if (i == irotY)
        ylabel('Rot. Y (deg) \rightarrow')
      end
      if (i == irotZ)
        ylabel('Rot. Z (deg) \rightarrow')
      end
      legend('old','new')
      subplot(2,3,i+3)
      hold on
      plot(0:itf-1,BodyPosOld(:,i),'.-')
      plot(0:itf-1,BodyPos   (:,i),'.-')
      xlabel('time (frames) \rightarrow')
      switch i
        case 1
          ylabel('x (m) \rightarrow')
        case 2
          ylabel('y (m) \rightarrow')
        case 3
          ylabel('z (m) \rightarrow')
      end
      grid
      legend('old','new')
    end
  end

  % Transform OptiTrak data to the robot logic center frame
  roty = @(th) [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
  RobotPos = zeros(size(BodyPos));
  RobotTh  = zeros(length(BodyRot(:,irotY)),1);
  for k=1:itf
    aux = [roty(deg2rad(BodyRot(k,irotY))) BodyPos(k,:)';zeros(1,3) 1] * HRob * [0 0 0 1]';
    RobotPos(k,:) = aux(1:3)';
    RobotTh(k,:)  = deg2rad(BodyRot(k,irotY)) + ThRobFrame;
  end
  RobotPos = ([roty(RobotTh(1))' -roty(RobotTh(1))'*RobotPos(1,:)';zeros(1,3) 1] * [RobotPos'; ones(1,length(RobotTh))])';
  RobotPos = RobotPos(:,1:3);
  RobotTh  = wrapToPi(wrapToPi(RobotTh) - wrapToPi(RobotTh(1)));

  % OptiTrack data
  TimeGt = M(:,itime) - M(1,itime);
  XGt    = [RobotPos(:,3) RobotPos(:,1) RobotTh(:,1)];

  % Get time instants for synchronization: Visualization
  figure
  subplot(1,2,1)
  hold on
  plot(TimeOdo,XOdo(:,1),'.-')
  plot(TimeOdo,XOdo(:,2),'.-')
  plot(TimeGt,XGt  (:,1),'.-')
  plot(TimeGt,XGt  (:,2),'.-')
  grid
  xlabel('time (s) \rightarrow')
  ylabel('{x,y} (m) \rightarrow')
  legend('x_{odo}','y_{odo}','x_{gt}','y_{gt}')
  subplot(1,2,2)
  hold on
  plot(TimeOdo,wrapTo180(rad2deg(XOdo(:,3))),'.-')
  plot(TimeGt,wrapTo180(rad2deg(XGt  (:,3))),'.-')
  grid
  xlabel('time (s) \rightarrow')
  ylabel('\theta (deg) \rightarrow')
  legend('\theta_{odo}','\theta_{gt}')

  % Get time instants for synchronization: Ask user
  tfGt  = TimeGt(end);
  tfOdo = TimeOdo(end);
  [itfGt ,~] = size(XGt);
  [itfOdo,~] = size(XOdo);
  prompt    = sprintf('Please insert the INITIAL time instant where the ground-truth data is synchronised (tsynci <= %d): ', tfGt);
  tsyncGt0  = input(prompt);
  prompt    = sprintf('Please insert the INITIAL time instant where the odometric    data is synchronised (tsynci <= %d): ', tfOdo);
  tsyncOdo0 = input(prompt);
  prompt    = sprintf('Please insert the FINAL   time instant where the ground-truth data is synchronised (tsyncf <= %d): ', tfGt);
  tsyncGtf  = input(prompt);
  prompt    = sprintf('Please insert the FINAL   time instant where the odometric    data is synchronised (tsyncf <= %d): ', tfOdo);
  tsyncOdof = input(prompt);
  tsyncGt0  = max([ tsyncGt0  , 0 ]);
  tsyncOdo0 = max([ tsyncOdo0 , 0 ]);
  tsyncGtf  = min([ tsyncGtf  , tfGt  ]);
  tsyncOdof = min([ tsyncOdof , tfOdo ]);

  % Data synchronization
  rotz = @(th) [cos(th) -sin(th);sin(th) cos(th)];

  % Data synchronization: Odometry data
  TOdo  = TimeOdo(2) - TimeOdo(1);
  iOdo0 = max([ round(tsyncOdo0/TOdo) + 1 , 0 ]);
  iOdof = min([ round(tsyncOdof/TOdo) + 1 , itfOdo ]);
  OdoSync  = Odo(iOdo0:iOdof,:);
  XOdoSync = XOdo(iOdo0:iOdof,:) - XOdo(iOdo0,:);

  % Data synchronization: Ground-truth data
  TGt  = TimeGt(2) - TimeGt(1);
  iGt0 = max([ round(tsyncGt0/TGt) + 1 , 0 ]);
  iGtf = min([ round(tsyncGtf/TGt) + 1 , itfGt ]);
  XGtSync    = XGt(iGt0:end,:);
  XGtSyncAux = (                                                   ...
    [                                                              ...
      rotz(XGtSync(1,3))' , -rotz(XGtSync(1,3))'*XGtSync(1,1:2)' ; ...
               zeros(1,2) ,                                    1   ...
    ] ...
    * ...
    [ ...
      XGtSync(:,1:2)'      ; ...
      ones(1,itfGt-iGt0+1)   ...
    ])';
  XGtSync(:,1:2) = XGtSyncAux(:,1:2);
  XGtSync(:,3)   = wrapToPi(wrapToPi(XGtSync(:,3)) - wrapToPi(XGtSync(1,3)));
  [itfGt,~] = size(XGtSync);
  for i=2:itfGt
    XGtSync(i,3) = XGtSync(i-1,3) + wrapToPi(wrapToPi(XGtSync(i,3)) - wrapToPi(XGtSync(i-1,3)));
  end
  timesync = round(linspace(0,tsyncGtf-tsyncGt0,length(OdoSync))./TGt) + 1;
  XGtSync  = XGtSync(timesync,:);

  % Output
  TimeData = ((0:iOdof-iOdo0).*TOdo)';
  XGt      = XGtSync;
  Odo      = OdoSync;
  XOdo     = XOdoSync;
  iTimeSync= [ tsyncGt0 , tsyncGtf , tsyncOdo0 , tsyncOdof ]

  % Visualization of the data alignment
  if (visualize)
    figure
    hold on
    plot(XOdo(:,1),XOdo(:,2),'.-')
    plot(XGt(:,1) ,XGt(:,2) ,'.-')
    grid
    axis equal
    xlabel('x \rightarrow')
    ylabel('y \rightarrow')
    legend('\{x_{odo},y_{odo}\}','\{x_{gt},y_{gt}\}')
    title('Robot Position')
    figure
    subplot(1,3,1)
    hold on
    plot(TimeData,XOdo(:,1),'.-')
    plot(TimeData,XGt(:,1),'.-')
    grid
    xlabel('time (s) \rightarrow')
    ylabel('x (m) \rightarrow')
    legend('x_{odo}','x_{gt}')
    title('Robot Position X')
    subplot(1,3,2)
    hold on
    plot(TimeData,XOdo(:,2),'.-')
    plot(TimeData,XGt(:,2),'.-')
    grid
    xlabel('time (s) \rightarrow')
    ylabel('y (m) \rightarrow')
    legend('y_{odo}','y_{gt}')
    title('Robot Position Y')
    subplot(1,3,3)
    hold on
    plot(TimeData,rad2deg(XOdo(:,3)),'.-')
    plot(TimeData,rad2deg(XGt(:,3)),'.-')
    grid
    xlabel('time (cycles) \rightarrow')
    ylabel('\theta (deg) \rightarrow')
    legend('\theta_{odo}','\theta_{gt}')
    title('Robot Orientation')
  end
end