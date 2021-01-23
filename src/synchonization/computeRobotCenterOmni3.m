function [HRob,ThRobFrame,imarkers] = computeRobotCenterOmni3(filenameCentre,filenameRob,TypeEulerAngle,visualize)

  %% INITIALISATION
  strRigidBodyMarker = "Rigid Body Marker";
  
  %% DATA PROCESSING OF FILE FILENAMECENTRE '..._CENTRE.CSV'

  % Evaluate number of markers of the rigid body
  fileID = fopen(filenameCentre,'r');
  fgetl(fileID);             % Ignore 2 lines
  fgetl(fileID);
  line = fgetl(fileID);      % Process next line
  line = strsplit(line,',');
  countRigidBodyMarker = 0;
  for i=1:length(line)
    if strcmp(line{i},strRigidBodyMarker)
      countRigidBodyMarker = countRigidBodyMarker + 1;
    end
  end
  fclose(fileID);
  % Compute the number of markers of the rigid body (division by 4, because 
  % each marker ID is repeated 4 times in the line that was processed)
  countRigidBodyMarker = (countRigidBodyMarker/4);
  countRigidBodyMarker = round(countRigidBodyMarker);

  % Initialisation
  ir = 7; ic = 9;
  M = csvread(filenameCentre,ir,ic);
  [tf,~] = size(M);
  Marker = zeros(tf,3,countRigidBodyMarker);
  % Markers from the robot
  imarker = 1;
  legMarker = cell(1,countRigidBodyMarker);
  for i=1:countRigidBodyMarker
    Marker(:,:,i) = M(:,imarker:imarker+2);                 % Get position data relative to the marker i
    [r,~] = find(Marker(:,:,i) == 0);                       % Find samples relative to occlusion or data lose
    r = unique(r);
    Marker(r,:,i) = (Marker(r-1,:,i) + Marker(r+1,:,i))/2;  % Perform a average between the previous sample and the next
    imarker = imarker + 4;                                  % Update index of the marker
    legMarker{i} = sprintf("Marker_%d",i);
  end
  % Plot original indexes of the markers
  figure
  hold on
  for i=1:countRigidBodyMarker
  plot(Marker(:,3,i),Marker(:,1,i),"x")
  text(sum(Marker(:,3,i))/tf,sum(Marker(:,1,i))/tf,sprintf("\\leftarrow (%d)",i))
  end
  grid
  axis equal
  xlabel('x (m) \rightarrow')
  ylabel('y (m) \rightarrow')
  legend(legMarker)
  title('Original Markers of rigid body ROBOTCENTRE')
  % Confirm with the user the standard for the markers indexes
  im    = zeros(1,3);
  prompt = "\nRobot markers on the wheels shatf standard:\n\n                x (2)\n               /       y\n             ...       |\n             /         |\n(3) x--...--<          |____x\n             \\ \n             ...\n               \\ \n                x (1)\n\n";
  fprintf(prompt);
  prompt = "Please insert the corresponding marker 1: ";
  im(1) = input(prompt);
  prompt = "Please insert the corresponding marker 2: ";
  im(2) = input(prompt);
  prompt = "Please insert the corresponding marker 3: ";
  im(3) = input(prompt);
  if (~visualize)
    close
  end
  % Compute the indexes of other markers
  imOther = zeros(1,countRigidBodyMarker-3);
  k=1;
  for i=1:countRigidBodyMarker
    if (i ~= im(1)) && (i ~= im(2)) && (i ~= im(3))
      imOther(k) = i;
      k=k+1;
    end
  end
  % Change the markes indexes to the standard
  MarkerAux = zeros(size(Marker));
  for i=1:3                                     % Update the standard markers for the wheels shat markers
    MarkerAux(:,:,i) = Marker(:,:,im(i));
  end
  k=1;
  for i=4:countRigidBodyMarker                  % Update the other markers
    MarkerAux(:,:,i) = Marker(:,:,imOther(k));
    k=k+1;
  end
  Marker = MarkerAux;
  clear MarkerAux
  % Plot modified indexes of the markers
  if (visualize)
    figure
    hold on
    for i=1:countRigidBodyMarker
    plot(Marker(:,3,i),Marker(:,1,i),"x")
    text(sum(Marker(:,3,i))/tf,sum(Marker(:,1,i))/tf,sprintf('\\leftarrow (%d)',i))
    end
    grid
    axis equal
    xlabel('x (m) \rightarrow')
    ylabel('y (m) \rightarrow')
    legend(legMarker)
    title("Modified Markers of rigid body ROBOTCENTRE")
  end
  
  %% DATA PROCESSING OF FILE FILENAMEOMNI3 '..._BODY.CSV'
  
  % Initialisation
  ir = 7; ic = 2;
  icrot = 2; icrot = icrot - ic + 1;
  icpos = 5; icpos = icpos - ic + 1;
  
  % Evaluate which type of Euler Angles the file has
  if strcmp(TypeEulerAngle,"YXZ")
    irotY = 1;
  elseif strcmp(TypeEulerAngle,"ZXY")
    irotY = 3;
  end
  
  % Open file
  M = csvread(filenameRob,ir,ic);
  BodyRot = removeDataOcclusionsByLinearApproximation(M(:,icrot:icrot+2));
  BodyPos = removeDataOcclusionsByLinearApproximation(M(:,icpos:icpos+2));

  % Compute center of gravity of robot
  rigidbody_omni_cg = [ sum(BodyPos(:,1))/length(BodyPos(:,1)) , sum(BodyPos(:,2))/length(BodyPos(:,2)) , sum(BodyPos(:,3))/length(BodyPos(:,3)) ];

  % Compute the centre of gravity relative to each marker
  MarkerCG = zeros(3,countRigidBodyMarker);
  for i=1:countRigidBodyMarker
    MarkerCG(:,i) = [ sum(Marker(:,1,i))/tf , sum(Marker(:,2,i))/tf , sum(Marker(:,3,i))/tf ]';
  end

  % Compute the orientation of the rigid body ROBOTOMNI
  theta_y = atan2( sum(sin(deg2rad(BodyRot(:,irotY))))/length(BodyRot(:,irotY)) , sum(cos(deg2rad(BodyRot(:,irotY))))/length(BodyRot(:,irotY)) );

  % Display robot centre of gravity
  if (visualize)
    figure
    hold on
    plot(BodyPos(:,3), BodyPos(:,1),"x")
    plot(rigidbody_omni_cg(3),rigidbody_omni_cg(1),"x")
    grid
    axis equal
    xlabel('x (m) \rightarrow')
    ylabel('y (m) \rightarrow')
    legend("\{x,z\}_{diff,i}','c.g._{diff,i}")
  end

  %% ROBOT LOGIC CENTRE
  robotlogiccentre = ( MarkerCG(:,1) + MarkerCG(:,2) + MarkerCG(:,3) ) / 3;
  theta_robotworld = atan2( robotlogiccentre(1) - MarkerCG(1,3) , robotlogiccentre(3) - MarkerCG(3,3) );
  ThRobFrame = theta_robotworld - theta_y;
  translation_vec  = [ robotlogiccentre(1)-rigidbody_omni_cg(1) , robotlogiccentre(2)-rigidbody_omni_cg(2) , robotlogiccentre(3)-rigidbody_omni_cg(3) ]';
  
  % Display robot with center of gravity and the markers
  if (visualize)
    figure
    hold on
    for i=1:countRigidBodyMarker
    plot(Marker(:,3,i),Marker(:,1,i),"x")
    text(sum(Marker(:,3,i))/tf,sum(Marker(:,1,i))/tf,sprintf("\\leftarrow (%d)",i))
    end
    plot(rigidbody_omni_cg(3),rigidbody_omni_cg(1),"o")
    plot(robotlogiccentre(3),robotlogiccentre(1),"d")
    pointsRigidBodyOmni3 = [];
    for i=1:countRigidBodyMarker
      pointsRigidBodyOmni3 = [ pointsRigidBodyOmni3 ; rigidbody_omni_cg(3) rigidbody_omni_cg(1) ];
      pointsRigidBodyOmni3 = [ pointsRigidBodyOmni3 ; MarkerCG(3,i)        MarkerCG(1,i)        ];
    end
    plot(pointsRigidBodyOmni3(:,1),pointsRigidBodyOmni3(:,2),"Color", [150/255 150/255 150/255],"LineStyle","-")
    plot([rigidbody_omni_cg(3)+0.05*cos(theta_y),rigidbody_omni_cg(3),rigidbody_omni_cg(3)-0.05*sin(theta_y)],...
         [rigidbody_omni_cg(1)+0.05*sin(theta_y),rigidbody_omni_cg(1),rigidbody_omni_cg(1)+0.05*cos(theta_y)],...
          "Color", [0 0 0])
    plot([MarkerCG(3,1),MarkerCG(3,2),MarkerCG(3,3),MarkerCG(3,1)],...
         [MarkerCG(1,1),MarkerCG(1,2),MarkerCG(1,3),MarkerCG(1,1)],...
          "Color", [150/255 150/255 150/255],"LineStyle","-")
    plot([robotlogiccentre(3),MarkerCG(3,1),robotlogiccentre(3),MarkerCG(3,2),robotlogiccentre(3),MarkerCG(3,3),robotlogiccentre(3)],...
         [robotlogiccentre(1),MarkerCG(1,1),robotlogiccentre(1),MarkerCG(1,2),robotlogiccentre(1),MarkerCG(1,3),robotlogiccentre(1)],...
          "Color", [150/255 150/255 150/255],"LineStyle","-")
    plot([robotlogiccentre(3)+0.05*cos(theta_robotworld),robotlogiccentre(3),robotlogiccentre(3)-0.05*sin(theta_robotworld)],...
         [robotlogiccentre(1)+0.05*sin(theta_robotworld),robotlogiccentre(1),robotlogiccentre(1)+0.05*cos(theta_robotworld)],...
          "Color", [0 0 0])
    grid
    axis equal
    xlabel('z \rightarrow')
    ylabel('x \rightarrow')
    legMarker{countRigidBodyMarker+1} = "c.g._{diff}";
    legMarker{countRigidBodyMarker+2} = "RobotCentre";
    legend(legMarker)
  end

  %% OUTPUT TRANSFORMATION
  roty = @(th) [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)];
  HRob = [roty(ThRobFrame) translation_vec; zeros(1,3) 1];
  imarkers = im;
end