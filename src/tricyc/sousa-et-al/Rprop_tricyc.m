function [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_tricyc(RobotParam,XGt,Odo,iSamples,options)

  % Initialization: Robot parameters
  n  = RobotParam.ngear;
  Ce = RobotParam.encRes;
  L  = RobotParam.L;
  D  = RobotParam.D;
  ThOff  = RobotParam.ThOff;

  % Initialization: Input data
  Nsim = length(XGt);

  % Initialization: Resilient propagation parameters
  rproptype = options.type;
  nplus  = options.nplus;
  nminus = options.nminus;
  maxvar = options.maxvar;
  minvar = options.minvar;
  maxiter = options.maxiter;
  miniter = options.miniter;
  minvarbetiter = options.minvarbetiter;
  numiterminavg = options.numiterminavg;

  % Initialization: Auxiliar variables (to improve the algorithm's performance)
  a_pi_nCe = pi/(n*Ce);
  a_Wh = cell(Nsim,1);
  itf  = zeros(Nsim,1);
  for i=1:Nsim
    [itf(i),~] = size(Odo{i});
    a_Wh{i}    = zeros(itf(i),2);
  end
  nWh = 1;
  nL  = length(L);
  nTh = 1;

  % Initialization: Procedure parameters
  %XOdoDebug = XGt;
  XOdoF = zeros(Nsim,3);
  cost    = zeros(maxiter,1);
  costSim = zeros(maxiter,1);
  costDeriv = zeros(1,3);        % [ dE/dL dE/dD(1) dE/dD(2) ] <=> [ dE/dL dE/dD_R dE/dD_L ]
  historyRobotParam = zeros(maxiter,nL+nWh+nTh);

  % Initialization: Parameters variation
  newdeltaD = minvar;
  newdeltaW = zeros(1,nL+nWh+nTh);

  % RPROP ALGORITHM
  iteration = 1;
  if strcmp(rproptype,'rprop+')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D , ThOff ];

      % Parameters initialization
      derivParam = zeros(Nsim,6);  % [ dx/dL dx/dD dx/dThOff dy/dL dy/dD dy/dThOff ]
      costDeriv  = zeros(1,3);     % [ dE/dL dE/dD dE/dThOff ]
      
      isegAll = 1;

      % Common variables
      for i=1:Nsim
        for k=2:itf(i)
          a_Wh{i}(k,1) = sum( Odo{i}(2:k-1,1) .* cos( Odo{i}(2:k-1,2) + ThOff ) );
          a_Wh{i}(k,2) = sum( Odo{i}(2:k-1,1) .* sin( Odo{i}(2:k-1,2) + ThOff ) );
        end
      end

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          ThWh    = Odo{i}(k,2);
          deltaD  = pi*D*Odo{i}(k,1)*cos(ThWh+ThOff)/(n*Ce);
          deltaTH = pi*D*Odo{i}(k,1)*sin(ThWh+ThOff)/(n*Ce*L);
          a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
          a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );

          % X and Y
          XOdoF(i,1) = XOdoF(i,1) + deltaD * a_cosThDth2;
          XOdoF(i,2) = XOdoF(i,2) + deltaD * a_sinThDth2;

          % Compute derivatives
          derivParam(i,1) = derivParam(i,1) + deltaD * a_sinThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,2) = derivParam(i,2)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,3) = derivParam(i,3)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;
          derivParam(i,4) = derivParam(i,4) - deltaD * a_cosThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,5) = derivParam(i,5)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,6) = derivParam(i,6)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;

          % TH
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2);

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,6);
            a_derivParam(1) = derivParam(i,1) / L;
            a_derivParam(2) = derivParam(i,2) * a_pi_nCe;
            a_derivParam(3) = derivParam(i,3) * D * a_pi_nCe;
            a_derivParam(4) = derivParam(i,4) / L;
            a_derivParam(5) = derivParam(i,5) * a_pi_nCe;
            a_derivParam(6) = derivParam(i,6) * D * a_pi_nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(4);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);

            % Cost function
            costSim(iteration,isegAll) = ( a_XOdoF(1) - XGt{i}(k,1) )^2 + ( a_XOdoF(2) - XGt{i}(k,2) )^2;
            cost(iteration)            = cost(iteration) + costSim(iteration,isegAll);

            % Update segment
            iseg    = iseg    + 1;
            isegAll = isegAll + 1;
          end

          % Debug
          %XOdoDebug{i}(k,:) = XOdoF(i,:);
        end
      end

      % >>>>> RPROP+: Resilient Propagation with Weight-backtracking
      for j=1:nL+nWh+nTh
        if ( costDeriv(j)*oldcostDeriv(j) > 0 )
          newdeltaD(j) =  min([ olddeltaD(j) * nplus(j) , maxvar(j) ]);
          newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
        elseif ( costDeriv(j)*oldcostDeriv(j) < 0 )
          newdeltaD(j) =  max([ olddeltaD(j) * nminus(j) , minvar(j) ]);
          newdeltaW(j) = -olddeltaW(j);
          costDeriv(j) = 0;
        else
          newdeltaD(j) =  olddeltaD(j);
          newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
        end
      end
      newL     = L     + newdeltaW(1:nL);
      newD     = D     + newdeltaW(nL+1:nL+nWh);
      newThOff = ThOff + newdeltaW(nL+nWh+1:nL+nWh+nTh);
      
      % Stop condition
      % 1: maximum iterations
      if (iteration >= maxiter)
        break;
      end
      % 2: minimum variation between consecutive iterations
      if (iteration >= miniter)
        avg    = sum(cost(iteration-  numiterminavg+1:iteration              ))/numiterminavg;
        avgOld = sum(cost(iteration-2*numiterminavg+1:iteration-numiterminavg))/numiterminavg;
        if ( abs( (avg - avgOld)/avgOld ) < minvarbetiter )
          break;
        end
      end

      % Update number of iterations
      iteration = iteration + 1;

      % Update robot parameters estimation
      L = newL;
      D = newD;
      ThOff = newThOff;

      toc
    end
  elseif strcmp(rproptype,'rprop-')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D , ThOff ];

      % Parameters initialization
      derivParam = zeros(Nsim,6);  % [ dx/dL dx/dD dx/dThOff dy/dL dy/dD dy/dThOff ]
      costDeriv  = zeros(1,3);     % [ dE/dL dE/dD dE/dThOff ]
      
      isegAll = 1;

      % Common variables
      for i=1:Nsim
        for k=2:itf(i)
          a_Wh{i}(k,1) = sum( Odo{i}(2:k-1,1) .* cos( Odo{i}(2:k-1,2) + ThOff ) );
          a_Wh{i}(k,2) = sum( Odo{i}(2:k-1,1) .* sin( Odo{i}(2:k-1,2) + ThOff ) );
        end
      end

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          ThWh    = Odo{i}(k,2);
          deltaD  = pi*D*Odo{i}(k,1)*cos(ThWh+ThOff)/(n*Ce);
          deltaTH = pi*D*Odo{i}(k,1)*sin(ThWh+ThOff)/(n*Ce*L);
          a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
          a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );

          % X and Y
          XOdoF(i,1) = XOdoF(i,1) + deltaD * a_cosThDth2;
          XOdoF(i,2) = XOdoF(i,2) + deltaD * a_sinThDth2;

          % Compute derivatives
          derivParam(i,1) = derivParam(i,1) + deltaD * a_sinThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,2) = derivParam(i,2)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,3) = derivParam(i,3)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;
          derivParam(i,4) = derivParam(i,4) - deltaD * a_cosThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,5) = derivParam(i,5)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,6) = derivParam(i,6)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;

          % TH
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2);

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,6);
            a_derivParam(1) = derivParam(i,1) / L;
            a_derivParam(2) = derivParam(i,2) * a_pi_nCe;
            a_derivParam(3) = derivParam(i,3) * D * a_pi_nCe;
            a_derivParam(4) = derivParam(i,4) / L;
            a_derivParam(5) = derivParam(i,5) * a_pi_nCe;
            a_derivParam(6) = derivParam(i,6) * D * a_pi_nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(4);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);

            % Cost function
            costSim(iteration,isegAll) = ( a_XOdoF(1) - XGt{i}(k,1) )^2 + ( a_XOdoF(2) - XGt{i}(k,2) )^2;
            cost(iteration)            = cost(iteration) + costSim(iteration,isegAll);

            % Update segment
            iseg    = iseg    + 1;
            isegAll = isegAll + 1;
          end

          % Debug
          %XOdoDebug{i}(k,:) = XOdoF(i,:);
        end
      end

      % >>>>> RPROP-: Resilient Propagation without Wheight-backtracking
      for j=1:nL+nWh+nTh
        if ( costDeriv(j)*oldcostDeriv(j) > 0 )
          newdeltaD(j) = min([ olddeltaD(j) * nplus(j) , maxvar(j) ]);
        elseif ( costDeriv(j)*oldcostDeriv(j) < 0 )
          newdeltaD(j) = max([ olddeltaD(j) * nminus(j) , minvar(j) ]);
        else
          newdeltaD(j) = olddeltaD(j);
        end
        newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
      end
      newL     = L     + newdeltaW(1:nL);
      newD     = D     + newdeltaW(nL+1:nL+nWh);
      newThOff = ThOff + newdeltaW(nL+nWh+1:nL+nWh+nTh);
      
      % Stop condition
      % 1: maximum iterations
      if (iteration >= maxiter)
        break;
      end
      % 2: minimum variation between consecutive iterations
      if (iteration >= miniter)
        avg    = sum(cost(iteration-  numiterminavg+1:iteration              ))/numiterminavg;
        avgOld = sum(cost(iteration-2*numiterminavg+1:iteration-numiterminavg))/numiterminavg;
        if ( abs( (avg - avgOld)/avgOld ) < minvarbetiter )
          break;
        end
      end

      % Update number of iterations
      iteration = iteration + 1;

      % Update robot parameters estimation
      L = newL;
      D = newD;
      ThOff = newThOff;

      toc
    end
  elseif strcmp(rproptype,'irprop+')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D , ThOff ];

      % Parameters initialization
      derivParam = zeros(Nsim,6);  % [ dx/dL dx/dD dx/dThOff dy/dL dy/dD dy/dThOff ]
      costDeriv  = zeros(1,3);     % [ dE/dL dE/dD dE/dThOff ]
      
      isegAll = 1;

      % Common variables
      for i=1:Nsim
        for k=2:itf(i)
          a_Wh{i}(k,1) = sum( Odo{i}(2:k-1,1) .* cos( Odo{i}(2:k-1,2) + ThOff ) );
          a_Wh{i}(k,2) = sum( Odo{i}(2:k-1,1) .* sin( Odo{i}(2:k-1,2) + ThOff ) );
        end
      end

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          ThWh    = Odo{i}(k,2);
          deltaD  = pi*D*Odo{i}(k,1)*cos(ThWh+ThOff)/(n*Ce);
          deltaTH = pi*D*Odo{i}(k,1)*sin(ThWh+ThOff)/(n*Ce*L);
          a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
          a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );

          % X and Y
          XOdoF(i,1) = XOdoF(i,1) + deltaD * a_cosThDth2;
          XOdoF(i,2) = XOdoF(i,2) + deltaD * a_sinThDth2;

          % Compute derivatives
          derivParam(i,1) = derivParam(i,1) + deltaD * a_sinThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,2) = derivParam(i,2)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,3) = derivParam(i,3)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;
          derivParam(i,4) = derivParam(i,4) - deltaD * a_cosThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,5) = derivParam(i,5)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,6) = derivParam(i,6)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;

          % TH
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2);

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,6);
            a_derivParam(1) = derivParam(i,1) / L;
            a_derivParam(2) = derivParam(i,2) * a_pi_nCe;
            a_derivParam(3) = derivParam(i,3) * D * a_pi_nCe;
            a_derivParam(4) = derivParam(i,4) / L;
            a_derivParam(5) = derivParam(i,5) * a_pi_nCe;
            a_derivParam(6) = derivParam(i,6) * D * a_pi_nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(4);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);

            % Cost function
            costSim(iteration,isegAll) = ( a_XOdoF(1) - XGt{i}(k,1) )^2 + ( a_XOdoF(2) - XGt{i}(k,2) )^2;
            cost(iteration)            = cost(iteration) + costSim(iteration,isegAll);

            % Update segment
            iseg    = iseg    + 1;
            isegAll = isegAll + 1;
          end

          % Debug
          %XOdoDebug{i}(k,:) = XOdoF(i,:);
        end
      end

      % >>>>> IRPROP+: Improved Resilient Propagation with Weight-backtracking
      for j=1:nL+nWh+nTh
        if ( costDeriv(j)*oldcostDeriv(j) > 0 )
          newdeltaD(j) =  min([ olddeltaD(j) * nplus(j) , maxvar(j) ]);
          newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
        elseif ( costDeriv(j)*oldcostDeriv(j) < 0 )
          newdeltaD(j) =  max([ olddeltaD(j) * nminus(j) , minvar(j) ]);
          if (iteration ~= 1)
            if (cost(iteration) > cost(iteration-1))
              newdeltaW(j) = -olddeltaW(j);
            else
              newdeltaW(j) = 0;
            end
          else
            newdeltaW(j) = 0;
          end
          costDeriv(j) = 0;
        else
          newdeltaD(j) =  olddeltaD(j);
          newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
        end
      end
      newL     = L     + newdeltaW(1:nL);
      newD     = D     + newdeltaW(nL+1:nL+nWh);
      newThOff = ThOff + newdeltaW(nL+nWh+1:nL+nWh+nTh);
      
      % Stop condition
      % 1: maximum iterations
      if (iteration >= maxiter)
        break;
      end
      % 2: minimum variation between consecutive iterations
      if (iteration >= miniter)
        avg    = sum(cost(iteration-  numiterminavg+1:iteration              ))/numiterminavg;
        avgOld = sum(cost(iteration-2*numiterminavg+1:iteration-numiterminavg))/numiterminavg;
        if ( abs( (avg - avgOld)/avgOld ) < minvarbetiter )
          break;
        end
      end

      % Update number of iterations
      iteration = iteration + 1;

      % Update robot parameters estimation
      L = newL;
      D = newD;
      ThOff = newThOff;

      toc
    end
  elseif strcmp(rproptype,'irprop-')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D , ThOff ];

      % Parameters initialization
      derivParam = zeros(Nsim,6);  % [ dx/dL dx/dD dx/dThOff dy/dL dy/dD dy/dThOff ]
      costDeriv  = zeros(1,3);     % [ dE/dL dE/dD dE/dThOff ]
      
      isegAll = 1;

      % Common variables
      for i=1:Nsim
        for k=2:itf(i)
          a_Wh{i}(k,1) = sum( Odo{i}(2:k-1,1) .* cos( Odo{i}(2:k-1,2) + ThOff ) );
          a_Wh{i}(k,2) = sum( Odo{i}(2:k-1,1) .* sin( Odo{i}(2:k-1,2) + ThOff ) );
        end
      end

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          ThWh    = Odo{i}(k,2);
          deltaD  = pi*D*Odo{i}(k,1)*cos(ThWh+ThOff)/(n*Ce);
          deltaTH = pi*D*Odo{i}(k,1)*sin(ThWh+ThOff)/(n*Ce*L);
          a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
          a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );

          % X and Y
          XOdoF(i,1) = XOdoF(i,1) + deltaD * a_cosThDth2;
          XOdoF(i,2) = XOdoF(i,2) + deltaD * a_sinThDth2;

          % Compute derivatives
          derivParam(i,1) = derivParam(i,1) + deltaD * a_sinThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,2) = derivParam(i,2)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,3) = derivParam(i,3)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_cosThDth2 - ...
                            deltaD * a_sinThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;
          derivParam(i,4) = derivParam(i,4) - deltaD * a_cosThDth2 * ( XOdoF(i,3) - XGt{i}(1,3) + deltaTH/2 );
          derivParam(i,5) = derivParam(i,5)                               + ...
                            Odo{i}(k,1) * cos(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,2) + Odo{i}(k,1)*sin(ThWh+ThOff)/2 ) / L;
          derivParam(i,6) = derivParam(i,6)                               - ...
                            Odo{i}(k,1) * sin(ThWh + ThOff) * a_sinThDth2 + ...
                            deltaD * a_cosThDth2 * ( a_Wh{i}(k,1) + Odo{i}(k,1)*cos(ThWh+ThOff)/2 ) / L;

          % TH
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2);

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,6);
            a_derivParam(1) = derivParam(i,1) / L;
            a_derivParam(2) = derivParam(i,2) * a_pi_nCe;
            a_derivParam(3) = derivParam(i,3) * D * a_pi_nCe;
            a_derivParam(4) = derivParam(i,4) / L;
            a_derivParam(5) = derivParam(i,5) * a_pi_nCe;
            a_derivParam(6) = derivParam(i,6) * D * a_pi_nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(4);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);

            % Cost function
            costSim(iteration,isegAll) = ( a_XOdoF(1) - XGt{i}(k,1) )^2 + ( a_XOdoF(2) - XGt{i}(k,2) )^2;
            cost(iteration)            = cost(iteration) + costSim(iteration,isegAll);

            % Update segment
            iseg    = iseg    + 1;
            isegAll = isegAll + 1;
          end

          % Debug
          %XOdoDebug{i}(k,:) = XOdoF(i,:);
        end
      end

      % >>>>> IRPROP-: Improved Resilient Propagation without Weight-backtracking
      for j=1:nL+nWh+nTh
        if ( costDeriv(j)*oldcostDeriv(j) > 0 )
          newdeltaD(j) = min([ olddeltaD(j) * nplus(j) , maxvar(j) ]);
        elseif ( costDeriv(j)*oldcostDeriv(j) < 0 )
          newdeltaD(j) = max([ olddeltaD(j) * nminus(j) , minvar(j) ]);
          costDeriv(j) = 0;
        else
          newdeltaD(j) = olddeltaD(j);
        end
        newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
      end
      newL     = L     + newdeltaW(1:nL);
      newD     = D     + newdeltaW(nL+1:nL+nWh);
      newThOff = ThOff + newdeltaW(nL+nWh+1:nL+nWh+nTh);
      
      % Stop condition
      % 1: maximum iterations
      if (iteration >= maxiter)
        break;
      end
      % 2: minimum variation between consecutive iterations
      if (iteration >= miniter)
        avg    = sum(cost(iteration-  numiterminavg+1:iteration              ))/numiterminavg;
        avgOld = sum(cost(iteration-2*numiterminavg+1:iteration-numiterminavg))/numiterminavg;
        if ( abs( (avg - avgOld)/avgOld ) < minvarbetiter )
          break;
        end
      end

      % Update number of iterations
      iteration = iteration + 1;

      % Update robot parameters estimation
      L = newL;
      D = newD;
      ThOff = newThOff;

      toc
    end
  end

  % Update robot parameters estimation
  L = newL;
  D = newD;
  ThOff = newThOff;

  % Output arguments
  cost    = cost(1:iteration);
  costSim = costSim(1:iteration,:);
  historyRobotParam = historyRobotParam(1:iteration,:);
  numIterations = iteration;
  RobotEstParam = RobotParam;
  RobotEstParam.L = L;
  RobotEstParam.D = D;
  RobotEstParam.ThOff = ThOff;
end