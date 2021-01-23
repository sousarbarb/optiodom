function [RobotEstParam,cost,costSim,numIterations,historyRobotParam] = Rprop_omni3(RobotParam,XGt,Odo,iSamples,options)

  % Initialization: Robot parameters
  n  = RobotParam.ngear;
  Ce = RobotParam.encRes;
  L  = RobotParam.L;
  D  = RobotParam.D;

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
  a_pi_3nCe = pi/(3*n*Ce);
  a_pi_6nCe = pi/(6*n*Ce);
  a_Wh = cell(Nsim,1);
  itf  = zeros(Nsim,1);
  for i=1:Nsim
    [a_itf,nWh] = size(Odo{i});
    itf(i)  = a_itf;
    a_Wh{i} = zeros(itf(i),nWh);
    for j=1:nWh
      for k=2:itf(i)
        a_Wh{i}(k,j) = sum(Odo{i}(2:k-1,j))*a_pi_3nCe;
      end
    end
  end
  nL = length(L);

  % Initialization: Procedure parameters
  %XOdoDebug = XGt;
  XOdoF = zeros(Nsim,3);
  cost    = zeros(maxiter,1);
  costSim = zeros(maxiter,1);
  costDeriv = zeros(1,nWh+nL);        % [ dE/dL dE/dD(1) dE/dD(2) dE/dD(3) ]
  historyRobotParam = zeros(maxiter,nWh+nL);

  % Initialization: Parameters variation
  newdeltaD = minvar;
  newdeltaW = zeros(1,nWh+nL);

  % RPROP ALGORITHM
  iteration = 1;
  if strcmp(rproptype,'rprop+')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D ];

      % Parameters initialization
      derivParam = zeros(Nsim,(nWh+nL)*2);  % [ dx/dL dx/dD(1) dx/dD(2) dx/dD(3) dy/dL dy/dD(1) dy/dD(2) dy/dD(3) ]
      costDeriv  = zeros(1,nWh+nL);         % [ dE/dL dE/dD(1) dE/dD(2) dE/dD(3) ]
      
      isegAll = 1;

      % Common variables
      a_pi_3nCeL = a_pi_3nCe / L;

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          deltaD  = ( -D(1)*Odo{i}(k,1) + D(2)*Odo{i}(k,2) )*((3)^(1/2));
          deltaDN = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) + 2*D(3)*Odo{i}(k,3) );
          deltaTH = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) -   D(3)*Odo{i}(k,3) ) * a_pi_3nCeL;

          % X and Y
          if ( abs(deltaTH)/0.040 < 1e-5 )
            % Common variables
            a_sinTh = sin(XOdoF(i,3));
            a_cosTh = cos(XOdoF(i,3));
            a_dDCosdDnSin = ( deltaD * a_cosTh - deltaDN * a_sinTh );
            a_dDSindDnCos = ( deltaD * a_sinTh + deltaDN * a_cosTh );

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDCosdDnSin;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindDnCos;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1) + a_dDSindDnCos * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,2) = derivParam(i,2)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_cosTh + ...
                                          Odo{i}(k,1) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,1) / L;
            derivParam(i,3) = derivParam(i,3)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_cosTh + ...
                                          Odo{i}(k,2) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,2) / L;
            derivParam(i,4) = derivParam(i,4)                   - ...
                                      2 * Odo{i}(k,3) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,3) / L;
            derivParam(i,5) = derivParam(i,5) + ( -deltaD * a_cosTh + deltaDN * a_sinTh ) * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,6) = derivParam(i,6)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_sinTh - ...
                                          Odo{i}(k,1) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,1) / L;
            derivParam(i,7) = derivParam(i,7)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_sinTh - ...
                                          Odo{i}(k,2) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,2) / L;
            derivParam(i,8) = derivParam(i,8)                   + ...
                                      2 * Odo{i}(k,3) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,3) / L;
          else
            % Common variables
            a_sindTh = sin(deltaTH);
            a_cosdTh = cos(deltaTH);
            a_dDSindTh  = deltaD  * a_sindTh;
            a_dDCosdTh  = deltaD  * a_cosdTh;
            a_dDnSindTh = deltaDN * a_sindTh;
            a_dDnCosdTh = deltaDN * a_cosdTh;
            a_dDnCosdTh1 = deltaDN * (a_cosdTh - 1);
            a_dD1CosdTh  = deltaD  * (1 - a_cosdTh);
            a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
            a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );
            a_sinThDth2_dTh = a_sinThDth2 / deltaTH;
            a_cosThDth2_dTh = a_cosThDth2 / deltaTH;
            a_dDSindThdDnCosdTh1 = a_dDSindTh  + a_dDnCosdTh1;
            a_dD1CosdThdDnSindTh = a_dD1CosdTh + a_dDnSindTh;

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDSindThdDnCosdTh1 * a_cosThDth2_dTh - a_dD1CosdThdDnSindTh * a_sinThDth2_dTh;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindThdDnCosdTh1 * a_sinThDth2_dTh + a_dD1CosdThdDnSindTh * a_cosThDth2_dTh;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH + ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_sinThDth2                                                                     - ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH;
            derivParam(i,2) = derivParam(i,2)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,3) = derivParam(i,3)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,4) = derivParam(i,4)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,5) = derivParam(i,5)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_sinThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH - ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH;
            derivParam(i,6) = derivParam(i,6)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,7) = derivParam(i,7)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,8) = derivParam(i,8)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
          end

          % Theta
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2) * a_pi_3nCe;

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,8);
            a_derivParam(1) = derivParam(i,1) * a_pi_3nCeL;
            a_derivParam(2) = derivParam(i,2) * a_pi_3nCe;
            a_derivParam(3) = derivParam(i,3) * a_pi_3nCe;
            a_derivParam(4) = derivParam(i,4) * a_pi_3nCe;
            a_derivParam(5) = derivParam(i,5) * a_pi_3nCeL;
            a_derivParam(6) = derivParam(i,6) * a_pi_3nCe;
            a_derivParam(7) = derivParam(i,7) * a_pi_3nCe;
            a_derivParam(8) = derivParam(i,8) * a_pi_3nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(7);
            costDeriv(4) = costDeriv(4) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(4) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(8);

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
      for j=1:nWh+nL
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
      newL = L + newdeltaW(1:nL);
      newD = D + newdeltaW(nL+1:nL+nWh);
      
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

      toc
    end
  elseif strcmp(rproptype,'rprop-')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D ];

      % Parameters initialization
      derivParam = zeros(Nsim,(nWh+nL)*2);  % [ dx/dL dx/dD(1) dx/dD(2) dx/dD(3) dy/dL dy/dD(1) dy/dD(2) dy/dD(3) ]
      costDeriv  = zeros(1,nWh+nL);         % [ dE/dL dE/dD(1) dE/dD(2) dE/dD(3) ]
      
      isegAll = 1;

      % Common variables
      a_pi_3nCeL = a_pi_3nCe / L;

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          deltaD  = ( -D(1)*Odo{i}(k,1) + D(2)*Odo{i}(k,2) )*((3)^(1/2));
          deltaDN = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) + 2*D(3)*Odo{i}(k,3) );
          deltaTH = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) -   D(3)*Odo{i}(k,3) ) * a_pi_3nCeL;

          % X and Y
          if ( abs(deltaTH)/0.040 < 1e-5 )
            % Common variables
            a_sinTh = sin(XOdoF(i,3));
            a_cosTh = cos(XOdoF(i,3));
            a_dDCosdDnSin = ( deltaD * a_cosTh - deltaDN * a_sinTh );
            a_dDSindDnCos = ( deltaD * a_sinTh + deltaDN * a_cosTh );

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDCosdDnSin;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindDnCos;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1) + a_dDSindDnCos * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,2) = derivParam(i,2)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_cosTh + ...
                                          Odo{i}(k,1) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,1) / L;
            derivParam(i,3) = derivParam(i,3)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_cosTh + ...
                                          Odo{i}(k,2) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,2) / L;
            derivParam(i,4) = derivParam(i,4)                   - ...
                                      2 * Odo{i}(k,3) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,3) / L;
            derivParam(i,5) = derivParam(i,5) + ( -deltaD * a_cosTh + deltaDN * a_sinTh ) * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,6) = derivParam(i,6)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_sinTh - ...
                                          Odo{i}(k,1) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,1) / L;
            derivParam(i,7) = derivParam(i,7)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_sinTh - ...
                                          Odo{i}(k,2) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,2) / L;
            derivParam(i,8) = derivParam(i,8)                   + ...
                                      2 * Odo{i}(k,3) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,3) / L;
          else
            % Common variables
            a_sindTh = sin(deltaTH);
            a_cosdTh = cos(deltaTH);
            a_dDSindTh  = deltaD  * a_sindTh;
            a_dDCosdTh  = deltaD  * a_cosdTh;
            a_dDnSindTh = deltaDN * a_sindTh;
            a_dDnCosdTh = deltaDN * a_cosdTh;
            a_dDnCosdTh1 = deltaDN * (a_cosdTh - 1);
            a_dD1CosdTh  = deltaD  * (1 - a_cosdTh);
            a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
            a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );
            a_sinThDth2_dTh = a_sinThDth2 / deltaTH;
            a_cosThDth2_dTh = a_cosThDth2 / deltaTH;
            a_dDSindThdDnCosdTh1 = a_dDSindTh  + a_dDnCosdTh1;
            a_dD1CosdThdDnSindTh = a_dD1CosdTh + a_dDnSindTh;

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDSindThdDnCosdTh1 * a_cosThDth2_dTh - a_dD1CosdThdDnSindTh * a_sinThDth2_dTh;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindThdDnCosdTh1 * a_sinThDth2_dTh + a_dD1CosdThdDnSindTh * a_cosThDth2_dTh;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH + ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_sinThDth2                                                                     - ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH;
            derivParam(i,2) = derivParam(i,2)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,3) = derivParam(i,3)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,4) = derivParam(i,4)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,5) = derivParam(i,5)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_sinThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH - ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH;
            derivParam(i,6) = derivParam(i,6)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,7) = derivParam(i,7)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,8) = derivParam(i,8)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
          end

          % Theta
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2) * a_pi_3nCe;

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,8);
            a_derivParam(1) = derivParam(i,1) * a_pi_3nCeL;
            a_derivParam(2) = derivParam(i,2) * a_pi_3nCe;
            a_derivParam(3) = derivParam(i,3) * a_pi_3nCe;
            a_derivParam(4) = derivParam(i,4) * a_pi_3nCe;
            a_derivParam(5) = derivParam(i,5) * a_pi_3nCeL;
            a_derivParam(6) = derivParam(i,6) * a_pi_3nCe;
            a_derivParam(7) = derivParam(i,7) * a_pi_3nCe;
            a_derivParam(8) = derivParam(i,8) * a_pi_3nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(7);
            costDeriv(4) = costDeriv(4) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(4) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(8);

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
      for j=1:nWh+nL
        if ( costDeriv(j)*oldcostDeriv(j) > 0 )
          newdeltaD(j) = min([ olddeltaD(j) * nplus(j) , maxvar(j) ]);
        elseif ( costDeriv(j)*oldcostDeriv(j) < 0 )
          newdeltaD(j) = max([ olddeltaD(j) * nminus(j) , minvar(j) ]);
        else
          newdeltaD(j) = olddeltaD(j);
        end
        newdeltaW(j) = -sign(costDeriv(j)) * newdeltaD(j);
      end
      newL = L + newdeltaW(1:nL);
      newD = D + newdeltaW(nL+1:nL+nWh);
      
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

      toc
    end
  elseif strcmp(rproptype,'irprop+')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D ];

      % Parameters initialization
      derivParam = zeros(Nsim,(nWh+nL)*2);  % [ dx/dL dx/dD(1) dx/dD(2) dx/dD(3) dy/dL dy/dD(1) dy/dD(2) dy/dD(3) ]
      costDeriv  = zeros(1,nWh+nL);         % [ dE/dL dE/dD(1) dE/dD(2) dE/dD(3) ]
      
      isegAll = 1;

      % Common variables
      a_pi_3nCeL = a_pi_3nCe / L;

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          deltaD  = ( -D(1)*Odo{i}(k,1) + D(2)*Odo{i}(k,2) )*((3)^(1/2));
          deltaDN = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) + 2*D(3)*Odo{i}(k,3) );
          deltaTH = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) -   D(3)*Odo{i}(k,3) ) * a_pi_3nCeL;

          % X and Y
          if ( abs(deltaTH)/0.040 < 1e-5 )
            % Common variables
            a_sinTh = sin(XOdoF(i,3));
            a_cosTh = cos(XOdoF(i,3));
            a_dDCosdDnSin = ( deltaD * a_cosTh - deltaDN * a_sinTh );
            a_dDSindDnCos = ( deltaD * a_sinTh + deltaDN * a_cosTh );

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDCosdDnSin;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindDnCos;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1) + a_dDSindDnCos * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,2) = derivParam(i,2)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_cosTh + ...
                                          Odo{i}(k,1) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,1) / L;
            derivParam(i,3) = derivParam(i,3)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_cosTh + ...
                                          Odo{i}(k,2) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,2) / L;
            derivParam(i,4) = derivParam(i,4)                   - ...
                                      2 * Odo{i}(k,3) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,3) / L;
            derivParam(i,5) = derivParam(i,5) + ( -deltaD * a_cosTh + deltaDN * a_sinTh ) * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,6) = derivParam(i,6)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_sinTh - ...
                                          Odo{i}(k,1) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,1) / L;
            derivParam(i,7) = derivParam(i,7)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_sinTh - ...
                                          Odo{i}(k,2) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,2) / L;
            derivParam(i,8) = derivParam(i,8)                   + ...
                                      2 * Odo{i}(k,3) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,3) / L;
          else
            % Common variables
            a_sindTh = sin(deltaTH);
            a_cosdTh = cos(deltaTH);
            a_dDSindTh  = deltaD  * a_sindTh;
            a_dDCosdTh  = deltaD  * a_cosdTh;
            a_dDnSindTh = deltaDN * a_sindTh;
            a_dDnCosdTh = deltaDN * a_cosdTh;
            a_dDnCosdTh1 = deltaDN * (a_cosdTh - 1);
            a_dD1CosdTh  = deltaD  * (1 - a_cosdTh);
            a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
            a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );
            a_sinThDth2_dTh = a_sinThDth2 / deltaTH;
            a_cosThDth2_dTh = a_cosThDth2 / deltaTH;
            a_dDSindThdDnCosdTh1 = a_dDSindTh  + a_dDnCosdTh1;
            a_dD1CosdThdDnSindTh = a_dD1CosdTh + a_dDnSindTh;

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDSindThdDnCosdTh1 * a_cosThDth2_dTh - a_dD1CosdThdDnSindTh * a_sinThDth2_dTh;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindThdDnCosdTh1 * a_sinThDth2_dTh + a_dD1CosdThdDnSindTh * a_cosThDth2_dTh;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH + ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_sinThDth2                                                                     - ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH;
            derivParam(i,2) = derivParam(i,2)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,3) = derivParam(i,3)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,4) = derivParam(i,4)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,5) = derivParam(i,5)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_sinThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH - ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH;
            derivParam(i,6) = derivParam(i,6)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,7) = derivParam(i,7)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,8) = derivParam(i,8)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
          end

          % Theta
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2) * a_pi_3nCe;

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,8);
            a_derivParam(1) = derivParam(i,1) * a_pi_3nCeL;
            a_derivParam(2) = derivParam(i,2) * a_pi_3nCe;
            a_derivParam(3) = derivParam(i,3) * a_pi_3nCe;
            a_derivParam(4) = derivParam(i,4) * a_pi_3nCe;
            a_derivParam(5) = derivParam(i,5) * a_pi_3nCeL;
            a_derivParam(6) = derivParam(i,6) * a_pi_3nCe;
            a_derivParam(7) = derivParam(i,7) * a_pi_3nCe;
            a_derivParam(8) = derivParam(i,8) * a_pi_3nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(7);
            costDeriv(4) = costDeriv(4) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(4) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(8);

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
      for j=1:nWh+nL
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
      newL = L + newdeltaW(1:nL);
      newD = D + newdeltaW(nL+1:nL+nWh);
      
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

      toc
    end
  elseif strcmp(rproptype,'irprop-')
    while (true)
      tic

      % Updating old values
      oldcostDeriv = costDeriv;
      olddeltaD  = newdeltaD;
      olddeltaW  = newdeltaW;
      historyRobotParam(iteration,:) = [ L , D ];

      % Parameters initialization
      derivParam = zeros(Nsim,(nWh+nL)*2);  % [ dx/dL dx/dD(1) dx/dD(2) dx/dD(3) dy/dL dy/dD(1) dy/dD(2) dy/dD(3) ]
      costDeriv  = zeros(1,nWh+nL);         % [ dE/dL dE/dD(1) dE/dD(2) dE/dD(3) ]
      
      isegAll = 1;

      % Common variables
      a_pi_3nCeL = a_pi_3nCe / L;

      % Odometry estimation % derivatives computation
      for i=1:Nsim
        %XOdoDebug{i}(1,:) = XGt{i}(1,:);

        XOdoF(i,:) = XGt{i}(1,:);
        iseg = 1;
        
        for k=2:itf(i)
          % Common variables
          deltaD  = ( -D(1)*Odo{i}(k,1) + D(2)*Odo{i}(k,2) )*((3)^(1/2));
          deltaDN = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) + 2*D(3)*Odo{i}(k,3) );
          deltaTH = ( -D(1)*Odo{i}(k,1) - D(2)*Odo{i}(k,2) -   D(3)*Odo{i}(k,3) ) * a_pi_3nCeL;

          % X and Y
          if ( abs(deltaTH)/0.040 < 1e-5 )
            % Common variables
            a_sinTh = sin(XOdoF(i,3));
            a_cosTh = cos(XOdoF(i,3));
            a_dDCosdDnSin = ( deltaD * a_cosTh - deltaDN * a_sinTh );
            a_dDSindDnCos = ( deltaD * a_sinTh + deltaDN * a_cosTh );

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDCosdDnSin;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindDnCos;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1) + a_dDSindDnCos * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,2) = derivParam(i,2)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_cosTh + ...
                                          Odo{i}(k,1) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,1) / L;
            derivParam(i,3) = derivParam(i,3)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_cosTh + ...
                                          Odo{i}(k,2) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,2) / L;
            derivParam(i,4) = derivParam(i,4)                   - ...
                                      2 * Odo{i}(k,3) * a_sinTh + ...
                              a_dDSindDnCos * a_Wh{i}(k,3) / L;
            derivParam(i,5) = derivParam(i,5) + ( -deltaD * a_cosTh + deltaDN * a_sinTh ) * ( XOdoF(i,3) - XGt{i}(1,3) );
            derivParam(i,6) = derivParam(i,6)                   - ...
                              (3^(1/2)) * Odo{i}(k,1) * a_sinTh - ...
                                          Odo{i}(k,1) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,1) / L;
            derivParam(i,7) = derivParam(i,7)                   + ...
                              (3^(1/2)) * Odo{i}(k,2) * a_sinTh - ...
                                          Odo{i}(k,2) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,2) / L;
            derivParam(i,8) = derivParam(i,8)                   + ...
                                      2 * Odo{i}(k,3) * a_cosTh - ...
                              a_dDCosdDnSin * a_Wh{i}(k,3) / L;
          else
            % Common variables
            a_sindTh = sin(deltaTH);
            a_cosdTh = cos(deltaTH);
            a_dDSindTh  = deltaD  * a_sindTh;
            a_dDCosdTh  = deltaD  * a_cosdTh;
            a_dDnSindTh = deltaDN * a_sindTh;
            a_dDnCosdTh = deltaDN * a_cosdTh;
            a_dDnCosdTh1 = deltaDN * (a_cosdTh - 1);
            a_dD1CosdTh  = deltaD  * (1 - a_cosdTh);
            a_sinThDth2 = sin( XOdoF(i,3) + deltaTH/2 );
            a_cosThDth2 = cos( XOdoF(i,3) + deltaTH/2 );
            a_sinThDth2_dTh = a_sinThDth2 / deltaTH;
            a_cosThDth2_dTh = a_cosThDth2 / deltaTH;
            a_dDSindThdDnCosdTh1 = a_dDSindTh  + a_dDnCosdTh1;
            a_dD1CosdThdDnSindTh = a_dD1CosdTh + a_dDnSindTh;

            % X and Y
            XOdoF(i,1) = XOdoF(i,1) + a_dDSindThdDnCosdTh1 * a_cosThDth2_dTh - a_dD1CosdThdDnSindTh * a_sinThDth2_dTh;
            XOdoF(i,2) = XOdoF(i,2) + a_dDSindThdDnCosdTh1 * a_sinThDth2_dTh + a_dD1CosdThdDnSindTh * a_cosThDth2_dTh;

            % Compute derivatives
            derivParam(i,1) = derivParam(i,1)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH + ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_sinThDth2                                                                     - ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH;
            derivParam(i,2) = derivParam(i,2)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,3) = derivParam(i,3)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,4) = derivParam(i,4)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dDSindThdDnCosdTh1 * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) + ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,5) = derivParam(i,5)                                                                                                   + ...
                              ( -a_dDCosdTh  + a_dDnSindTh  ) * a_sinThDth2                                                                     + ...
                              (  a_dDSindTh  + a_dDnCosdTh1 ) * ( -a_cosThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_sinThDth2 ) / deltaTH - ...
                              (  a_dDSindTh  + a_dDnCosdTh  ) * a_cosThDth2                                                                     + ...
                              (  a_dD1CosdTh + a_dDnSindTh  ) * (  a_sinThDth2 * ( XOdoF(i,3)-XGt{i}(1,3)+deltaTH/2 ) + a_cosThDth2 ) / deltaTH;
            derivParam(i,6) = derivParam(i,6)                                                                                                                               - ...
                              ( (3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              ( (3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,1) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,1)+Odo{i}(k,1)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,1)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,7) = derivParam(i,7)                                                                                                                               - ...
                              (-(3^(1/2))*a_sindTh + a_dDCosdTh*a_pi_3nCeL + (a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (-(3^(1/2))*(1-a_cosdTh) + a_dDSindTh*a_pi_3nCeL + a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,2) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,2)+Odo{i}(k,2)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,2)*a_pi_3nCe ) / (L*(deltaTH^2));
            derivParam(i,8) = derivParam(i,8)                                                                                                                               - ...
                              (                    a_dDCosdTh*a_pi_3nCeL - 2*(a_cosdTh-1) - a_dDnSindTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_sinThDth2_dTh                        - ...
                              a_dDSindThdDnCosdTh1 * ( a_cosThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH - a_sinThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2)) - ...
                              (                        a_dDSindTh*a_pi_3nCeL - 2*a_sindTh + a_dDnCosdTh*a_pi_3nCeL ) * Odo{i}(k,3) * a_cosThDth2_dTh                        + ...
                              a_dD1CosdThdDnSindTh * ( a_sinThDth2 * (a_Wh{i}(k,3)+Odo{i}(k,3)*a_pi_6nCe) * deltaTH + a_cosThDth2*Odo{i}(k,3)*a_pi_3nCe ) / (L*(deltaTH^2));
          end

          % Theta
          XOdoF(i,3) = XOdoF(i,3) + deltaTH;


          % >>>>> FINAL COST FOR EACH PATH SEGMENT <<<<<
          if (k == iSamples{i}(iseg))
            % Final position
            a_XOdoF = XOdoF(i,1:2) * a_pi_3nCe;

            % Final computation of the partial derivatives
            a_derivParam = zeros(1,8);
            a_derivParam(1) = derivParam(i,1) * a_pi_3nCeL;
            a_derivParam(2) = derivParam(i,2) * a_pi_3nCe;
            a_derivParam(3) = derivParam(i,3) * a_pi_3nCe;
            a_derivParam(4) = derivParam(i,4) * a_pi_3nCe;
            a_derivParam(5) = derivParam(i,5) * a_pi_3nCeL;
            a_derivParam(6) = derivParam(i,6) * a_pi_3nCe;
            a_derivParam(7) = derivParam(i,7) * a_pi_3nCe;
            a_derivParam(8) = derivParam(i,8) * a_pi_3nCe;

            % Computation of the cost derivatives: [ dE/dL dE/dD(1) dE/dD(2) ]
            costDeriv(1) = costDeriv(1) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(1) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(5);
            costDeriv(2) = costDeriv(2) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(2) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(6);
            costDeriv(3) = costDeriv(3) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(3) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(7);
            costDeriv(4) = costDeriv(4) + 2 * ( a_XOdoF(1) - XGt{i}(k,1) ) * a_derivParam(4) + 2 * ( a_XOdoF(2) - XGt{i}(k,2) ) * a_derivParam(8);

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
      for j=1:nWh+nL
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
      newL = L + newdeltaW(1:nL);
      newD = D + newdeltaW(nL+1:nL+nWh);
      
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

      toc
    end
  end

  % Update robot parameters estimation
  L = newL;
  D = newD;

  % Output arguments
  cost    = cost(1:iteration);
  costSim = costSim(1:iteration,:);
  historyRobotParam = historyRobotParam(1:iteration,:);
  numIterations = iteration;
  RobotEstParam = RobotParam;
  RobotEstParam.L = L;
  RobotEstParam.D = D;
  RobotEstParam.J = [
    -(3^(1/2))/3 , (3^(1/2))/3 ,        0 ; ...
            -1/3 ,        -1/3 ,      2/3 ; ...
        -1/(3*L) ,    -1/(3*L) , -1/(3*L)   ...
  ];
  RobotEstParam.Jinv = eye(3)/RobotEstParam.J;
end