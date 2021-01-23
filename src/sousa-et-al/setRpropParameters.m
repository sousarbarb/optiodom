function [Method] = setRpropParameters(Method,RobotParam)

  % - options:
  %   > type         : 'rprop+', 'rprop-', 'irprop+', 'irprop-'
  %   > nplus        : rprop acceleration parameter
  %   > nminus       : rprop deceleration parameter
  %   > maxvar       : maximum parameters variation
  %   > minvar       : minimum parameters variation
  %   > maxiter      : maximum iterations
  %   > miniter      : minimum iterations
  %   > minvarbetiter: minimum variation to break algorithm
  %   > numiterminavg: number of iterations to compute the average

  if     (strcmp(RobotParam.type,'diff'))

    Method.options.type   = 'irprop-';
    Method.options.nplus  = [ 1.005 , 1.005 , 1.005 ];
    Method.options.nminus = [ 0.500 , 0.500 , 0.500 ];
    Method.options.maxvar = [ 0.00025 , 0.00025 , 0.00025 ];
    Method.options.minvar = [ 0.00001 , 0.00001 , 0.00001 ];
    Method.options.maxiter = 1000;
    Method.options.miniter = 30;
    Method.options.minvarbetiter = 0.001;
    Method.options.numiterminavg = 10;

  elseif (strcmp(RobotParam.type,'tricyc'))

    Method.options.type   = 'irprop-';
    Method.options.nplus  = [ 1.005 , 1.005 , 1.005 ];
    Method.options.nminus = [ 0.500 , 0.500 , 0.500 ];
    Method.options.maxvar = [ 0.00025 , 0.0025 , 0.025 ];
    Method.options.minvar = [ 0.00001 , 0.0001 , 0.001 ];
    Method.options.maxiter = 1000;
    Method.options.miniter = 30;
    Method.options.minvarbetiter = 0.001;
    Method.options.numiterminavg = 10;

  elseif (strcmp(RobotParam.type,'omni3'))

    Method.options.type   = 'irprop-';
    Method.options.nplus  = [ 1.005 , 1.005 , 1.005 , 1.005 ];
    Method.options.nminus = [ 0.500 , 0.500 , 0.500 , 0.500 ];
    Method.options.maxvar = [ 0.00025 , 0.00025 , 0.00025 , 0.00025 ];
    Method.options.minvar = [ 0.00001 , 0.00001 , 0.00001 , 0.00001 ];
    Method.options.maxiter = 1000;
    Method.options.miniter = 50;
    Method.options.minvarbetiter = 0.0001;
    Method.options.numiterminavg = 20;

  elseif (strcmp(RobotParam.type,'omni4'))

    Method.options.type   = 'irprop-';
    Method.options.nplus  = [ 1.005 , 1.005 , 1.005 , 1.005 , 1.005 , 1.005 ];
    Method.options.nminus = [ 0.500 , 0.500 , 0.500 , 0.500 , 0.500 , 0.500 ];
    Method.options.maxvar = [ 0.00025 , 0.00025 , 0.00025 , 0.00025 , 0.00025 , 0.00025 ];
    Method.options.minvar = [ 0.00001 , 0.00001 , 0.00001 , 0.00001 , 0.00001 , 0.00001 ];
    Method.options.maxiter = 1000;
    Method.options.miniter = 50;
    Method.options.minvarbetiter = 0.0001;
    Method.options.numiterminavg = 20;

  end
end