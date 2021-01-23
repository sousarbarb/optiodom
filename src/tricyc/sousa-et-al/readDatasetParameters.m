function [parameters] = readDatasetParameters(filename)

  % Initialization
  metadataCell = readcell(filename);
  metadataMat  = readmatrix(filename);
  
  % Dataset Parameters
  parameters.N = metadataMat(6,2);
  if (~isempty(metadataCell{8,2}))
    parameters.L = metadataMat(7,2);
  end
end