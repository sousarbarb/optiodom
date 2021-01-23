function [Xcell] = cellarray2cellarray(X,iini,ifin)

  % Initialisation
  Xcell = cell(1,ifin-iini+1);

  for i=iini:ifin
    Xcell{i-iini+1} = X{i};
  end
end