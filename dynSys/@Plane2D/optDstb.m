function dOpt = optDstb(obj, t, xs, deriv, dMode, ~)
% uOpt = optCtrl(obj, t, deriv, uMode, dMode, MIEdims)

%% Input processing
if nargin < 5
  dMode = 'max';
end


%% Optimal control
if iscell(deriv)
  dOpt = cell(obj.nu, 1);
  if strcmp(dMode, 'max')
    dOpt{1} = (deriv{1}>=0)*obj.dMax(1) + (deriv{1}<0)* -obj.dMax(1);
    dOpt{2} = (deriv{2}>=0)*obj.dMax(2) + (deriv{2}<0)* -obj.dMax(2);
    
  elseif strcmp(dMode, 'min')
    dOpt{1} = (deriv{1}>=0)* -obj.dMax(1) + (deriv{1}<0)*obj.dMax(1);
    dOpt{2} = (deriv{2}>=0)* -obj.dMax(2) + (deriv{2}<0)*obj.dMax(2);
  else
    error('Unknown uMode!')
  end  
  
else
  dOpt = zeros(obj.nu, 1);
  if strcmp(dMode, 'max')
    dOpt(1) = (deriv(1)>=0)*obj.dMax(1) + (deriv(1)<0)* -obj.dMax(1);
    dOpt(2) = (deriv(2)>=0)*obj.dMax(2) + (deriv(2)<0)* -obj.dMax(2);
    
  elseif strcmp(dMode, 'min')
    dOpt(1) = (deriv(1)>=0)* -obj.dMax(1) + (deriv(1)<0)*obj.dMax(1);
    dOpt(2) = (deriv(2)>=0)* -obj.dMax(2) + (deriv(2)<0)*obj.dMax(2);
    
  else
    error('Unknown uMode!')
  end
end




end