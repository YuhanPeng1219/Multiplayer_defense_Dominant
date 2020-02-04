classdef Plane2D < DynSys
  properties
    % Bound for speed in x
    vxMax
    
    % Bound for speed in y
    vyMax
    
    % Disturbance bounds
    dMax

  end
  
  methods
    function obj = Plane2D(x, vxMax, vyMax, dMax)
      % obj = Plane(x, vxMax, vyMax)
      %
      % Constructor. Creates a plane object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = vx 
      %    \dot{x}_2 = vy 
      %         vx \in [-vxMax, vxMax]
      %         vy \in [-vyMax, vyMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   vxMax  - maximum speed in x
      %   vyMax  - maximum speed in y
      %
      % Output:
      %   obj       - a Plane object
      %
      
      if numel(x) ~= 2
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
            
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.vxMax = vxMax;
      obj.vyMax = vyMax;
      
      obj.dMax = dMax;
      
      obj.pdim = 1:2;
      
      obj.nx = 2;
      obj.nu = 2;
      obj.nd = 2;
    end
    
  end % end methods
end % end classdef
