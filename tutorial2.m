% function tutorial2()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

clear all
close all
% rng(2)
%% Should we compute the trajectory?
compTraj = false;

%% Grid
% grid_min = [-5; -5; -pi]; % Lower corner of computation domain
% grid_max = [5; 5; pi];    % Upper corner of computation domain
% N = [41; 41; 41];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
% g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

grid_min = [-2;-2];
grid_max = [ 2; 2];
N = [100;100];
g = createGrid(grid_min, grid_max, N);

%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.
x0 = [0.8;0.3];
% data0 = shapeRectangleByCenter(g, x0, [.1;.1]);
data0 = shapeSphere(g, x0, .1);
figure()
surf(data0)


%% time vector
t0 = 0;
tMax = 5;
dt = 0.02;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
wMax  = 1;
% do dStep1 here
% control trying to min or max value function?
uMode = 'min';
% do dStep2 here

%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
% dCar = DubinsCar([0, 0, 0], wMax, speed); %do dStep3 here
obj = KinVehicleND([0, 0], speed);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
% schemeData.dynSys = dCar;
schemeData.dynSys = obj;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here

%% additive random noise
%do Step8 here
%HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here

% for ii = 1:10
%     obst{ii} = shapeRectangleByCenter( g, rand(2,1)*4-2, rand(2,1)*1.5 );
% end

obst{1} = shapeRectangleByCenter( g, [-.4;0], [.3;1] );
obst{2} = shapeRectangleByCenter( g, [0;.4],  [1;.3] );
obst{3} = shapeRectangleByCenter( g, [1;-1]*.5,  [.7;.5] );
obst{4} = shapeRectangleByCenter( g, [-.6;-.75], [.3;1] );
obst{5} = shapeRectangleByCenter( g, [0.2;-1.1],  [1.8;.3] );
obst{6} = shapeRectangleByCenter( g, [-.7;1.2], [1;.5] );
obst{7} = shapeRectangleByCenter( g, [1.2;1]*.9,  [.5;.9] );
% obst{6} = shapeRectangleByCenter( g, [0;.4],  [1;.3] );
% obst{5} = shapeRectangleByCenter( g, rand(2,1)*4-2, [.2;1] );
% obst{6} = shapeRectangleByCenter( g, rand(2,1)*4-2, [.2;1] );
obstacles = obst{1};
for ii = 1:numel(obst)
    obstacles = min(obstacles,obst{ii});
end

obstacles(obstacles<-0.05) = -.1;

HJIextraArgs.obstacles = obstacles;

%% Compute value function

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);

mat = zeros(N(1),N(2));
for ii = 0:size(data,3)-1
    dataii= data(:,:,end-ii);
    idx = dataii<0;
    mat(idx) = max(mat(idx),ii);
end

%%
% contourf(mat)

%%
X = g.xs{1};
Y = g.xs{2};

% [vx,vy] = gradient(mat,X,Y);

vx = diff(mat,[],1);
vy = diff(mat,[],2);
vx(end+1,:) = vx(end,:);
vy(:,end+1) = vy(:,end);

vnorm = sqrt(vx.^2 + vy.^2);
vx = vx./vnorm;
vy = vy./vnorm;

% mv = mean(abs(vx(:)));
% vx = min(vx,mv);
% vx = max(vx,-mv);
% vy = min(vy,mv);
% vy = max(vy,-mv);

% surf(vx.^2+vy.^2)
quiver(X,Y,vx,vy)

%%
[xm,ym] = meshgrid(linspace(-2,2,20),linspace(-2,2,20));
pstart = [xm(:)';ym(:)'];
h = streamline(X',Y',vx',vy',pstart(1,:),pstart(2,:));
set(h,'linewidth',1,'color','r')

%%

% figure()
% surf(obstacles)
% %% Compute optimal trajectory from some initial state
% if compTraj
%   pause
%   
%   %set the initial state
%   xinit = [2, 1, -pi];
%   
%   %check if this initial state is in the BRS/BRT
%   %value = eval_u(g, data, x)
%   value = eval_u(g,data(:,:,:,end),xinit);
%   
%   if value <= 0 %if initial state is in BRS/BRT
%     % find optimal trajectory
%     
%     dCar.x = xinit; %set initial state of the dubins car
% 
%     TrajextraArgs.uMode = uMode; %set if control wants to min or max
%     TrajextraArgs.visualize = true; %show plot
%     TrajextraArgs.fig_num = 2; %figure number
%     
%     %we want to see the first two dimensions (x and y)
%     TrajextraArgs.projDim = [1 1 0]; 
%     
%     %flip data time points so we start from the beginning of time
%     dataTraj = flip(data,4);
%     
%     % [traj, traj_tau] = ...
%     % computeOptTraj(g, data, tau, dynSys, extraArgs)
%     [traj, traj_tau] = ...
%       computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
%   else
%     error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
%   end
% end
% % end
