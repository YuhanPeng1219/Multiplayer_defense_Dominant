function [data,mat,vx,vy, tau2] = compute_isodata(target,params)


%% Should we compute the trajectory?
compTraj = false;

%% Grid
% grid_min = params.grid_min;
% grid_max = params.grid_max;
% N        = params.N;
% 
% g = createGrid(grid_min, grid_max, N);
g = params.g;

%% target set
% x0 = [-1;0];
dx = params.dx;
% data0 = shapeRectangleByCenter(g, x0, [dx;dx]);
data0 = target;
%% time vector
t0   = 0;
tMax = params.tMax;
dt   = params.dt;
tau  = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
% wMax  = 1;
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

%% If you have obstacles, compute them here
obstacles = params.obstacles;
HJIextraArgs.obstacles = obstacles;


%% Compute value function
HJIextraArgs.visualize = false; %show plot
HJIextraArgs.fig_num   = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.quiet = true; % quiet mode

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);


%% Post Process
mat = zeros(g.N(1),g.N(2));
for ii = 0:size(data,3)-1
    dataii= data(:,:,end-ii);
    idx = dataii<0;
    mat(idx) = max(mat(idx),ii);
end
mat = tMax-mat*dt;

vx = diff(mat,[],1);
vy = diff(mat,[],2);
vx(end+1,:) = vx(end,:);
vy(:,end+1) = vy(:,end);

vnorm = sqrt(vx.^2 + vy.^2);
vx = vx./vnorm;
vy = vy./vnorm;

