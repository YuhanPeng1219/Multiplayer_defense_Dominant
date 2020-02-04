clear all
addpath(genpath('util'));
%% ============== data loader =====================
% load('data/data_new_10000.mat');
load('data/data_N100_mesh10000.mat');

%% parameters
grid_min = [-2;-2];
grid_max = [ 2; 2];
N        = [100;100];
g = createGrid(grid_min, grid_max, N);

tMax     = 5;
dt       = 0.01;

X = g.xs{1};
Y = g.xs{2};

%% Obstacles
obst{1} = shapeRectangleByCenter( g, [-.4;0], [.3;1] );
obst{2} = shapeRectangleByCenter( g, [0;.4],  [1;.3] );
obst{3} = shapeRectangleByCenter( g, [1;-1]*.5,  [.7;.5] );
obst{4} = shapeRectangleByCenter( g, [-.6;-.75], [.3;1] );
obst{5} = shapeRectangleByCenter( g, [0.2;-1.1],  [1.8;.3] );
obst{6} = shapeRectangleByCenter( g, [-.7;1.2], [1;.5] );
obst{7} = shapeRectangleByCenter( g, [1.2;1]*.9,  [.5;.9] );
obstacles = obst{1};
for ii = 1:numel(obst)
    obstacles = min(obstacles,obst{ii});
end

obstacles(obstacles<-0) = -.1;

%% Players
xd = [-1,0.5]; xa = [1,-1.5];
target = [0.6,1];

%%

xg      = linspace(grid_min(1),grid_max(1),100);
yg      = linspace(grid_min(2),grid_max(2),100);
[xm,ym] = meshgrid(xg,yg);
% xg = g.vs{1};
% yg = g.vs{2};

dx       = xg(2)-xg(1); % initial box size


% x0_list = {[0.1;0.7],[-1.5;-0.5]};
x0_list = [xm(:)';ym(:)'];
n       = size(x0_list,2);

%% Store params
params = v2struct(g,tMax,dt,obstacles,dx);
Interp_N = 800;

%% ================== Visulize Figure =========================
figure(); axis equal; hold on
% obstacles
contourf(X,Y,obstacles,[0 0],'k','linewidth',2)

%% =================== Dominant Reigon =========================
[matD,~] = interpolate_mat(mat_save,x0_list,xg,yg,xd);
[matA,~] = interpolate_mat(mat_save,x0_list,xg,yg,xa);

[matD_interp, ~] = gridInterpolation(g, matD, Interp_N);
[matA_interp, ~] = gridInterpolation(g, matA, Interp_N);

dmat = matD-matA;
dmat(obstacles<0) = nan;

% players
hd = plot(xd(1),xd(2),'ks','markerfacecolor','b','markersize',10);
ha = plot(xa(1),xa(2),'k^','markerfacecolor','r','markersize',10);
plot(target(1),target(2),'*','markerfacecolor','y','markersize',10)

% Dominant region
[dmat_interp, g_new] = gridInterpolation(g, dmat, Interp_N);
[bound,hdomin] = contour(g_new.xs{1},g_new.xs{2},dmat_interp,[0,0],'k','linewidth',3);
bound(:,1) = [];

% compute isochrone
matD_interp(dmat_interp>0) = nan;
matA_interp(dmat_interp<0) = nan;

%% ============== Optimal breach point =====================
% Reach or not
value = eval_u(g, dmat, target);
if value<0
    disp('Attacker cannot reach')
    % interpolate
    [X,Y] = ndgrid(xg);
    [matG,~] = interpolate_mat(mat_save,x0_list,xg,yg,target);
    matG(obstacles<0) = nan;
    
    [matG_interp, g_new] = gridInterpolation(g, matG, Interp_N);
    
    t_v = eval_u(g_new, matG_interp, bound);
    
    [~, ind] = min(t_v);
    opt = bound(:,ind);
    scatter(opt(1),opt(2));
else
    disp('Attacker can reach')
    opt = target;
end

%% ================ Main Loop ===========================
% Dynamic
obj = KinVehicleND([0, 0], 1);
obj.x = xd;
xs = [xd;xa];
xsave{1} = xd;
xsave{2} = xa;

players = 2;

% Time parameters
t0   = 0;
tMax = params.tMax;
dt   = params.dt;
tau  = t0:dt:tMax;
iter = 1;
tauLength = length(tau);
subSamples = 4;
tEarliest = [1,1];
dtSmall = (tau(2) - tau(1))/subSamples;
small = 0.1;

% data save
dmat_list{1} = dmat;
matD_list{1} = matD_interp;
matA_list{1} = matA_interp;
opt_list = [opt];

[data,mat,vx,vy,~] = compute_pointdata(opt,params);
data_f = flip(data,3);
    
%         [traj, traj_tau] = computeOptTraj(g, data_f, tau, obj);
while iter <= tauLength
    tEarliest = [1,1];
    for i = 1:2
        % Determine the earliest time that the current state is in the reachable set
        % Binary search
        upper = tauLength;
        lower = tEarliest(i);
        
        tEarliest(i) = find_earliest_BRS_ind(g, data_f, xs(i,:), upper, lower);
        
        % BRS at current time
        BRS_at_t = data_f(:,:,tEarliest(i));
        
        Deriv = computeGradients(g, BRS_at_t);
        for j = 1:subSamples
            deriv = eval_u(g, Deriv, xs(i,:));
            u = obj.optCtrl(tau(tEarliest(i)), xs(i,:), deriv, 'min');    
            xs(i,:) = xs(i,:)+u'*dtSmall;
        end
        xsave{i} = [xsave{i};xs(i,:)];
        scatter(xs(i,1), xs(i,2));
    end
    
    % update dominant region
    [dmat_interp, dmat, matD_interp, matA_interp, bound] = dominantCal(xs(1,:),...
        xs(2,:), mat_save, x0_list, g, xg, yg, hdomin, obstacles, Interp_N);
    
    % restore dominant region and isochrones 
    dmat_list{iter+1} = dmat_interp;
    matD_interp(dmat_interp>0) = nan;
    matA_interp(dmat_interp<0) = nan;
    matD_list{iter+1} = matD_interp;
    matA_list{iter+1} = matA_interp;
    
    % update optimal breach point
    opt = optCal(dmat_interp, target, matG_interp, g_new, bound);
    opt_list = [opt_list,opt];
    
    % if opt change, recompute reachable set
    if norm(opt - opt_list(:,end-1))>small
        [data,mat,vx,vy,~] = compute_pointdata(opt,params);
        data_f = flip(data,3);
    end
    
    % End
    dist = norm(xs(1,:)-xs(2,:));
    if dist < small
       disp('Attacker been caught!') 
       break
    end
    
    iter = iter+1;
    drawnow;
end

%% save data for video
save('data/video_1v1_point', 'bound', 'dmat_list', 'g', 'g_new','params',...
     'x0_list','X','Y','xg','yg', 'target','matA_list','matD_list','obstacles',...
     'opt_list','tau','vx','vy','xa','xd','xsave')


