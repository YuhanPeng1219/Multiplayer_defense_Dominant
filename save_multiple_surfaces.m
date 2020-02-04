clear all

%% parameters
grid_min = [-2;-2];
grid_max = [ 2; 2];
N        = [100;100];
g = createGrid(grid_min, grid_max, N);

tMax     = 7;
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

obstacles(obstacles<-0.05) = -.1;


%%

xg      = linspace(grid_min(1),grid_max(1),100);
yg      = linspace(grid_min(2),grid_max(2),100);
[xm,ym] = meshgrid(xg,yg);

dx       = xg(2)-xg(1); % initial box size


% x0_list = {[0.1;0.7],[-1.5;-0.5]};
x0_list = [xm(:)';ym(:)'];
n       = size(x0_list,2);

%%
params = v2struct(g,tMax,dt,obstacles,dx);

mat_save= zeros(N(1),N(2),n);
vx_save= zeros(N(1),N(2),n);
vy_save= zeros(N(1),N(2),n);

kk = 1;
parfor ii = 1:size(x0_list,2)
    x0 = x0_list(:,ii);
    x0box = x0+[1 1 -1 -1; 1 -1 1 -1];
    %Vq = interp2(X',Y',obstacles',x0(1)',x0(2)')
    Vq = interp2(X',Y',obstacles',x0box(1,:)',x0box(2,:)');
    
    if any(Vq>0)
        [data,mat,vx,vy] = compute_pointdata(x0,params);
    else
        disp('invalid initial condition')
        data = zeros(N(1),N(2));
        mat = zeros(size(data));
        vx  = zeros(size(data));
        vy  = zeros(size(data));
    end        
    mat_save(:,:,ii) = mat;
    vx_save(:,:,ii) = vx;
    vy_save(:,:,ii) = vy;
end


%%
% str_name = sprintf('data_N%.0f_mesh%.0f',N(1),numel(xm));
str_name = sprintf('data_new_10000');
save(str_name,'mat_save','vx_save','vy_save','params','x0_list','X','Y','xg','yg')

%%
load('data/data_new_10000.mat');
figure(); axis equal; hold on

contourf(X,Y,obstacles,[0 0],'k','linewidth',2)
% mat(mat<1) = nan;
contourf(X,Y,mat_save(:,:,2),20)


%% dominance region
idx = [10,20];
nu  = 1;

matD = mat_save(:,:,idx(1));
matA = mat_save(:,:,idx(2))/nu;
xD = x0_list(:,idx(1));
xA = x0_list(:,idx(2));

dmat = matD-matA;
dmat(obstacles<0) = nan;

matD(dmat>0) = nan;
matA(dmat<0) = nan;

% PLOT
cc = lines(2);
figure(); axis equal; hold on

% agents
% xD = [-1.5,1];
% xA = [1,-1.5];
plot(xD(1),xD(2),'ks','markerfacecolor','b','markersize',10)
plot(xA(1),xA(2),'k^','markerfacecolor','r','markersize',10)

% plot(x0_list(1,:),x0_list(2,:),'.')

% boundary
contour(X,Y,dmat,[0,0],'k','linewidth',3)

% reaching time contours
contour(X,Y,matD,20,'color',cc(1,:))
contour(X,Y,matA,20,'color',cc(2,:))

% obstacles
[co,ho] = contour(X,Y,obstacles,[0 0],'k','linewidth',2);
% co(co<1)= nan;
% figure()
% tt = 1;
% while true
%     ts = tt+1;
%     te = tt+co(2,tt);
%     co(2,tt)
%     patch(co(1,ts:te),co(2,ts:te),[1,1,1]*0.8);
%     tt = te+1;
%     if tt >= size(co,2)
%         break
%     end
% end
% plot(co(1,2:end),co(2,2:end),'g')
% set(ccc(1),'facecolor','r')

