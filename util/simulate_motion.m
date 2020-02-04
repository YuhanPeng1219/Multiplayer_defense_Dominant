function simulate_motion
% clear all

% load data_N100_mesh64.mat
% load data_N100_mesh3600.mat
load data_N100_mesh10000.mat

%%
obstacles = params.obstacles;

xvec = xg;
yvec = yg;
% dominance region
xD = [-2;-2];

vx = diff(mat,[],1);
vy = diff(mat,[],2);
vx(end+1,:) = vx(end,:);
vy(:,end+1) = vy(:,end);

vnorm = sqrt(vx.^2 + vy.^2);
vx = vx./vnorm;
vy = vy./vnorm;

% V = wx' * [f11,f12; f21,f22] * wy
% V = wx(1)*wy(1)*f11+ wx(1)*wy(2)*f12+wx(2)*wy(1)*f21+wx(2)*wy(2)*f22
% Vq = interp2(X',Y',obstacles',xD(1),xD(2))
% idx = [1,2];
% nu  = 1;
% matD = mat_save(:,:,idx(1));
% matA = mat_save(:,:,idx(2))/nu;
% xD = x0_list(:,idx(1));
% xA = x0_list(:,idx(2));
% 
% dmat = matD-matA;
% dmat(obstacles<0) = nan;
% 
% matD(dmat>0) = nan;
% matA(dmat<0) = nan;

% PLOT
cc = lines(2);
figure(); axis equal; hold on

% quiver(X,Y,vx,vy)
% plot(x0_list(1,:),x0_list(2,:),'.')

% agents
plot(xD(1),xD(2),'ks','markerfacecolor','b','markersize',10)
% sample points
for ii = 1:4
    plot(x0_list(1,ind(ii)),x0_list(2,ind(ii)),'or')
end

% Stream line
% [xm,ym] = meshgrid(linspace(-2,2,20),linspace(-2,2,20));
% pstart = [xm(:)';ym(:)'];
pstart = [0;0];
h = streamline(X',Y',-vx',-vy',pstart(1,:),pstart(2,:));
set(h,'linewidth',1,'color','r')


% plot(xA(1),xA(2),'k^','markerfacecolor','r','markersize',10)

% boundary
% contour(X,Y,dmat,[0,0],'k','linewidth',3)
% 
% reaching time contours
contour(X,Y,mat,20,'color',cc(1,:))
% contour(X,Y,matA,20,'color',cc(2,:))
% 
% obstacles
[co,ho] = contour(X,Y,obstacles,[0 0],'k','linewidth',2);
% co(co<1)= nan;
% figure()
tt = 1;
ii = 1;
while true
    ts = tt+1;
    te = tt+co(2,tt);
    hobst(ii) = patch(co(1,ts:te),co(2,ts:te),[1,1,1]*0.8);
    ii = ii+1;
    tt = te+1;
    if tt >= size(co,2)
        break
    end
end

uistack(hobst,'bottom')


function vOUT = bilinear_interp(xvec,yvec,x0_list,x1,y1,vIN)
vOUT = zeros(size(vIN));
[w,idx] = interp_weight(xvec,yvec,x0_list,x1,y1);
for ii = 1:2
    for jj = 1:2
        vOUT = vOUT + w(ii)*wy(jj)*mat_save(:,:,idx(ii,jj));
        %vx = vx + wx(ii)*wy(jj)*vx_save(:,:,ind(ii,jj));
        %vy = vy + wx(ii)*wy(jj)*vy_save(:,:,ind(ii,jj));
    end
end

function [w,idx] = interp_weight(xvec,yvec,x0_list,x1,y1)
% interpolate
indx = find(xvec <= x1,1,'last');
indy = find(yvec <= y1,1,'last');

dx1 = x1 - xvec(indx);
dx2 = xvec(indx+1) - x1;
dy1 = y1 - yvec(indy);
dy2 = yvec(indy+1) - y1;
w(1) = [dx2;dx1]/(dx1+dx2);
w(2) = [dy2;dy1]/(dy1+dy2);
idx = zeros(2,2);
for ii = 1:2
    for jj = 1:2
        [~,idx(ii,jj),~] = intersect(x0_list',[xvec(indx+ii-1);yvec(indy+jj-1)]','rows');
    end
end
