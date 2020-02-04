clear all

% load data_N100_mesh64.mat
% load data_N100_mesh3600.mat
load data_N100_mesh10000.mat

%%
obstacles = params.obstacles;

xvec = xg;
yvec = yg;

nu = 0.8;

% xD = [-1.5;-1.9];
xD = [-.7;0];
[matD,indD] = interpolate_mat(mat_save,x0_list,xvec,yvec,xD);

xA = [0;-1.7];
[matA,indA] = interpolate_mat(mat_save,x0_list,xvec,yvec,xA);

matA = matA/nu;
dmat = matD-matA;
dmat(obstacles<0) = nan;

matD(dmat>0) = nan;
matA(dmat<0) = nan;


% vx = diff(mat,[],1);
% vy = diff(mat,[],2);
% % [vx,vy] = gradient(mat,1,1);
% vx(end+1,:) = vx(end,:);
% vy(:,end+1) = vy(:,end);
% 
% vnorm = sqrt(vx.^2 + vy.^2);
% vx = vx./vnorm;
% vy = vy./vnorm;

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
figure(); axis equal; hold on; box on; 
set(gcf,'color','w')
figsize(.6,.8)

a = sum(sum(mat_save,1),2);
indpos = find(a>0);
% quiver(X,Y,vx,vy)
% plot(x0_list(1,indpos),x0_list(2,indpos),'.')

% agents
plot(xD(1),xD(2),'ks','markerfacecolor','b','markersize',10)
plot(xA(1),xA(2),'k^','markerfacecolor','r','markersize',10)


% boundary
contour(X,Y,dmat,[0,0],'k','linewidth',3)

% reaching time contours
tlist  = linspace(0,5,20);
contour(X,Y,matD,tlist,'color',cc(1,:))
contour(X,Y,matA,tlist,'color',cc(2,:))

% obstacles
% [co,ho] = contour(X,Y,obstacles,[0 0],'k','linewidth',2);

% % sample points
% for ii = 1:4
%     plot(x0_list(1,ind(ii)),x0_list(2,ind(ii)),'or')
% end
 
% Stream line
% [xm,ym] = meshgrid(linspace(-2,2,20),linspace(-2,2,20));
% pstart = [xm(:)';ym(:)'];
% pstart = [0;0];
% h = streamline(X',Y',-vx',-vy',pstart(1,:),pstart(2,:));
% set(h,'linewidth',1,'color','r')

% boundary
% contour(X,Y,dmat,[0,0],'k','linewidth',3)
 
% obstacles
[co,ho] = contour(X,Y,obstacles,[0 0],'k','linewidth',2);
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

% title(sprintf('\\nu = %.1f',nu),'fontsize',15)

set(gca,'xtick',[])
set(gca,'ytick',[])
box on
set(gcf,'renderer','painter')
export_pdf(gcf,['dominance_region','-nu',num2str(nu*10)])

% export_png(gcf,['dominance_region','-nu',num2str(nu*10)])
% uistack(hobst,'top')