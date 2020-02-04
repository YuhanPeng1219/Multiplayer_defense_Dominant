% clear all
% load('data/sim_1vs1.mat');
load('data/video_1v1_point.mat')
% load('data/video_1v1_rect_stable.mat')
%% =================== Video Obj ================================
% save video
pic_num =1;
vobj=VideoWriter('video\sim1v1', 'MPEG-4');
vobj.FrameRate=34;
vobj.Quality=100;
open(vobj);

%% =================== Map Generation ============================
sim = figure(); axis equal; hold on
cc = lines(2);
% obstacles
contour(X,Y,obstacles,[0 0],'k','linewidth',2)

% target
if size(target,1)==1
    % point target
    plot(target(1),target(2),'*','markerfacecolor','y','markersize',10)
else
    % other shape
    contour(X,Y,target,[0 0],'r','linewidth',5)
end

% players
hd = plot(xsave{1}(1,1),xsave{1}(1,2),'ks','markerfacecolor','b','markersize',10);
ha = plot(xsave{2}(1,1),xsave{2}(1,2),'k^','markerfacecolor','r','markersize',10);

%% =================== Dominant Reigon =========================
% Dominant region
[bound,hdomin] = contour(g_new.xs{1},g_new.xs{2},dmat_list{2},[0,0],'k','linewidth',3);
% Isochrones
matD(dmat_list{1}>0|obstacles<0) = nan;
matA(dmat_list{1}<0|obstacles<0) = nan;
% reaching time contours
[~,hdc] = contour(g_new.xs{1},g_new.xs{2},matD_list{1},20,'color',cc(1,:));
[~,hac] = contour(g_new.xs{1},g_new.xs{2},matA_list{1},20,'color',cc(2,:));

%% ============== Optimal breach point =====================
% players
hb = plot(opt_list(1,1),opt_list(2,1),'d','markerfacecolor','w','markersize',10);

%% ====================== Main Loop ===============================
for i = 2:length(opt_list)
    % update 
    set(hb, 'XData', opt_list(1,i), 'YData', opt_list(2,i));
    set(hd, 'XData', xsave{1}(i,1), 'YData', xsave{1}(i,2));
    set(ha, 'XData', xsave{2}(i,1), 'YData', xsave{2}(i,2));
    set(hdomin, 'ZData', dmat_list{i});
    % draw isochrones
    set(hdc, 'ZData', matD_list{i});
    set(hac, 'ZData', matA_list{i});
   
    drawnow;
    
    % save video
    F=getframe(sim);
    writeVideo(vobj, F);
    pic_num = pic_num + 1;
end

close(vobj);

