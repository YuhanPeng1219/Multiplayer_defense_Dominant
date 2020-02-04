function opt = optCal(dmat, t_bound, matG_interp, g_new,bound)
value = eval_u(g_new, dmat, t_bound);
if all(value<0)
    disp('Attacker cannot reach')
    % interpolate
%     [matG,~] = interpolate_mat(mat_save,x0_list,xg,yg,Goal);
%     matG(obstacles<0) = nan;
%     
%     [matG_interp, g_new] = gridInterpolation(g, matG, 400);
    
    t_v = eval_u(g_new, matG_interp, bound);
    
    [~, ind] = min(t_v);
    opt = bound(:,ind);
else
    disp('Attacker can reach')
    [~, ind] = max(value);
    opt = t_bound(:,ind);
end
end