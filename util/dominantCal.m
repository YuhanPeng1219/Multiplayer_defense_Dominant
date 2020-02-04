function [dmat_interp, dmat, matD_interp, matA_interp, bound] = dominantCal(xd, xa, mat_save, x0_list, g, xg, yg, h, obstacles, Interp_N)

[matD,~] = interpolate_mat(mat_save,x0_list,xg,yg,xd);
[matA,~] = interpolate_mat(mat_save,x0_list,xg,yg,xa);

[matD_interp, ~] = gridInterpolation(g, matD, Interp_N);
[matA_interp, ~] = gridInterpolation(g, matA, Interp_N);

dmat = matD-matA;
dmat(obstacles<0) = nan;

% Dominant region
[dmat_interp, ~] = gridInterpolation(g, dmat, Interp_N);
[bound,~] = contour(g.xs{1},g.xs{2},dmat,[0,0],'k','linewidth',3, 'Visible', 'off');
set(h, 'ZData', dmat_interp);
bound(:,1) = [];
end