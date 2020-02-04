function [data_new, g_new] = gridInterpolation(g, data, N)
%   Computes the interpolated grid and return new grid structure

% interpolate
X = g.xs{1};
Y = g.xs{2};
xp = linspace(g.min(1),g.max(1),N);
yp = linspace(g.min(2),g.max(2),N);
F_d = griddedInterpolant(X,Y,data);
[xq,yq] = ndgrid(xp,yp);
data_new = F_d(xq,yq);

% update grid structure
g_new = g;
g_new.xs{1} = xq;
g_new.xs{2} = yq;
g_new.vs{1} = xp;
g_new.vs{2} = yp;

end