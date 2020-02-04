function [mat,ind] = interpolate_mat(mat_save,x0_list,xvec,yvec,xD)

% dominance region
% xD = [-1;-.5];

% interpolate
indx = find(xvec <= xD(1),1,'last');
indy = find(yvec <= xD(2),1,'last');

dx1 = xD(1) - xvec(indx);
dx2 = xvec(indx+1) - xD(1);
dy1 = xD(2) - yvec(indy);
dy2 = yvec(indy+1) - xD(2);

wx = [dx2;dx1]/(dx1+dx2);
wy = [dy2;dy1]/(dy1+dy2);

mat = zeros(size(mat_save(:,:,1)));
ind = zeros(2,2);
for ii = 1:2
    for jj = 1:2
        [~,ind(ii,jj),~] = intersect(x0_list',[xvec(indx+ii-1);yvec(indy+jj-1)]','rows');
        mat = mat + wx(ii)*wy(jj)*mat_save(:,:,ind(ii,jj));
        %vx = vx + wx(ii)*wy(jj)*vx_save(:,:,ind(ii,jj));
        %vy = vy + wx(ii)*wy(jj)*vy_save(:,:,ind(ii,jj));
    end
end