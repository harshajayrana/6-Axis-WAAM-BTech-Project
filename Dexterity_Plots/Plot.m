clc;
clear;
data =  xlsread('DexterityPlot.xls');
x = data(:,1);
y = data(:,2);
z = data(:,3);
d = data(:,4);

tri = delaunay(x,y);
% plot(x,y,'.')
[r,c] = size(tri);
disp(r)

subplot(2,3,1);
h = trisurf(tri, x, y, d);
axis equal;
% shading interp
% colorbar EastOutside

% figure();
subplot(2,3,2);
% axis equal
% hidden on
trimesh(tri,x,y,d)

% figure();
subplot(2,3,3);
scatter3(x,y,d);

% figure();
subplot(2,3,4);
scatter3(x,y,d, '.');

% figure();
subplot(2,3,5);
plot3(x,y,d);

% figure()
% xv = linspace(min(x), max(x), 300);
% yv = linspace(min(y), max(y), 300);
% [xm, ym] = ndgrid(xv, yv);
% dm = griddata(x, y, d, xm, xm);
% contour3(xm,ym,dm);

figure()
N = 300;
xi = linspace(min(x),max(x),N) ;
yi = linspace(min(y),max(y),N) ;
zi = linspace(min(z),max(z),N) ;
[Xi,Yi] = meshgrid(xi,yi) ;
Zi = repmat(zi,size(Xi)) ;
Di = griddata(x,y,d,Xi,Yi) ;
surf(Xi,Yi,Zi,Di) ;
shading interp



