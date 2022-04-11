% NOTE: Requires readObj.m and child_cam_frame.obj file to be in the same folder as this script

% Import obj file and plot it without axes or grid
obj = readObj('child_cam_frame.obj');
x = obj.v(:,1);
z = obj.v(:,2);
z = z*-1;
y = obj.v(:,3);
T = obj.f.v;
trimesh(T,x,y,z,'EdgeColor','k','EdgeAlpha',0.1,'FaceAlpha',0.5);
grid off;
axis off;

% Create small sphere to plot as keypoints
hold on;
[X,Y,Z] = sphere;
r = 0.01;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

% Plot keypoint: HEAD
kpx = 14.23220266;  
kpz = -0.18251431;
kpy = 15.02725569;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Plot keypoint: TORSO
kpx = 14.22773599;  
kpz = -0.40288901;
kpy = 15.00293972;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Plot keypoint: R ARM
kpx = 14.36541668;  
kpz = -0.47924066;
kpy = 14.94473699;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Plot keypoint: L ARM
kpx = 14.05887308;  
kpz = -0.48560548;
kpy = 14.99242187;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Plot keypoint: R LEG
kpx = 14.31005758;  
kpz = -0.88257858;
kpy = 15.00951936;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Plot keypoint: L LEG
kpx = 14.21365938;  
kpz = -0.83052474;
kpy = 15.18158017;
surf(X2+kpx,Y2+kpy,Z2+kpz,'EdgeColor','r','FaceColor','r');

% Change viewpoint of figure to make it easier to view by default 
caz = 135;
cel = 1;
view(caz,cel);
