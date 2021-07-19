clear
clc
hold on;

%theta d a alpha
t(1) = Link([0 0 0 -pi/2 1]);
t(1).qlim = [0 500];
t(2) = Link([pi/2 0 0.3 pi/2 1]);
t(2).qlim = [0 600];
t(3) = Link([0 0 0.3 0 1]);
t(3).qlim = [0 650];

robot1 = SerialLink(t);
robot1.name = "Robot1";
robot1.base = trotz(0)*transl(-300, -325, 0);
%robot1.teach();

%robot1.plot([500 600 650]);

d1=0:25:500;
d2=0:30:600;
d3=0:30:650;


[D1,D2,D3]=ndgrid(d1,d2,d3);
datat = robot1.fkine([D1(:),D2(:),D3(:)]);
DATAt = transl(datat);
size(DATAt)

scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3), 8, ".")

axis equal
shading interp
hold on;
k = boundary(DATAt);
hold on;
trisurf(k,DATAt(:,1),DATAt(:,2),DATAt(:,3),'Facecolor','blue','FaceAlpha',0.1,'LineStyle','none')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
hold on;

%theta d a alpha
d(1) = Link([0 0 0 pi/2 0]);
d(2) = Link([0 0 0 -pi/2 0]);
d(2).qlim = [-0.575*pi 0.15*pi];
d(3) = Link([0 0 0 pi/2 0]);
d(4) = Link([0 0 0 -pi/2 1]);
d(4).qlim = [0 315];
d(5) = Link([0 0 0 0 1]);
d(5).qlim = [0 400];


robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(0, 0, 0);
%robot.teach();

robot.plot([0, 0, 0, 0, 0]);

th1=0;
th2=-0.5*pi:pi/10:0.5*pi;
th3=-0:0.2:2*pi;
d4=0:30:315;
d5=0:40:400;


[T1, T2, T3, D4, D5]=ndgrid(th1, th2, th3, d4, d5);
data = robot.fkine([T1(:),T2(:),T3(:),D4(:),D5(:)]);
DATA = transl(data);
size(DATA)

scatter3(DATA(:,1),DATA(:,2),DATA(:,3), 10, ".")


axis equal
shading interp
hold on;
k = boundary(DATA);
hold on;
trisurf(k,DATA(:,1),DATA(:,2),DATA(:,3),'Facecolor','red','FaceAlpha',0.1,'LineStyle','none')
