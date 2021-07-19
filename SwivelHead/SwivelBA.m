clc;
clear;

%theta d a alpha
l(1) = Link([pi/2 0 0 pi/2 1]);
l(1).qlim = [0 100];
l(2) = Link([0 0 0 pi/2 0]);
l(2).qlim = [7*pi/6 11*pi/6];
l(3) = Link([0 0 20 -pi/2 1]);
l(4) = Link([0 0 0 0 1]);


trobot = SerialLink(l);
trobot.name = "tRobot";
trobot.base = trotz(0)*transl(0, 0, 0);
%trobot.teach();

%trobot.plot([0, 0, 0]);
hold on;

td1=20:2:60;
tth2=9*pi/6:pi/24:2*pi;
td3 = 20;
td4 = -20:10:20;

[tD1, tT2, tD3, tD4]=ndgrid(td1, tth2, td3, td4);
datat = trobot.fkine([tD1(:),tT2(:),tD3(:),tD4(:)]);
DATAt = transl(datat);
size(DATAt)

scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3),35, ".")
axis equal;
shading interp
hold on;
kt = boundary(DATAt);
hold on;
trisurf(kt,DATAt(:,1),DATAt(:,2),DATAt(:,3),'Facecolor','blue','FaceAlpha',0.1,'LineStyle','none')

clc;
clear;

%theta d a alpha
d(1) = Link([0 0 0 -pi/2 0]);
d(2) = Link([0 0 0 0 0]);
d(3) = Link([pi/2 0 0 pi/2 1]);
d(3).qlim = [0 100];
d(4) = Link([-pi/2 0 0 pi/2 1]);
d(4).qlim = [0 100];
d(5) = Link([0 0 0 0 1]);


robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(0, 0, 0);
% robot.teach();

% robot.plot([0, 0, 0, 0]);

th1=0;
th2=-pi/6:pi/6:pi/6;
d3=-30:10:30;
d4=-20:10:20;
d5=0:5:20;


[T1, T2, D3, D4, D5]=ndgrid(th1, th2, d3, d4, d5);
data = robot.fkine([T1(:),T2(:),D3(:),D4(:),D5(:)]);
DATA = transl(data);
size(DATA)
hold on;
scatter3(DATA(:,1),DATA(:,2),DATA(:,3),15, ".")
shading interp
hold on;
k = boundary(DATA);
hold on;
trisurf(k,DATA(:,1),DATA(:,2),DATA(:,3),'Facecolor','red','FaceAlpha',0.1,'LineStyle','none')