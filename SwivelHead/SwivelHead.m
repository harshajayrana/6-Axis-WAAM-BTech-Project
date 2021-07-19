clc;
clear;

%theta d a alpha
l(1) = Link([0 0 0 pi/2 1]);
l(1).qlim = [0 100];
l(2) = Link([0 0 0 pi/2 0]);
l(2).qlim = [7*pi/6 11*pi/6];
l(3) = Link([0 0 20 pi/2 0]);


trobot = SerialLink(l);
trobot.name = "tRobot";
trobot.base = trotz(0)*transl(0, 0, 0);
%trobot.teach();

%trobot.plot([0, 0, 0]);
hold on;

td1=0:2:100;
tth2=7*pi/6:pi/24:11*pi/6;
tth3 = 0;

[tD1, tT2, tT3]=ndgrid(td1, tth2, tth3);
datat = trobot.fkine([tD1(:),tT2(:),tT3(:)]);
DATAt = transl(datat);
size(DATAt)

scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3),35, ".")
axis equal;


%theta d a alpha
d(1) = Link([0 0 0 -pi/2 0]);
d(2) = Link([pi/2 0 0 pi/2 1]);
d(2).qlim = [0 100];
d(3) = Link([-pi/2 0 0 pi/2 1]);
d(3).qlim = [0 100];
d(4) = Link([0 0 0 -pi/2 0]);
d(5) = Link([0 0 0 pi/2 1]);
d(5).qlim = [0 20];
d(6) = Link([0 0 0 0 1]);
d(6).qlim = [0 20];


robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(-50, -50, 0);
%robot.teach();

%robot.plot([0, 0, 0, 0, 0, 0]);

th1=0;
d2=0:30:100;
d3=0:30:100;
th4=0:pi/6:2*pi;
d5=0:2:20;
d6=0:2:20;


[T1, D2, D3, T4, D5, D6]=ndgrid(th1, d2, d3, th4, d5, d6);
data = robot.fkine([T1(:),D2(:),D3(:),T4(:),D5(:),D6(:)]);
DATA = transl(data);
size(DATA)

scatter3(DATA(:,1),DATA(:,2),DATA(:,3),15, ".")