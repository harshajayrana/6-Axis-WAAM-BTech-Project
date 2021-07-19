clc;
clear;

syms th3;
thc = pi/4;

Sr = [cosd(45), 0, sind(45);
    0, 1, 0;
    -sind(45), 0, cosd(45)];

Sr2 = [cosd(-45), 0, sind(-45);
    0, 1, 0;
    -sind(-45), 0, cosd(-45)];

RV = [cosd(th3), 0, sind(th3);
    0, 1, 0;
    -sind(th3), 0, cosd(th3)];

Rv = Sr*RV*Sr2

psi = [1, 0, 0;
    0, 1, 0];
psit = psi';

Au = [0, -1, 0;
    1, 0, 0;
    0, 0, 0];
Av = [0, 0, 1;
    0, 1, 0;
    -1, 0, 1];

T = [0; 0; 1];

Kv = [psi*Au*Rv*T, psi*Rv*Av*T];

% Hu = psi*Ru*psit;
% 
% Jrr = Hu*Kv;

mew = det(Kv);


%theta d a alpha
l(1) = Link([-pi/2 0 0 -pi/2 1]);
l(1).qlim = [20 60];
l(2) = Link([3*pi/4 0 0 pi/2 1]);
l(2).qlim = [0 60];
l(3) = Link([0 0 25 0 0]);
l(3).qlim = [10*pi/6 14*pi/6];


trobot = SerialLink(l);
trobot.name = "tRobot";
trobot.base = trotz(0)*transl(0, 50, 0);
%trobot.teach();

%trobot.plot([0, 0, 0]);
hold on;

td1 = 20:5:60;
td2 = -50:50:50;
tth3 = -pi/2 : pi/24 : pi/2;

% [tD1, tD2, tT3]=ndgrid(td1, td2, tth3);
% datat = trobot.fkine([tD1(:),tD2(:),tT3(:)]);
% DATAt = transl(datat);
% size(DATAt)

for i = 1:length(td1)
    for j= 1:length(td2)
        for k= 1:length(tth3)
            data = trobot.fkine([td1(i),td2(j),tth3(k)]);
            DATAt = transl(data);
            M = subs(mew, {th3}, {(180/pi)*tth3(k)});
            m = double(M);
            scatter3(DATAt(1,1),DATAt(1,2),DATAt(1,3), 40 ,m*100,'.','MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
            hold on;
        end
    end
end

% scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3),35, ".")
axis equal;
shading interp
hold on;
kt = boundary(DATAt);
hold on;
trisurf(kt,DATAt(:,1),DATAt(:,2),DATAt(:,3),'Facecolor','blue','FaceAlpha',0.1,'LineStyle','none')


%theta d a alpha
d(1) = Link([-pi/2 0 0 -pi/2 0]);
d(2) = Link([0 0 0 pi/2 1]);
d(2).qlim = [0 100];
d(3) = Link([0 0 0 -pi/2 0]);
d(4) = Link([0 0 0 pi/2 1]);
d(4).qlim = [0 20];
d(5) = Link([0 0 0 0 1]);
d(5).qlim = [0 20];


robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(0, 0, 0);
%robot.teach();

%robot.plot([0, 0, 0, 0, 0]);

th1=0;
d2=-30:10:30;
th4=0:pi/6:2*pi;
d5=0:2:20;
d6=0:2:20;


[T1, D2, T4, D5, D6]=ndgrid(th1, d2, th4, d5, d6);
data = robot.fkine([T1(:),D2(:),T4(:),D5(:),D6(:)]);
DATA = transl(data);
size(DATA)

scatter3(DATA(:,1),DATA(:,2),DATA(:,3),15, ".")
shading interp
hold on;
k = boundary(DATA);
hold on;
trisurf(k,DATA(:,1),DATA(:,2),DATA(:,3),'Facecolor','red','FaceAlpha',0.1,'LineStyle','none')