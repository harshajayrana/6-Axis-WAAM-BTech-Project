clear
clc
hold on;

%theta d a alpha
t(1) = Link([0 0 0 -pi/2 1]);
t(1).qlim = [0 300];
t(2) = Link([pi/2 0 0.3 pi/2 1]);
t(2).qlim = [0 300];
t(3) = Link([0 0 0.3 0 1]);
t(3).qlim = [0 300];

robot1 = SerialLink(t);
robot1.name = "Robot1";
robot1.base = trotz(0)*transl(-150, -150, 0);
%robot1.teach();

%robot1.plot([500 600 650]);

d1=0:30:300;
d2=0:30:300;
d3=0:30:300;


[D1,D2,D3]=ndgrid(d1,d2,d3);
datat = robot1.fkine([D1(:),D2(:),D3(:)]);
DATAt = transl(datat);
size(DATAt)

scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3), 8, ".")
xlabel('Y')
ylabel('X')
zlabel('Z')


axis equal
shading interp
hold on;
k = boundary(DATAt);
hold on;
trisurf(k,DATAt(:,1),DATAt(:,2),DATAt(:,3),'Facecolor','blue','FaceAlpha',0.1,'LineStyle','none')
xlabel('Y')
ylabel('X')
zlabel('Z')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms fh3;

Sr = [cosd(45), 0, sind(45);
    0, 1, 0;
    -sind(45), 0, cosd(45)];

Sr2 = [cosd(-45), 0, sind(-45);
    0, 1, 0;
    -sind(-45), 0, cosd(-45)];

RV = [cosd(fh3), 0, sind(fh3);
    0, 1, 0;
    -sind(fh3), 0, cosd(fh3)];

Rv = Sr*RV*Sr2;

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

hold on;

%theta d a alpha
d(1) = Link([0 0 0 -pi/4 0]);
d(2) = Link([0 0 0 pi/4 0]);
d(3) = Link([0 0 0 -pi/2 0]);
d(4) = Link([0 0 0 pi/2 1]);
d(4).qlim = [0 300];
d(5) = Link([0 0 0 0 1]);
d(5).qlim = [0 300];


robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(0, 0, 0);
robot.teach();

robot.plot([0, 0, 0, 0, 0]);
xlabel('Y')
ylabel('X')
zlabel('Z')

th1=0;
th2=0:pi/2:2*pi;
th3=-0:pi/2:2*pi;
d4=0:10:50;
d5=0:60:300;

for i = 1:length(th1)
    for j= 1:length(th2)
        for k= 1:length(th3)
            for l= 1:length(d4)
                for m= 1:length(d5)
                    data = robot.fkine([th1(i),th2(j),th3(k),d4(l),d5(m)]);
                    DATAt = transl(data);
                    M = subs(mew, {fh3}, {(180/pi)*th2(j)});
                    m = double(M);
                    scatter3(DATAt(1,1),DATAt(1,2),DATAt(1,3), 40 ,m*500,'.','MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
                    hold on;
                end
            end
        end
    end
end

% [T1, T2, T3, D4, D5]=ndgrid(th1, th2, th3, d4, d5);
% data = robot.fkine([T1(:),T2(:),T3(:),D4(:),D5(:)]);
% DATA = transl(data);
% size(DATA)

% scatter3(DATA(:,1),DATA(:,2),DATA(:,3), 10, ".")
% xlabel('Y')
% ylabel('X')
% zlabel('Z')

axis equal
shading interp
hold on;
k = boundary(DATA);
hold on;
trisurf(k,DATA(:,1),DATA(:,2),DATA(:,3),'Facecolor','red','FaceAlpha',0.1,'LineStyle','none')
xlabel('Y')
ylabel('X')
zlabel('Z')

