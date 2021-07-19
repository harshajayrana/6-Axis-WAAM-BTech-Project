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


syms th2 th3;

% Ru = [cosd(th3), -sind(th3), 0;
%     sind(th3),cosd(th3), 0;
%     0, 0, 1];

Rv = [cosd(th2), 0, sind(th2);
    0, 1, 0;
    -sind(th2), 0, cosd(th2)];

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


td1=20:2:100;
tth2=9*pi/6:pi/24:2*pi;
td3 = 20;
td4 = -20:10:30;

for i = 1:length(td1)
    for j= 1:length(tth2)
        for k= 1:length(td3)
            for l= 1:length(td4)
                data = trobot.fkine([td1(i),tth2(j),td3(k),td4(l)]);
                DATA = transl(data);
                M = subs(mew, {th2}, {(180/pi)*tth2(j)});
                m = double(M);
                scatter3(DATA(1,1),DATA(1,2),DATA(1,3), 40 ,m*10,'.','MarkerFaceAlpha',0.2,'MarkerEdgeAlpha',0.2);
                hold on;
            end
        end
    end
end
        

% [tD1, tT2, tD3, tD4]=ndgrid(td1, tth2, td3, td4);
% datat = trobot.fkine([tD1(:),tT2(:),tD3(:),tD4(:)]);
% DATAt = transl(datat);
% size(DATAt)
% 
% scatter3(DATAt(:,1),DATAt(:,2),DATAt(:,3),35, ".")
% 
% axis equal;
% shading interp
% hold on;
% kt = boundary(DATAt);
% hold on;
% trisurf(kt,DATAt(:,1),DATAt(:,2),DATAt(:,3),'Facecolor','blue','FaceAlpha',0.1,'LineStyle','none')
