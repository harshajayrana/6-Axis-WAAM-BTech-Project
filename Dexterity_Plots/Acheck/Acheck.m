clear
clc
hold on;

%theta d a alpha
d(1) = Link([0 0.1 0 -pi/2 1]);
d(1).qlim = [0 1];
d(2) = Link([pi/2 0 0 pi/2 1]);
d(2).qlim = [0 1];
d(3) = Link([0 0 0 0 1]);
d(3).qlim = [0 1];

robot = SerialLink(d);
robot.name = "Robot";
robot.base = trotz(0)*transl(-0.5, -1, 0);
%robot.teach();

%robot.plot([1 1 1]);

hold on;

% dt1=0:0.1:1;
% dt2=0:0.1:1;
% dt3=0:0.1:1;
% 
% 
% [D1,D2,D3]=ndgrid(dt1,dt2,dt3);
% data = robot.fkine([D1(:),D2(:),D3(:)]);
% DATA = transl(data);
% size(DATA)
% 
% plot3(DATA(:,1),DATA(:,2),DATA(:,3), ".")

%theta d a alpha
t(1) = Link([0 0.3 0 pi/2 0]);
t(2) = Link([0 0.5 0 -pi/2 0]);
t(2).qlim = [-pi/6 pi/6];
t(3) = Link([0 0.3 0 pi/2 0]);
t(4) = Link([0 0 0 -pi/2 1]);
t(4).qlim = [0 0.6];
t(5) = Link([0 0 0 0 1]);
t(5).qlim = [0 0.6];

global robot2
robot2 = SerialLink(t);
robot2.name = "Table";
robot2.base = trotz(0)*transl(0, 0, 0);
%robot2.teach();

%robot2.plot([0, 0, 0, 0.2, 0.2]);

global th1 th2 th3 d4 d5;
th1=0;
th2=0;
th3=-0:0.1:2*pi;
d4=0:0.1:0.6;
d5=0:0.1:0.6;

axis equal;
%global dexterity
for j = 1:length(th2)
    for k = 1:length(th3)
        for r= 1:length(d4)
            for m= 1:length(d5)
                %global dexterity;
                data = robot2.fkine([0,th2(j),th3(k),d4(r),d5(m)]);
                DATAt = transl(data);
                dex = dext(th3(k),d4(r),d5(m))
                POINTS = [DATAt(1,1) DATAt(1,2) DATAt(1,3) dex];
                %dexterity = vertcat(dexterity, POINTS);
                scatter3(DATAt(1,1),DATAt(1,2),DATAt(1,3),10,dex,'filled');
                %scatter3(DATAt(1,1),DATAt(1,2),dex, '.', 'r');
                hold on;
            end
        end
    end
end

function f = dext(th3, d4, d5)
    global f robot2 
    f =0;
    th2=-pi/6:pi/128:pi/6;
    for n = 1:length(th2)
        data = robot2.fkine([0,th2(n),th3,d4,d5]);
        DATAt = transl(data);
        X = DATAt(1,1);
        Y = DATAt(1,2);
        Z = DATAt(1,3);
        if X<=0.5 && X>=-0.5 && Y<=0.5 && Y>=-0.5 && Z<=1 && Z>=0
            f = f+1;
        end
    end
    f = f/43 ;
end
