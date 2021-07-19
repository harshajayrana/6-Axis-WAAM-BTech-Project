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
robot.base = trotz(0)*transl(-0.5, -0.5, 0);
robot.teach();

robot.plot([1 1 1]);

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
t(2) = Link([0 0 0 -pi/2 1]);
t(2).qlim = [0 1];
t(3) = Link([0 0 0 0 1]);
t(3).qlim = [0 1];

global robot2
robot2 = SerialLink(t);
robot2.name = "Table";
robot2.base = trotz(0)*transl(0, 0, 0);
robot2.teach();

robot2.plot([1, 0.5, 0.5]);

global th d2 d3;
th=0:30:360;
d2=0:0.2:1;
d3=0:0.2:1;

% [T1, D2, D3]=ndgrid(th, d2, d3);
% data = robot2.fkine([T1(:), D2(:), D3(:)]);
% DATA = transl(data);
% size(DATA)
% 
% plot3(DATA(:,1),DATA(:,2),DATA(:,3), ".")

global dexterity
for i = 1:length(th)
    for j= 1:length(d2)
        for k= 1:length(d3)
            global dexterity;
            data = robot2.fkine([th(i),d2(j),d3(k)]);
            DATAt = transl(data);
            dex = dext(d2(j), d3(k))
            POINTS = [DATAt(1,1) DATAt(1,2) DATAt(1,3) dex];
            %dexterity = vertcat(dexterity, POINTS);
            scatter3(DATAt(1,1),DATAt(1,2),DATAt(1,3),10,dex,'filled');
            %scatter3(DATAt(1,1),DATAt(1,2),dex, '.', 'r');
            hold on;
        end
    end
end

function f = dext(d2, d3)
    global f robot2
    th=0:30:360;
    f =0;
    for n = 1:length(th)
        data2 = robot2.fkine([th(n),d2,d3]);
        DATA2 = transl(data2);
        X = DATA2(1,1);
        Y = DATA2(1,2);
        Z = DATA2(1,3);
        if X<=0.5 && X>=-0.5 && Y<=0.5 && Y>=-0.5 && Z<=1 && Z>=0
            f = f+1;
        end
    end
    f = f/13;
end
