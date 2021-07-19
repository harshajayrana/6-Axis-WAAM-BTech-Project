clc;
clear;
%theta d a alpha
l(1) = Link([0 0 0.4 0 0]);
l(2) = Link([0 0 0.25 pi 0]);
l(3) = Link([0 0 0 0 1]);
l(3).qlim = [0, 0.15];
l(4) = Link([0 0 0.15 0 0]);

robot = SerialLink(l);
robot.name = "Scara";
robot.base = trotz(0)*transl(0, 0, 0.5);
% robot.teach();

%robot.plot([0, 0, 0, 0]);
hold on;

syms th1 th2 d3 th4 XE YE ZE NX NY

T01 = [cos(th1), -sin(th1), 0, 0.4*cos(th1);
    sin(th1), cos(th1), 0, 0.4*sin(th1);
    0, 0, 1, 0;
    0, 0, 0, 1];

T12 = [cos(th2), sin(th2), 0, 0.25*cos(th2);
    sin(th2), -cos(th2), 0, 0.25*sin(th2);
    0, 0, -1, 0;
    0, 0, 0, 1];

T23 = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d3-0.5;
    0, 0, 0, 1];

T34 = [cos(th4), -sin(th4), 0, 0.15*cos(th4);
    sin(th4), cos(th4), 0, 0.15*sin(th4);
    0, 0, 1, 0;
    0, 0, 0, 1];

T02 = T01*T12;
T03 = T02*T23;
T04 = simplify(T03*T34)

XE_RHS = (3*NX)/20 + cos(th1 + th2)/4 + (2*cos(th1))/5
YE_RHS = (3*NY)/20 + sin(th1 + th2)/4 + (2*sin(th1))/5
ZE_RHS = - d3 + 1/2


XE_MLF = matlabFunction(XE_RHS,'Vars',[th1 th2 NX]);
YE_MLF = matlabFunction(YE_RHS,'Vars',[th1 th2 NY]);
ZE_MLF = matlabFunction(ZE_RHS,'Vars',[d3]);


XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;
ZE_EQ = ZE == ZE_RHS;

S = solve([XE_EQ YE_EQ ZE_EQ], [th1 th2 d3]);
simplify(S.th1)
simplify(S.th2)
simplify(S.d3)

global TH1_MLF TH2_MLF D3_MLF dexterity;
TH1_MLF{1} = matlabFunction(S.th1(1),'Vars',[XE YE NX NY]);
TH1_MLF{2} = matlabFunction(S.th1(2),'Vars',[XE YE NX NY]);
TH2_MLF{1} = matlabFunction(S.th2(1),'Vars',[XE YE NX NY]);
TH2_MLF{2} = matlabFunction(S.th2(2),'Vars',[XE YE NX NY]);
D3_MLF{1} = matlabFunction(S.d3(1),'Vars',[ZE]);
D3_MLF{2} = matlabFunction(S.d3(2),'Vars',[ZE]);

T = robot.fkine([pi/3, -pi/6, 0.3, pi/9])
size(T)

TH1 = -(110*pi/180):pi/6:pi/2;
TH2 = 0:pi/6:(115*pi/180);
D3= 0:0.03:0.15;
TH4 = 0:pi/6:2*pi;
axis equal;
for i = 1:length(TH1)
    for j= 1:length(TH2)
        for k= 1:length(D3)
            for l= 1:length(TH4)
                global dexterity;
                data = robot.fkine([TH1(i),TH2(j),D3(k),TH4(l)]);
                DATAt = transl(data);
                dex = dext(DATAt(1,1),DATAt(1,2),DATAt(1,3))
                POINTS = [DATAt(1,1) DATAt(1,2) DATAt(1,3) dex];
                dexterity = vertcat(dexterity, POINTS);
                %scatter3(DATAt(1,1),DATAt(1,2),DATAt(1,3), 40 ,dex,'.','MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
                scatter3(DATAt(1,1),DATAt(1,2),dex, '.', 'r');
                hold on;
            end
        end
    end
end
%xlswrite('DexterityPlot.xls',dexterity);


function f = dext(X, Y, Z)
    global TH1_MLF TH2_MLF D3_MLF f;
    phi = 0:pi/36:2*pi;
    f = 0;
    for n = 1:length(phi)
        NX = cos(phi(n));
        NY = sin(phi(n));
        theta11 = rad2deg(TH1_MLF{1}(X, Y, NX, NY));
        theta12 = rad2deg(TH1_MLF{2}(X, Y, NX, NY));
        theta21 = -rad2deg(TH2_MLF{1}(X, Y, NX, NY));
        theta22 = rad2deg(TH2_MLF{2}(X, Y, NX, NY));

        d31 = D3_MLF{1}(Z);
        d32 = D3_MLF{2}(Z);

        theta41 = theta11 + theta21 - rad2deg(atan(NY/NX));
        theta42 = theta12 + theta22 - rad2deg(atan(NY/NX)); 
        
        if isreal(theta11) && isreal(theta12) && isreal(theta21) && isreal(theta22) && isreal(d31) && isreal(d32)
            if theta11<=90.0000 && theta11>=-110.0000 && theta12<=90 && theta12>=-110 && theta21<=115 && theta21>=0 && theta22<=115 && theta22>=0 && d31<=0.15 && d31>=0 && d32<=0.15 && d32>=0
                f = f+1;
            end
        end
    end
    f = f/72;
end

