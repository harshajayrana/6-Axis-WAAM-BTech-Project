clc;
clear;

global l1 l2 l3 l4 l5 center J;
l1 = 5;
l2 = 5;
l3 = 5;
l4 = 5;
l5 = 5;
center = l5/2;

% xlim([-4 10])
% ylim([0 10])

xx = [];
yy = [];
meow = [];
syms th1 th4

CX = l3*cosd(2*atand((l1*sind(th1) - l4*sind(th4) - sqrt((l1*sind(th1) - l4*sind(th4))^2 + (l1*cosd(th1) - l4*cosd(th4) - l5)^2 - (((l1)^2 - (l2)^2 + (l3)^2 + (l4)^2 + (l5)^2)/(2*l3) + (l4*l5*cosd(th4))/(l3) -  (l1*l5*cosd(th1))/(l3) - (l1*l4)*(sind(th4)*sind(th1) + cosd(th4)*cosd(th1))/(l3))^2))/(l1*cosd(th1) - l4*cosd(th4) - l5 + ((l1)^2 - (l2)^2 + (l3)^2 + (l4)^2 + (l5)^2)/(2*l3) + (l4*l5*cosd(th4))/(l3) -  (l1*l5*cosd(th1))/(l3) - (l1*l4)*(sind(th4)*sind(th1) + cosd(th4)*cosd(th1))/(l3)))) + l4*cosd(th4) + l5;
CY = l3*sind(2*atand((l1*sind(th1) - l4*sind(th4) - sqrt((l1*sind(th1) - l4*sind(th4))^2 + (l1*cosd(th1) - l4*cosd(th4) - l5)^2 - (((l1)^2 - (l2)^2 + (l3)^2 + (l4)^2 + (l5)^2)/(2*l3) + (l4*l5*cosd(th4))/(l3) -  (l1*l5*cosd(th1))/(l3) - (l1*l4)*(sind(th4)*sind(th1) + cosd(th4)*cosd(th1))/(l3))^2))/(l1*cosd(th1) - l4*cosd(th4) - l5 + ((l1)^2 - (l2)^2 + (l3)^2 + (l4)^2 + (l5)^2)/(2*l3) + (l4*l5*cosd(th4))/(l3) -  (l1*l5*cosd(th1))/(l3) - (l1*l4)*(sind(th4)*sind(th1) + cosd(th4)*cosd(th1))/(l3)))) + l4*sind(th4);
J = ([diff(CX,th1), diff(CX,th4);
    diff(CY,th1), diff(CY,th4)]);


fh1 = 0:5:180;
fh4 = 0:5:180;

for i = 1:length(fh1)
    for j = 1:length(fh4)
        syms th1 th4
        fh1(i);
        fh4(j);
        [th311, th322] = theta3(fh1(i),fh4(j));
        if isreal(th311) == 1 && isreal(th322) == 1
            [th211, th222] = theta2(fh1(i),fh4(j),th311,th322);
        end
         if isreal(th311) == 1 && isreal(th322) == 1 ...
                 && isreal(th211) == 1 && isreal(th222) == 1 ...
                 && (fh1(i)-th222)>= 0 && (th322-fh4(j))>= 0 ...
                 && (th322-th222)<= 180 && (th311-th211)<= 180
%              [th211, th222] = theta2(fh1(i),fh4(j),th311,th322);
% %             if isreal(th211) == 1 
%                 OA = [0,0] ;
%                 OB = [l1*cosd(fh1(i)), l1*sind(fh1(i))] ;
%                 OC = [l5 + l4*cosd(fh4(j)) + l3*cosd(th311), l4*sind(fh4(j)) + l3*sind(th311)] ;
%                 OD = [l5 + l4*cosd(fh4(j)), l4*sind(fh4(j))] ;
%                 OE = [l5, 0] ;
%                 
%                 ABx = [OA(1,1) OB(1,1)];
%                 ABy = [OA(1,2) OB(1,2)];
%                 DEx = [OD(1,1) OE(1,1)];
%                 DEy = [OD(1,2) OE(1,2)];
%                 [xiABDE, yiABDE] = polyxpoly(ABx, ABy, DEx, DEy);
%                 
%                 BCx = [OB(1,1) OC(1,1)];
%                 BCy = [OB(1,2) OC(1,2)];
%                 DEx = [OD(1,1) OE(1,1)];
%                 DEy = [OD(1,2) OE(1,2)];
%                 [xiBCDE, yiBCDE] = polyxpoly(BCx, BCy, DEx, DEy);
%                                 
%                 CDx = [OC(1,1) OD(1,1)];
%                 CDy = [OC(1,2) OD(1,2)];
%                 ABx = [OA(1,1) OB(1,1)];
%                 ABy = [OA(1,2) OB(1,2)];
%                 [xiCDAB, yiCDAB] = polyxpoly(CDx, CDy, ABx, ABy);
%                 
%                 if isempty(xiABDE) == 0 && isempty(yiABDE) == 0 && length(xiABDE) == 1 && length(yiABDE) == 1
%                     diA0 = round(sqrt((xiABDE-OA(1,1))^2 + (yiABDE-OA(1,2))^2));
%                     diB0 = round(sqrt((xiABDE-OB(1,1))^2 + (yiABDE-OB(1,2))^2));
%                     diD0 = round(sqrt((xiABDE-OD(1,1))^2 + (yiABDE-OD(1,2))^2));
%                     diE0 = round(sqrt((xiABDE-OE(1,1))^2 + (yiABDE-OE(1,2))^2));
%                 else
%                     diA0 = 0;
%                     diB0 = 0;
%                     diD0 = 0;
%                     diE0 = 0;
%                 end
%                 
%                 if isempty(xiBCDE) == 0 && isempty(yiBCDE) == 0 && length(xiBCDE) == 1 && length(yiBCDE) == 1
%                     diB1 = round(sqrt((xiBCDE-OB(1,1))^2 + (yiBCDE-OB(1,2))^2));
%                     diC1 = round(sqrt((xiBCDE-OC(1,1))^2 + (yiBCDE-OC(1,2))^2));
%                     diD1 = round(sqrt((xiBCDE-OD(1,1))^2 + (yiBCDE-OD(1,2))^2));
%                     diE1 = round(sqrt((xiBCDE-OE(1,1))^2 + (yiBCDE-OE(1,2))^2));
%                 else
%                     diB1 = 0;
%                     diC1 = 0;
%                     diD1 = 0;
%                     diE1 = 0;
%                 end
%                 
%                 if isempty(xiCDAB) == 0 && isempty(yiCDAB) == 0 && length(xiCDAB) == 1 && length(yiCDAB) == 1
%                     diC2 = round(sqrt((xiCDAB-OC(1,1))^2 + (yiCDAB-OC(1,2))^2));
%                     diD2 = round(sqrt((xiCDAB-OD(1,1))^2 + (yiCDAB-OD(1,2))^2));
%                     diA2 = round(sqrt((xiCDAB-OA(1,1))^2 + (yiCDAB-OA(1,2))^2));
%                     diB2 = round(sqrt((xiCDAB-OB(1,1))^2 + (yiCDAB-OB(1,2))^2));
%                     
%                 else
%                     diC2 = 0;
%                     diD2 = 0;
%                     diA2 = 0;
%                     diB2 = 0;
%                 end
%                 
%                 AB = round(sqrt((OA(1,1)-OB(1,1))^2 + (OA(1,2)-OB(1,2))^2));
%                 BC = round(sqrt((OB(1,1)-OC(1,1))^2 + (OB(1,2)-OC(1,2))^2));
%                 CD = round(sqrt((OC(1,1)-OD(1,1))^2 + (OC(1,2)-OD(1,2))^2));
%                 DE = round(sqrt((OD(1,1)-OE(1,1))^2 + (OD(1,2)-OE(1,2))^2));
%                 AE = round(sqrt((OA(1,1)-OE(1,1))^2 + (OA(1,2)-OE(1,2))^2));
% 
%                 if AB == l1 && BC == l2 && CD == l3 && DE == l4 && AE == l5 && diB1 == 0 && diC1 == 0 && diD1 == 0 && diE1 == 0 && diC2 == 0 && diD2 == 0 && diA2 == 0 && diB2 == 0 && diA0 == 0 && diB0 == 0 && diD0 == 0 && diE0 == 0
%                     
%                         mew = jaco(fh1(i), fh4(j), OC(1,1), OC(1,2));
%                         hold on;
%                         h = scatter(OC(1,1), OC(1,2), 5, mew/0.045, 'filled');hold on;axis equal;
% %                         h = scatter(OC(1,1), OC(1,2), 10, 'b', 'filled');hold on;axis equal;
%                         axis equal;
%                         h1 = plot([OA(1,1) OB(1,1)],[OA(1,2) OB(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
%                         h2 = plot([OB(1,1) OC(1,1)],[OB(1,2) OC(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
%                         h3 = plot([OE(1,1) OD(1,1)],[OE(1,2) OD(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
%                         h4 = plot([OD(1,1) OC(1,1)],[OD(1,2) OC(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
%                         hold on;
%                         drawnow;
%                         delete(h1);
%                         delete(h2);
%                         delete(h3);
%                         delete(h4);
%                     
%                 end
%             end
%              if isreal(th222) == 1 
                OA = [-center,0] ;
                OB = [l1*cosd(fh1(i))-center, l1*sind(fh1(i))] ;
                OC = [l5+ l4*cosd(fh4(j)) + l3*cosd(th322)-center, l4*sind(fh4(j)) + l3*sind(th322)] ;
                OC1 = [l5+ l4*cosd(fh4(j)) + l3*cosd(th322)-center, l4*sind(fh4(j)) + l3*sind(th322)] ;
                OC2 = [l1*cosd(fh1(i)) + l2*cosd(th222)-center, l1*sind(fh1(i)) + l2*sind(th222)] ;
            
                OD = [l5 + l4*cosd(fh4(j))-center, l4*sind(fh4(j))] ;
                OE = [l5-center, 0] ;
                if round(OC1) == round(OC2)
                OC = OC1;
                ABx = [OA(1,1) OB(1,1)];
                ABy = [OA(1,2) OB(1,2)];
                DEx = [OD(1,1) OE(1,1)];
                DEy = [OD(1,2) OE(1,2)];
                [xiABDE, yiABDE] = polyxpoly(ABx, ABy, DEx, DEy);
                
                BCx = [OB(1,1) OC(1,1)];
                BCy = [OB(1,2) OC(1,2)];
                DEx = [OD(1,1) OE(1,1)];
                DEy = [OD(1,2) OE(1,2)];
                [xiBCDE, yiBCDE] = polyxpoly(BCx, BCy, DEx, DEy);
                
                CDx = [OC(1,1) OD(1,1)];
                CDy = [OC(1,2) OD(1,2)];
                ABx = [OA(1,1) OB(1,1)];
                ABy = [OA(1,2) OB(1,2)];
                [xiCDAB, yiCDAB] = polyxpoly(CDx, CDy, ABx, ABy);
                
                if isempty(xiABDE) == 0 && isempty(yiABDE) == 0  && length(xiABDE) == 1 && length(yiABDE) == 1
                    diA0 = round(sqrt((xiABDE-OA(1,1))^2 + (yiABDE-OA(1,2))^2));
                    diB0 = round(sqrt((xiABDE-OB(1,1))^2 + (yiABDE-OB(1,2))^2));
                    diD0 = round(sqrt((xiABDE-OD(1,1))^2 + (yiABDE-OD(1,2))^2));
                    diE0 = round(sqrt((xiABDE-OE(1,1))^2 + (yiABDE-OE(1,2))^2));
                else
                    diA0 = 0;
                    diB0 = 0;
                    diD0 = 0;
                    diE0 = 0;
                end
                
                if isempty(xiBCDE) == 0 && isempty(yiBCDE) == 0 && length(xiBCDE) == 1 && length(yiBCDE) == 1 
                    diB1 = round(sqrt((xiBCDE-OB(1,1))^2 + (yiBCDE-OB(1,2))^2));
                    diC1 = round(sqrt((xiBCDE-OC(1,1))^2 + (yiBCDE-OC(1,2))^2));
                    diD1 = round(sqrt((xiBCDE-OD(1,1))^2 + (yiBCDE-OD(1,2))^2));
                    diE1 = round(sqrt((xiBCDE-OE(1,1))^2 + (yiBCDE-OE(1,2))^2));
                else
                    diB1 = 0;
                    diC1 = 0;
                    diD1 = 0;
                    diE1 = 0;
                end
                
                if isempty(xiCDAB) == 0 && isempty(yiCDAB) == 0 && length(xiCDAB) == 1 && length(yiCDAB) == 1
                    diC2 = round(sqrt((xiCDAB-OC(1,1))^2 + (yiCDAB-OC(1,2))^2));
                    diD2 = round(sqrt((xiCDAB-OD(1,1))^2 + (yiCDAB-OD(1,2))^2));
                    diA2 = round(sqrt((xiCDAB-OA(1,1))^2 + (yiCDAB-OA(1,2))^2));
                    diB2 = round(sqrt((xiCDAB-OB(1,1))^2 + (yiCDAB-OB(1,2))^2));
                    
                else
                    diC2 = 0;
                    diD2 = 0;
                    diA2 = 0;
                    diB2 = 0;
                end
                
                AB = round(sqrt((OA(1,1)-OB(1,1))^2 + (OA(1,2)-OB(1,2))^2));
                BC = round(sqrt((OB(1,1)-OC(1,1))^2 + (OB(1,2)-OC(1,2))^2));
                CD = round(sqrt((OC(1,1)-OD(1,1))^2 + (OC(1,2)-OD(1,2))^2));
                DE = round(sqrt((OD(1,1)-OE(1,1))^2 + (OD(1,2)-OE(1,2))^2));
                AE = round(sqrt((OA(1,1)-OE(1,1))^2 + (OA(1,2)-OE(1,2))^2));

                if AB == l1 && BC == l2 && CD == l3 && DE == l4 && AE == l5 && diB1 == 0 && diC1 == 0 && diD1 == 0 && diE1 == 0 && diC2 == 0 && diD2 == 0 && diA2 == 0 && diB2 == 0 && diA0 == 0 && diB0 == 0 && diD0 == 0 && diE0 == 0
                    
                        mew = jaco(fh1(i), fh4(j), OC(1,1), OC(1,2));
                        mew = mew*200
%                         if mew >= 1
%                             mew = 1;
%                         end
                        meow = [meow, mew];
                        hold on;
                        h = scatter(OC(1,1), OC(1,2), 10, mew, 'filled');hold on;axis equal;
%                         h = scatter(OC(1,1), OC(1,2), 10, 'b', 'filled');hold on;axis equal;
                        xx = [xx, OC(1,1)];
                        yy = [yy, OC(1,2)];
                        axis equal;
                        h1 = plot([OA(1,1) OB(1,1)],[OA(1,2) OB(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
                        h2 = plot([OB(1,1) OC(1,1)],[OB(1,2) OC(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
                        h3 = plot([OE(1,1) OD(1,1)],[OE(1,2) OD(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
                        h4 = plot([OD(1,1) OC(1,1)],[OD(1,2) OC(1,2)],'o-','linewidth',3, 'color', 'red');hold on;axis equal;
                        hold on;
                        drawnow;
                        delete(h1);
                        delete(h2);
                        delete(h3);
                        delete(h4);
                    
                end
              end
        end
    end
end

% size(points)
% ax = gca; 
% h = findobj(gca,'Type','Scatter');
% xx = h.XData 
% yy = h.YData;
xx = xx';
yy = yy';
meow(isnan(meow))=1;
meow = normalize(meow, 'range');
meow = meow';
points = [xx, yy];
figure
[k, A] = boundary(xx,yy,1);
hold on;
axis equal;
plot(xx(k),yy(k));
% size(points)
circle_fit1 = [];
circle_fit2 = [];


ptcloud = [];
meow_rgb = [];
colormap winter
z =0:1:20;
for i = 1:length(z)
    for j = 1:length(xx)
%         h = scatter3(xx(j), yy(j), z(i), 10, meow(j), 'filled');
        if meow(j) >0.5
            meow_rgb = [meow_rgb, [meow(j); meow(j); 0]];
        else
            meow_rgb = [meow_rgb, [0; 0; meow(j)]];
        end
        if meow(j) < 0.5
            circle_fit1 = [circle_fit1, [xx(j); yy(j)]];
        end
        if meow(j) > 0.5
            circle_fit2 = [circle_fit2, [xx(j); yy(j)]];
        end
        
%         RGB = ind2rgb(meow(j), colormap);
%         meow_rgb = [meow_rgb, [RGB(:,:,1); RGB(:,:,2); RGB(:,:,3)]];
        vector = [xx(j); yy(j); z(i)];
        ptcloud = [ptcloud, [vector]];
        hold on;
    end
end
ptcloud = ptcloud';
Cloud = pointCloud(ptcloud, 'Color', meow_rgb')
% size(Cloud)

figure
axis equal;
% pcshow(Cloud)
xlabel('X')
ylabel('Y')
zlabel('Z')
% pointscolor=uint8(zeros(Cloud.Count,3));
% pointscolor(:,1)=255*meow;
% pointscolor(:,2)=255*meow;
% pointscolor(:,3)=51*meow;
% Cloud.Color=pointscolor;
pcshow(Cloud)
hold on;

theta = 0:pi/6:pi;
for i = 1:length(theta)
    r = l5*1.6;
    rot = [cos(theta(i)) sin(theta(i)) 0; ...
          -sin(theta(i)) cos(theta(i)) 0; ...
                   0          0  1];
    trans = [r*sin(theta(i)), r - r*cos(theta(i)), 0];
    tform = rigid3d(rot,trans);
    ptCloudOut = pctransform(Cloud,tform);
    pcshow(ptCloudOut)
    hold on;
end

theta = pi+(pi/6):pi/6:2*pi;
for i = 1:length(theta)
    r = l5*1.6;
    rot = [cos(theta(i)) sin(theta(i)) 0; ...
          -sin(theta(i)) cos(theta(i)) 0; ...
                   0          0  1];
    trans = [r*sin(theta(i)), r - r*cos(theta(i)), 0];
    tform = rigid3d(rot,trans);
    ptCloudOut = pctransform(Cloud,tform);
    pcshow(ptCloudOut)
    hold on;
end

figure()
% size(circle_fit1(1,:))
circle_1 = CircleFitByPratt(circle_fit1');
viscircles([circle_1(1), circle_1(2)], circle_1(3), 'Color', 'y')
hold on;
circle_2 = CircleFitByPratt(circle_fit2');
viscircles([circle_2(1), circle_2(2)], circle_2(3), 'Color', 'y')
hold on;
viscircles([circle_2(1), circle_2(2) + circle_2(3)], circle_2(3), 'Color', 'r')
hold on;
scatter(circle_fit1(1,:), circle_fit1(2,:), 10, 'filled');
hold on;
scatter(circle_fit2(1,:), circle_fit2(2,:), 10, 'filled');
axis equal;






function mew = jaco(fh1, fh4, OCx, OCy)
syms th1 th4
    global J;
%     J = [ (6*sin(2*atan((3*(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5))))((3((pi*cos((pi*th1)/180))/36 - (3*((pi*sin((pi*th1)/180)(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6))/18 + (pi(6*sin((pi*th1)/180) + 5*sin((pi*(th1 - th4))/180))(10*cos((pi*th1)/180) - 10*cos((pi*th4)/180) + (25*cos((pi(th1 - th4))/180))/3 - 43/3))/54 + (pi*cos((pi*th1)/180)(5*sin((pi*th1)/180) - 5*sin((pi*th4)/180)))/18))/(2(225*(sin((th1*pi)/180) - sin((th4*pi)/180))^2 + 9*(5*cos((th4*pi)/180) - 5*cos((th1*pi)/180) + 6)^2 - (30*cos((th1*pi)/180) - 30*cos((th4*pi)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5)) - (pi*(3*sin((pi*th1)/180) + 5*sin((pi*(th1 - th4))/180))(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(300*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2)))/((15*sin((pi*th4)/180) - 15*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))^2/(25*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2) + 1), - (pi*sin((pi*th4)/180))/36 - (6*sin(2*atan((3*(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5))))((3((pi*cos((pi*th4)/180))/36 - (3*((pi*sin((pi*th4)/180)(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6))/18 + (pi(6*sin((pi*th4)/180) + 5*sin((pi*(th1 - th4))/180))(10*cos((pi*th1)/180) - 10*cos((pi*th4)/180) + (25*cos((pi(th1 - th4))/180))/3 - 43/3))/54 + (pi*cos((pi*th4)/180)(5*sin((pi*th1)/180) - 5*sin((pi*th4)/180)))/18))/(2(225*(sin((th1*pi)/180) - sin((th4*pi)/180))^2 + 9*(5*cos((th4*pi)/180) - 5*cos((th1*pi)/180) + 6)^2 - (30*cos((th1*pi)/180) - 30*cos((th4*pi)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5)) - (pi*(3*sin((pi*th4)/180) + 5*sin((pi*(th1 - th4))/180))(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(300*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2)))/((15*sin((pi*th4)/180) - 15*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))^2/(25*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2) + 1);
%         -(6*cos(2*atan((3*(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5))))((3((pi*cos((pi*th1)/180))/36 - (3*((pi*sin((pi*th1)/180)(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6))/18 + (pi(6*sin((pi*th1)/180) + 5*sin((pi*(th1 - th4))/180))(10*cos((pi*th1)/180) - 10*cos((pi*th4)/180) + (25*cos((pi(th1 - th4))/180))/3 - 43/3))/54 + (pi*cos((pi*th1)/180)(5*sin((pi*th1)/180) - 5*sin((pi*th4)/180)))/18))/(2(225*(sin((th1*pi)/180) - sin((th4*pi)/180))^2 + 9*(5*cos((th4*pi)/180) - 5*cos((th1*pi)/180) + 6)^2 - (30*cos((th1*pi)/180) - 30*cos((th4*pi)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5)) - (pi*(3*sin((pi*th1)/180) + 5*sin((pi*(th1 - th4))/180))(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(300*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2)))/((15*sin((pi*th4)/180) - 15*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))^2/(25*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2) + 1),   (pi*cos((pi*th4)/180))/36 + (6*cos(2*atan((3*(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5))))((3((pi*cos((pi*th4)/180))/36 - (3*((pi*sin((pi*th4)/180)(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6))/18 + (pi(6*sin((pi*th4)/180) + 5*sin((pi*(th1 - th4))/180))(10*cos((pi*th1)/180) - 10*cos((pi*th4)/180) + (25*cos((pi(th1 - th4))/180))/3 - 43/3))/54 + (pi*cos((pi*th4)/180)(5*sin((pi*th1)/180) - 5*sin((pi*th4)/180)))/18))/(2(225*(sin((th1*pi)/180) - sin((th4*pi)/180))^2 + 9*(5*cos((th4*pi)/180) - 5*cos((th1*pi)/180) + 6)^2 - (30*cos((th1*pi)/180) - 30*cos((th4*pi)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))))/(5*(3*cos((pi*th1)/180) - 3*cos((pi*th4)/180) + 5*cos((pi*(th1 - th4))/180) - 5)) - (pi*(3*sin((pi*th4)/180) + 5*sin((pi*(th1 - th4))/180))(5*sin((pi*th4)/180) - 5*sin((pi*th1)/180) + (225(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2)/3))/(300*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2)))/((15*sin((pi*th4)/180) - 15*sin((pi*th1)/180) + (225*(sin((pi*th1)/180) - sin((pi*th4)/180))^2 + 9*(5*cos((pi*th4)/180) - 5*cos((pi*th1)/180) + 6)^2 - (30*cos((pi*th1)/180) - 30*cos((pi*th4)/180) + 25*cos((pi*(th1 - th4))/180) - 43)^2)^(1/2))^2/(25*(3*cos((th1*pi)/180) - 3*cos((th4*pi)/180) + 5*cos((pi*(th1 - th4))/180) - 5)^2) + 1)];
    J1 = subs(J, th1, fh1);
    J2 = subs(J1, th4, fh4); 
    J3 = double(J2);
    mew = sqrt(abs(det(J3*(J3'))));
    try
        [U, S, V] = svd(J3);
        LAM1 = S(1,1);
        LAM2 = S(2,2);
        ellipse(sqrt(LAM1)*0.5, sqrt(LAM2)*0.5, atan((U(2,1))/(U(1,1))), OCx, OCy, 'r' );
    end
end

function [th311, th322] = theta3(fh1, fh4)
    global l1 l2 l3 l4 l5
    A = l1*sind(fh1) - l4*sind(fh4);
    B = l1*cosd(fh1) - l4*cosd(fh4) - l5;
    C = ((l1)^2 - (l2)^2 + (l3)^2 + (l4)^2 + (l5)^2)/(2*l3) + (l4*l5*cosd(fh4))/(l3) -  (l1*l5*cosd(fh1))/(l3) - (l1*l4)*(sind(fh4)*sind(fh1) + cosd(fh4)*cosd(fh1))/(l3);
    th311 = 2*atand((A + sqrt((A*A) + (B*B) - (C*C)))/(B + C));
    th322 = 2*atand((A - sqrt((A*A) + (B*B) - (C*C)))/(B + C));
end

% function [th211, th222] = theta2(fh1, fh4, fh31, fh32)
%     global l1 l2 l3 l4
%     th211 = asind((l3*sind(fh31) + l4*sind(fh4) - l1*sind(fh1))/(l2));
%     th222 = asind((l3*sind(fh32) + l4*sind(fh4) - l1*sind(fh1))/(l2)); 
% end

function [th211, th222] = theta2(fh1, fh4, fh31, fh32)
    global l1 l2 l3 l4
    th211 = asind((l3*sind(180-fh31) + l4*sind(fh4) - l1*sind(180-fh1))/(l2));
    th222 = asind((l3*sind(180-fh32) + l4*sind(fh4) - l1*sind(180-fh1))/(l2)); 
end


function h=ellipse(ra,rb,ang,x0,y0,C,Nb)
% Ellipse adds ellipses to the current plot
%
% ELLIPSE(ra,rb,ang,x0,y0) adds an ellipse with semimajor axis of ra,
% a semiminor axis of radius rb, and an orientation of the semimajor
% axis with an angle of ang (in radians) rotated counter-clockwise 
% from the x-axis.  The ellipse is centered at the point x0,y0.
%
% The length of ra, rb, and ang should be the same. 
% If ra is a vector of length L and x0,y0 scalars, L ellipses
% are added at point x0,y0.
% If ra is a scalar and x0,y0 vectors of length M, M ellipse are with the same 
% radii are added at the points x0,y0.
% If ra, x0, y0 are vectors of the same length L=M, M ellipses are added.
% If ra is a vector of length L and x0, y0 are  vectors of length
% M~=L, L*M ellipses are added, at each point x0,y0, L ellipses of radius ra.
%
% ELLIPSE(ra,rb,ang,x0,y0,C)
% adds ellipses of color C. C may be a string ('r','b',...) or the RGB value. 
% If no color is specified, it makes automatic use of the colors specified by 
% the axes ColorOrder property. For several ellipses C may be a vector.
%
% ELLIPSE(ra,rb,ang,x0,y0,C,Nb), Nb specifies the number of points
% used to draw the ellipse. The default value is 300. Nb may be specified
% for each ellipse individually, in which case it should be the same
% length as ra, rb, etc.
%
% h=ELLIPSE(...) returns the handles to the ellipses.
%
% usage exmple: the following produces a red ellipse centered at 1,1
% and tipped down at a 45 deg axis from the x axis
% ellipse(1,2,pi/4,1,1,'r')
%
% note that if ra=rb, ELLIPSE plots a circle
%

% written by D.G. Long, Brigham Young University, based on the
% CIRCLES.m original written by Peter Blattner, Institute of 
% Microtechnology, University of Neuchatel, Switzerland, blattner@imt.unine.ch

% Check the number of input arguments 

if nargin<1,
  ra=[];
end;
if nargin<2,
  rb=[];
end;
if nargin<3,
  ang=[];
end;

if nargin<5,
  x0=[];
  y0=[];
end;
 
if nargin<6,
  C=[];
end

if nargin<7,
  Nb=[];
end

% set up the default values

if isempty(ra),ra=1;end;
if isempty(rb),rb=1;end;
if isempty(ang),ang=0;end;
if isempty(x0),x0=0;end;
if isempty(y0),y0=0;end;
if isempty(Nb),Nb=300;end;
if isempty(C),C=get(gca,'colororder');end;

% work on the variable sizes

x0=x0(:);
y0=y0(:);
ra=ra(:);
rb=rb(:);
ang=ang(:);
Nb=Nb(:);

if isstr(C),C=C(:);end;

if length(ra)~=length(rb),
  error('length(ra)~=length(rb)');
end;
if length(x0)~=length(y0),
  error('length(x0)~=length(y0)');
end;

% how many inscribed elllipses are plotted

if length(ra)~=length(x0)
  maxk=length(ra)*length(x0);
else
  maxk=length(ra);
end;

% drawing loop

for k=1:maxk
  
  if length(x0)==1
    xpos=x0;
    ypos=y0;
    radm=ra(k);
    radn=rb(k);
    if length(ang)==1
      an=ang;
    else
      an=ang(k);
    end;
  elseif length(ra)==1
    xpos=x0(k);
    ypos=y0(k);
    radm=ra;
    radn=rb;
    an=ang;
  elseif length(x0)==length(ra)
    xpos=x0(k);
    ypos=y0(k);
    radm=ra(k);
    radn=rb(k);
    an=ang(k)
  else
    rada=ra(fix((k-1)/size(x0,1))+1);
    radb=rb(fix((k-1)/size(x0,1))+1);
    an=ang(fix((k-1)/size(x0,1))+1);
    xpos=x0(rem(k-1,size(x0,1))+1);
    ypos=y0(rem(k-1,size(y0,1))+1);
  end;

  % draw ellipse
  
  co=cos(an);
  si=sin(an);
  the=linspace(0,2*pi,Nb(rem(k-1,size(Nb,1))+1,:)+1);
  %  x=radm*cos(the)*co-si*radn*sin(the)+xpos;
  %  y=radm*cos(the)*si+co*radn*sin(the)+ypos;
  p=line(radm*cos(the)*co-si*radn*sin(the)+xpos,radm*cos(the)*si+co*radn*sin(the)+ypos);

  set(p,'color',C(rem(k-1,size(C,1))+1,:));
  
  % output handles to each ellipse if output variable specified
  
  if nargout > 0
    h(k)=p;
  end
  
end
end

function Par = CircleFitByPratt(XY)

%--------------------------------------------------------------------------
%  
%     Circle fit by Pratt
%      V. Pratt, "Direct least-squares fitting of algebraic surfaces",
%      Computer Graphics, Vol. 21, pages 145-152 (1987)
%
%     Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)
%
%     Output: Par = [a b R] is the fitting circle:
%                           center (a,b) and radius R
%
%     Note: this fit does not use built-in matrix functions (except "mean"),
%           so it can be easily programmed in any programming language
%
%--------------------------------------------------------------------------

n = size(XY,1);      % number of data points

centroid = mean(XY);   % the centroid of the data set

%     computing moments (note: all moments will be normed, i.e. divided by n)

Mxx=0; Myy=0; Mxy=0; Mxz=0; Myz=0; Mzz=0;

for i=1:n
    Xi = XY(i,1) - centroid(1);  %  centering data
    Yi = XY(i,2) - centroid(2);  %  centering data
    Zi = Xi*Xi + Yi*Yi;
    Mxy = Mxy + Xi*Yi;
    Mxx = Mxx + Xi*Xi;
    Myy = Myy + Yi*Yi;
    Mxz = Mxz + Xi*Zi;
    Myz = Myz + Yi*Zi;
    Mzz = Mzz + Zi*Zi;
end
   
Mxx = Mxx/n;
Myy = Myy/n;
Mxy = Mxy/n;
Mxz = Mxz/n;
Myz = Myz/n;
Mzz = Mzz/n;

%    computing the coefficients of the characteristic polynomial

Mz = Mxx + Myy;
Cov_xy = Mxx*Myy - Mxy*Mxy;
Mxz2 = Mxz*Mxz;
Myz2 = Myz*Myz;

A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
A1 = Mzz*Mz + 4*Cov_xy*Mz - Mxz2 - Myz2 - Mz*Mz*Mz;
A0 = Mxz2*Myy + Myz2*Mxx - Mzz*Cov_xy - 2*Mxz*Myz*Mxy + Mz*Mz*Cov_xy;
A22 = A2 + A2;

epsilon=1e-12; 
ynew=1e+20;
IterMax=20;
xnew = 0;

%    Newton's method starting at x=0

for iter=1:IterMax
    yold = ynew;
    ynew = A0 + xnew*(A1 + xnew*(A2 + 4.*xnew*xnew));
    if (abs(ynew)>abs(yold))
        disp('Newton-Pratt goes wrong direction: |ynew| > |yold|');
        xnew = 0;
        break;
    end
    Dy = A1 + xnew*(A22 + 16*xnew*xnew);
    xold = xnew;
    xnew = xold - ynew/Dy;
    if (abs((xnew-xold)/xnew) < epsilon), break, end
    if (iter >= IterMax)
        disp('Newton-Pratt will not converge');
        xnew = 0;
    end
    if (xnew<0.)
        fprintf(1,'Newton-Pratt negative root:  x=%f\n',xnew);
        xnew = 0;
    end
end

%    computing the circle parameters

DET = xnew*xnew - xnew*Mz + Cov_xy;
Center = [Mxz*(Myy-xnew)-Myz*Mxy , Myz*(Mxx-xnew)-Mxz*Mxy]/DET/2;

Par = [Center+centroid , sqrt(Center*Center'+Mz+2*xnew)];

end    %    CircleFitByPratt
