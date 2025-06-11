clc
close all
clear all

%CALIBRAÇÃO

m = readtable("dadosMag3.txt");
m1 = table2array([m(:,'Mag_X'),m(:,'Mag_Y'),m(:,'Mag_Z')]);
[A,b,expmfs]=magcal(m1);
C = (m1-b)*A;

%PLOT CALIBRADO

figure()
hold on
plot3(m1(:,1),m1(:,2),m1(:,3),"LineStyle","none","Marker","X","MarkerSize",8)
grid(gca,'on')
plot3(C(:,1),C(:,2),C(:,3),"LineStyle", "none" ,"Marker", "o" ,"MarkerSize",8,"MarkerFaceColor","r")
axis equal
xlabel("x")
ylabel("y")
zlabel("z")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

%TESTE

m2 = readtable("dadosMag4.txt");
m22 = table2array([m2(:,'Mag_X'),m2(:,'Mag_Y'),m2(:,'Mag_Z')]);
C2 = (m22-b)*A

%PLOT DO TESTE

figure()
hold on
plot3(m22(:,1),m22(:,2),m22(:,3),"LineStyle","none","Marker","X","MarkerSize",8)
grid(gca,'on')
plot3(C2(:,1),C2(:,2),C2(:,3),"LineStyle", "none" ,"Marker", "o" ,"MarkerSize",8,"MarkerFaceColor","r")
axis equal
xlabel("x")
ylabel("y")
zlabel("z")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

