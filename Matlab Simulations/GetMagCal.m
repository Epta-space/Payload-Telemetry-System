clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
Baud=115200;
%Baud=921600;                    % define Baudrate #
Sampling_Time=120;
delay = .005;                    % make sure sample faster than resolution

%Define Function Variables
time = 0;
count = 1;

%Create data table
data=table;

%Open Serial COM Port
s = serialport(serialPort,Baud);
tic 

while (time(count)<Sampling_Time) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)

   %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   dat=readline(s);
   dat=split(dat);
   dat=str2double(dat);
   disp(dat);

   if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
       count = count + 1;    
       time(count) = toc;    %Extract Elapsed Time

       %Parsing to table
       data.Elapse(count-1)=(time(count)-time(count-1))*1000;
       data.Mag_X(count-1)=dat(1);
       data.Mag_Y(count-1)=dat(2);
       data.Mag_Z(count-1)=dat(3);

       %Allow MATLAB to Update Plot
       %pause(delay);
   end
end
writetable(data,'MagData');


m1=readmatrix("MagData.txt");
m1=rmmissing(m1);
m1=m1(:,2:4);
% m22=m1;
%CALIBRAÇÃO
% m1 = table2array([data(:,'Mag_X'),data(:,'Mag_Y'),data(:,'Mag_Z')]);
m1(1,:) = [];

[A,b,expmfs]=magcal(m1);
C = (m1-b)*A;

figure()
scatter3(m1(:,1),m1(:,2),m1(:,3),"blue","filled");hold on;
scatter3(C(:,1),C(:,2),C(:,3),"red","filled");
axis equal;
title('Magnetometer Data - Uncalibrated');
xlabel('Mag X');
ylabel('Mag Y');
zlabel('Mag Z');
hold off;

%PLOT CALIBRADO

figure()
hold on
plot3(m1(:,1),m1(:,2),m1(:,3),"LineStyle","none","Marker","X","MarkerSize",8);
grid(gca,'on');
plot3(C(:,1),C(:,2),C(:,3),"LineStyle", "none" ,"Marker", "o" ,"MarkerSize",8,"MarkerFaceColor","r");
axis equal;
xlabel("x");
ylabel("y");
zlabel("z");
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside");
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements");
hold off;

%TESTE

m2 = readtable("dadosMag4.txt");
m22 = table2array([m2(:,'Mag_X'),m2(:,'Mag_Y'),m2(:,'Mag_Z')]);
C2 = (m22-b)*A;

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

disp(A);
disp(b);

%Close Serial COM Port and Delete useless Variables
clear count dat delay s serialPort;
disp('Session Terminated...');