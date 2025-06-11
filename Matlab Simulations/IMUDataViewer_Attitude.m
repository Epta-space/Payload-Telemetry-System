clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
%Baud=921600;  % define Baudrate #
Baud=115200;
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .0;                    % make sure sample faster than resolution
count = 0;

%Define Function Variables
A_Phi = 0;      G_Phi = 0;      Phi_CF = 0;     Phi_SO = 0;     Phi_KF = 0;     Phi_EKF = 0;
A_Theta = 0;    G_Theta = 0;    Theta_CF = 0;   Theta_SO = 0;   Theta_KF = 0;   Theta_EKF = 0;
M_Gama = 0;     G_Gama = 0;     Gama_CF = 0;    Gama_SO = 0;    Gama_KF = 0;    Gama_EKF = 0;

data=table(A_Phi,G_Phi,A_Theta,G_Theta,M_Gama,G_Gama,Phi_CF,Theta_CF,Gama_CF,Phi_SO,Theta_SO,Gama_SO,Phi_KF,Theta_KF,Gama_KF,Phi_EKF,Theta_EKF,Gama_EKF);

time = 0;

%Set up Plot

%plottools on
figure(1)

q = quaternion([0 0 0],"eulerd","ZYX","frame");
position = [0 0 0];

%file = "Payload_Completa.stl"; 
file = "rocket3.stl"; 

plotOrientation = poseplot(q,position,MeshFileName=file,ScaleFactor=0.01,PatchFaceAlpha=0.8,PatchFaceColor="b");

xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);

xlabel("North-x (m)")
ylabel("East-y (m)")
zlabel("Down-z (m)");

%% Plot All Data
figure(2)

TitleSize = 10;
YlabSize = 6;

MkSize = 0.5;
LWidth = 0.5;
plotGrid = 'on';                % 'off' to turn off grid

LSpec_Acc='-r*';
LSpec_Gyro='-b*';
LSpec_CF='-go';
LSpec_SO='-mo';
LSpec_KF='-k*';
LSpec_EKF='-b*';

min_A = -90;                    % set y-min
max_A = 90;                     % set y-max
min_B = -90;                     % set y-min
max_B = 90;                      % set y-max

% Plot A

% Phi
SubP_A_Phi=subplot(1,3,1);

plotGraph_A_Phi_Acc = plot(time,data.A_Phi,LSpec_Acc, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Phi_Gyro = plot(time,data.G_Phi,LSpec_Gyro,'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Phi_CF = plot(time,data.Phi_CF,LSpec_CF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Phi_SO = plot(time,data.Phi_SO,LSpec_SO, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Phi_KF = plot(time,data.Phi_KF,LSpec_KF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Phi_EKF = plot(time,data.Phi_EKF,LSpec_EKF,'LineWidth',LWidth,'MarkerSize',MkSize);

ylabel('Phi [degrees]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

% Theta
SubP_A_Theta=subplot(1,3,2);

plotGraph_A_Theta_Acc = plot(time,data.A_Theta,LSpec_Acc, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Theta_Gyro = plot(time,data.G_Theta,LSpec_Gyro,'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Theta_CF = plot(time,data.Theta_CF,LSpec_CF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Theta_SO = plot(time,data.Theta_SO,LSpec_SO, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Theta_KF = plot(time,data.Theta_KF,LSpec_KF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Theta_EKF = plot(time,data.Theta_EKF,LSpec_EKF,'LineWidth',LWidth,'MarkerSize',MkSize);

ylabel('Theta [degrees]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

title("Attitude Estimation",'FontSize',TitleSize);

% Gama
SubP_A_Gama=subplot(1,3,3);

plotGraph_A_Gama_Mag = plot(time,data.M_Gama,LSpec_Acc, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Gama_Gyro = plot(time,data.G_Gama,LSpec_Gyro,'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_A_Gama_CF = plot(time,data.Gama_CF,LSpec_CF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Gama_SO = plot(time,data.Gama_SO,LSpec_SO, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Gama_KF = plot(time,data.Gama_KF,LSpec_KF, 'LineWidth',LWidth,'MarkerSize',MkSize); hold on;
plotGraph_B_Gama_EKF = plot(time,data.Gama_EKF,LSpec_EKF,'LineWidth',LWidth,'MarkerSize',MkSize);

ylabel('Gama [degrees]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

legend("Accelerometer Attitude","Gyro Attitude","Complementary Filter Attitude","State Observer Attitude" ...
    ,"Kalman Filter Attitude","Extended Kalman Filter Attitude","Location","southeast","Orientation","vertical")
%% While loop
%Open Serial COM Port
s = serialport(serialPort,Baud);
disp('Close Plot to End Session');

tic 

while (ishandle(plotOrientation) || ishandle(plotGraph_A_Phi_Acc)) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)

   %dat = fscanf(s,'%d,%d,%d'); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   dat = readline(s);
   dat=split(dat);
   dat=str2double(dat);
   dat=dat.';
   %disp(dat(4));

   if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
       count = count + 1;    
       %time(count) = toc;    %Extract Elapsed Time

       %data.time(count)    = toc;

       data.A_Phi(count)   = dat(1);   data.A_Theta(count)  = dat(2); data.M_Gama(count)   = dat(3); 

       data.G_Phi(count)   = dat(4);   data.G_Theta(count)  = dat(5); data.G_Gama(count)   = dat(6); 

       data.Phi_CF(count)  = dat(7);   data.Theta_CF(count) = dat(8); data.Gama_CF(count)  = dat(9); 

       data.Phi_SO(count)  = dat(10);  data.Theta_SO(count) = dat(11);data.Gama_SO(count)  = dat(12); 
       
       data.Phi_KF(count)  = dat(13);  data.Theta_KF(count) = dat(14);data.Gama_KF(count)  = dat(15); 

       %data.Phi_EKF(count) = dat(16);  data.Theta_EKF(count)= dat(17);data.Gama_EKF(count) = dat(18); 
       data.Phi_EKF(count) =-100;  data.Theta_EKF(count)= -100;data.Gama_EKF(count) = -100; 

       data.time(count)=dat(16)/1000;

       time(count) = data.time(count);

       %Plota
      if(ishandle(plotGraph_A_Phi_Acc))
           %Phi A
           set(plotGraph_A_Phi_CF,'XData',time(time > time(count)-scrollWidth),'YData',data.Phi_CF(time > time(count)-scrollWidth));
           set(plotGraph_A_Phi_Acc,'XData',time(time > time(count)-scrollWidth),'YData',data.A_Phi(time > time(count)-scrollWidth));
           set(plotGraph_A_Phi_Gyro,'XData',time(time > time(count)-scrollWidth),'YData',data.G_Phi(time > time(count)-scrollWidth));
           SubP_A_Phi.XLim=[time(count)-scrollWidth time(count)];
    
           %Theta A
           set(plotGraph_A_Theta_CF,'XData',time(time > time(count)-scrollWidth),'YData',data.Theta_CF(time > time(count)-scrollWidth));
           set(plotGraph_A_Theta_Acc,'XData',time(time > time(count)-scrollWidth),'YData',data.A_Theta(time > time(count)-scrollWidth));
           set(plotGraph_A_Theta_Gyro,'XData',time(time > time(count)-scrollWidth),'YData',data.G_Theta(time > time(count)-scrollWidth));
           SubP_A_Theta.XLim=[time(count)-scrollWidth time(count)];
    
           %Gama A
           set(plotGraph_A_Gama_CF,'XData',time(time > time(count)-scrollWidth),'YData',data.Gama_CF(time > time(count)-scrollWidth));
           set(plotGraph_A_Gama_Mag,'XData',time(time > time(count)-scrollWidth),'YData',data.M_Gama(time > time(count)-scrollWidth));
           set(plotGraph_A_Gama_Gyro,'XData',time(time > time(count)-scrollWidth),'YData',data.G_Gama(time > time(count)-scrollWidth));
           SubP_A_Gama.XLim=[time(count)-scrollWidth time(count)];
    
           %Phi B
           set(plotGraph_B_Phi_SO,'XData',time(time > time(count)-scrollWidth),'YData',data.Phi_SO(time > time(count)-scrollWidth));
           set(plotGraph_B_Phi_KF,'XData',time(time > time(count)-scrollWidth),'YData',data.Phi_KF(time > time(count)-scrollWidth));
           set(plotGraph_B_Phi_EKF,'XData',time(time > time(count)-scrollWidth),'YData',data.Phi_EKF(time > time(count)-scrollWidth));
           SubP_B_Phi.XLim=[time(count)-scrollWidth time(count)];
    
           %Theta B
           set(plotGraph_B_Theta_SO,'XData',time(time > time(count)-scrollWidth),'YData',data.Theta_SO(time > time(count)-scrollWidth));
           set(plotGraph_B_Theta_KF,'XData',time(time > time(count)-scrollWidth),'YData',data.Theta_KF(time > time(count)-scrollWidth));
           set(plotGraph_B_Theta_EKF,'XData',time(time > time(count)-scrollWidth),'YData',data.Theta_EKF(time > time(count)-scrollWidth));
           SubP_B_Theta.XLim=[time(count)-scrollWidth time(count)];
    
           %Gama B
           set(plotGraph_B_Gama_SO,'XData',time(time > time(count)-scrollWidth),'YData',data.Gama_SO(time > time(count)-scrollWidth));
           set(plotGraph_B_Gama_KF,'XData',time(time > time(count)-scrollWidth),'YData',data.Gama_KF(time > time(count)-scrollWidth));
           set(plotGraph_B_Gama_EKF,'XData',time(time > time(count)-scrollWidth),'YData',data.Gama_EKF(time > time(count)-scrollWidth));
           SubP_B_Gama.XLim=[time(count)-scrollWidth time(count)];
    
      end
      if(ishandle(plotOrientation))
          q = quaternion([data.Gama_KF(count),data.Theta_KF(count),data.Phi_KF(count)],"eulerd","ZYX","frame");
          set(plotOrientation,Orientation=q); 

      end

   end
   %Allow MATLAB to Update Plot
   pause(delay);
end

%Close Serial COM Port and Delete useless Variables
% delete(s);
% clear;

disp('Session Terminated...');