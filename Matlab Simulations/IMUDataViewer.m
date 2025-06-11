clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
%Baud=921600;  % define Baudrate #
Baud=115200;
scrollWidth = 15;               % display period in plot, plot entire data log if <= 0
delay = .1;                    % make sure sample faster than resolution
count = 0;

%Define Function Variables
time = 0;
data_A_X_f = 0; data_A_Y_f = 0; data_A_Z_f = 0; data_A_X = 0; data_A_Y = 0; data_A_Z = 0; data_A_R = 0; data_A_P = 0; data_A_Ya = 0;
data_G_X_f = 0; data_G_Y_f = 0; data_G_Z_f = 0; data_G_X = 0; data_G_Y = 0; data_G_Z = 0; data_G_R = 0; data_G_P = 0; data_G_Ya = 0;
data_M_X_f = 0; data_M_Y_f = 0; data_M_Z_f = 0; data_M_X = 0; data_M_Y = 0; data_M_Z = 0; data_M_R = 0; data_M_P = 0; data_M_Ya = 0;
data_R = 0;     data_P = 0;     data_Ya = 0;
%Set up Plot
figure(1)
plotStop = plot(0);

%% Plot All Data
TitleSize = 10;
YlabSize = 6;

MkSize = 0.5;
LWidth = 0.5;
plotGrid = 'on';                % 'off' to turn off grid

LSpec_A_X='-ko';    LSpec_A_Y='-ko';    LSpec_A_Z='-ko';
LSpec_A_X_r='-r*';  LSpec_A_Y_r='-r*';  LSpec_A_Z_r='-r*';

LSpec_G_X='-ko';    LSpec_G_Y='-ko';    LSpec_G_Z='-ko';
LSpec_G_X_r='-g*';  LSpec_G_Y_r='-g*';  LSpec_G_Z_r='-g*';

LSpec_M_X='-ko';    LSpec_M_Y='-ko';    LSpec_M_Z='-ko';
LSpec_M_X_r='-m*';  LSpec_M_Y_r='-m*';  LSpec_M_Z_r='-m*';

min_A = -1.5;                     % set y-min
max_A = 1.5;                      % set y-max
min_G = -20;                     % set y-min
max_G = 20;                      % set y-max
min_M = -30;                     % set y-min
max_M = 30;                      % set y-max

figure(2)

% Plot Aceleration

% Acc X
SubP_A_X=subplot(3,3,1);

plotGraph_A_X_f = plot(time,data.A_X_f,LSpec_A_X, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_A_X = plot(time,data_A_X,LSpec_A_X_r,'LineWidth',LWidth,'MarkerEdgeColor','r','MarkerSize',MkSize);

ylabel('Acc X [g]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

% Acc Y
SubP_A_Y=subplot(3,3,2);

plotGraph_A_Y_f = plot(time,data_A_Y_f,LSpec_A_Y, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_A_Y = plot(time,data_A_Y,LSpec_A_Y_r,'LineWidth',LWidth,'MarkerEdgeColor','r','MarkerSize',MkSize);

ylabel('Acc Y [g]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

title("Aceleration Plot",'FontSize',TitleSize);

% Acc Z
SubP_A_Z=subplot(3,3,3);

plotGraph_A_Z_f = plot(time,data_A_Z_f,LSpec_A_Z, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_A_Z = plot(time,data_A_Z,LSpec_A_Z_r,'LineWidth',LWidth,'MarkerEdgeColor','r','MarkerSize',MkSize);

ylabel('Acc Y [g]','FontSize',YlabSize);  axis([0 10 min_A max_A]);   grid(plotGrid);

% Plot Angular Velocity

% Gyro X
SubP_G_X=subplot(3,3,4);

plotGraph_G_X_f = plot(time,data_G_X_f,LSpec_G_X, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_G_X = plot(time,data_G_X,LSpec_G_X_r,'LineWidth',LWidth,'MarkerEdgeColor','b','MarkerSize',MkSize);

ylabel('Gyro X [deg/s]','FontSize',YlabSize);  axis([0 10 min_G max_G]);   grid(plotGrid);

% Gyro Y
SubP_G_Y=subplot(3,3,5);

plotGraph_G_Y_f = plot(time,data_G_Y_f,LSpec_G_Y, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_G_Y = plot(time,data_G_Y,LSpec_G_Y_r,'LineWidth',LWidth,'MarkerEdgeColor','b','MarkerSize',MkSize);

ylabel('Gyro Y [deg/s]','FontSize',YlabSize);  axis([0 10 min_G max_G]);   grid(plotGrid);

title("Angular Velocity Plot",'FontSize',TitleSize);
% Gyro Z
SubP_G_Z=subplot(3,3,6);

plotGraph_G_Z_f = plot(time,data_G_Z_f,LSpec_G_Z, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_G_Z = plot(time,data_G_Z,LSpec_G_Z_r,'LineWidth',LWidth,'MarkerEdgeColor','b','MarkerSize',MkSize);

ylabel('Gyro Z [deg/s]','FontSize',YlabSize);  axis([0 10 min_G max_G]);   grid(plotGrid);

% Plot Magnetic Field

% Mag X
SubP_M_X=subplot(3,3,7);

plotGraph_M_X_f = plot(time,data_M_X_f,LSpec_M_X, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_M_X = plot(time,data_M_X,LSpec_M_X_r,'LineWidth',LWidth,'MarkerEdgeColor','m','MarkerSize',MkSize);

ylabel('Mag X [uT]','FontSize',YlabSize);  axis([0 10 min_M max_M]);   grid(plotGrid);

% Mag Y
SubP_M_Y=subplot(3,3,8);

plotGraph_M_Y_f = plot(time,data_M_Y_f,LSpec_M_Y, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_M_Y = plot(time,data_M_Y,LSpec_M_Y_r,'LineWidth',LWidth,'MarkerEdgeColor','m','MarkerSize',MkSize);

ylabel('Mag Y [uT]','FontSize',YlabSize);  axis([0 10 min_M max_M]);   grid(plotGrid);

title("Magnetic Field Plot",'FontSize',TitleSize);

% Mag Z
SubP_M_Z=subplot(3,3,9);

plotGraph_M_Z_f = plot(time,data_M_Z_f,LSpec_M_Z, 'LineWidth',LWidth,'MarkerEdgeColor','k','MarkerSize',MkSize); hold on;
plotGraph_M_Z = plot(time,data_M_Z,LSpec_M_Z_r,'LineWidth',LWidth,'MarkerEdgeColor','m','MarkerSize',MkSize);

ylabel('Mag Z [uT]','FontSize',YlabSize);  axis([0 10 min_M max_M]);   grid(plotGrid);

%% While loop
%Open Serial COM Port
s = serialport(serialPort,Baud);
disp('Close Plot to End Session');

tic 

while (ishandle(plotStop) || ishandle(plotGraph_A_X_f)) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)

   %dat = fscanf(s,'%d,%d,%d'); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   dat = readline(s);
   dat=split(dat);
   dat=str2double(dat);
   dat=dat.';
   %disp(dat(4));

   if(~isempty(dat) && isfloat(dat) && ishandle(plotGraph_A_X_f)) %Make sure Data Type is Correct        
       count = count + 1;    
       time(count) = toc;    %Extract Elapsed Time

       data_A_X_f(count)   = dat(1);   data_A_Y_f(count)  = dat(2);   data_A_Z_f(count)  = dat(3); 

       data_A_X(count)     = dat(4);   data_A_Y(count)    = dat(5);   data_A_Z(count)    = dat(6); 

       data_G_X_f(count)   = dat(7);   data_G_Y_f(count)  = dat(8);   data_G_Z_f(count)  = dat(9); 

       data_G_X(count)     = dat(10);  data_G_Y(count)    = dat(11);  data_G_Z(count)    = dat(12); 
       
       data_M_X(count)     = dat(13);  data_M_Y(count)    = dat(14);  data_M_Z(count)    = dat(15); 

       data_A_R(count)     = dat(16);  data_A_P(count)    = dat(17);  data_A_Ya(count)   = dat(18);

       data_G_R(count)     = dat(19);  data_G_P(count)    = dat(20);  data_G_Ya(count)   = dat(21);

       data_M_R(count)     = dat(22);  data_M_P(count)    = dat(23);  data_M_Ya(count)   = dat(24);

       data_R(count)       = dat(25);  data_P(count)      = dat(26);  data_Ya(count)     = dat(27);

       %Plota

       %Acc X
       set(plotGraph_A_X_f,'XData',time(time > time(count)-scrollWidth),'YData',data_A_X_f(time > time(count)-scrollWidth));
       set(plotGraph_A_X,'XData',time(time > time(count)-scrollWidth),'YData',data_A_X(time > time(count)-scrollWidth));
       SubP_A_X.XLim=[time(count)-scrollWidth time(count)];

       %Acc Y
       set(plotGraph_A_Y_f,'XData',time(time > time(count)-scrollWidth),'YData',data_A_Y_f(time > time(count)-scrollWidth));
       set(plotGraph_A_Y,'XData',time(time > time(count)-scrollWidth),'YData',data_A_Y(time > time(count)-scrollWidth));
       SubP_A_Y.XLim=[time(count)-scrollWidth time(count)];

       %Acc Z
       set(plotGraph_A_Z_f,'XData',time(time > time(count)-scrollWidth),'YData',data_A_Z_f(time > time(count)-scrollWidth));
       set(plotGraph_A_Z,'XData',time(time > time(count)-scrollWidth),'YData',data_A_Z(time > time(count)-scrollWidth));
       SubP_A_Z.XLim=[time(count)-scrollWidth time(count)];

       %Gyro X
       set(plotGraph_G_X_f,'XData',time(time > time(count)-scrollWidth),'YData',data_G_X_f(time > time(count)-scrollWidth));
       set(plotGraph_G_X,'XData',time(time > time(count)-scrollWidth),'YData',data_G_X(time > time(count)-scrollWidth));
       SubP_G_X.XLim=[time(count)-scrollWidth time(count)];

       %Gyro Y
       set(plotGraph_G_Y_f,'XData',time(time > time(count)-scrollWidth),'YData',data_G_Y_f(time > time(count)-scrollWidth));
       set(plotGraph_G_Y,'XData',time(time > time(count)-scrollWidth),'YData',data_G_Y(time > time(count)-scrollWidth));
       SubP_G_Y.XLim=[time(count)-scrollWidth time(count)];

       %Gyro Z
       set(plotGraph_G_Z_f,'XData',time(time > time(count)-scrollWidth),'YData',data_G_Z_f(time > time(count)-scrollWidth));
       set(plotGraph_G_Z,'XData',time(time > time(count)-scrollWidth),'YData',data_G_Z(time > time(count)-scrollWidth));
       SubP_G_Z.XLim=[time(count)-scrollWidth time(count)];

       %Mag X
       %set(plotGraph_M_X_f,'XData',time(time > time(count)-scrollWidth),'YData',data_M_X_f(time > time(count)-scrollWidth));
       set(plotGraph_M_X,'XData',time(time > time(count)-scrollWidth),'YData',data_M_X(time > time(count)-scrollWidth));
       SubP_M_X.XLim=[time(count)-scrollWidth time(count)];

       %Mag Y
       %set(plotGraph_M_Y_f,'XData',time(time > time(count)-scrollWidth),'YData',data_M_Y_f(time > time(count)-scrollWidth));
       set(plotGraph_M_Y,'XData',time(time > time(count)-scrollWidth),'YData',data_M_Y(time > time(count)-scrollWidth));
       SubP_M_Y.XLim=[time(count)-scrollWidth time(count)];

       %Mag Z
       %set(plotGraph_M_Z_f,'XData',time(time > time(count)-scrollWidth),'YData',data_M_Z_f(time > time(count)-scrollWidth));
       set(plotGraph_M_Z,'XData',time(time > time(count)-scrollWidth),'YData',data_M_Z(time > time(count)-scrollWidth));
       SubP_M_Z.XLim=[time(count)-scrollWidth time(count)];
   end
   %Allow MATLAB to Update Plot
   pause(delay);
end

%Close Serial COM Port and Delete useless Variables
delete(s);
clear;

disp('Session Terminated...');