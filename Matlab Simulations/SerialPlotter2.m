clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
%Baud=921600;  % define Baudrate #
Baud=115200;
plotTitle = 'Serial Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -15;                     % set y-min
max = 15;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution

%Define Function Variables
time = 0;
data_x = 0;
data_y = 0;
data_z = 0;
count = 0;

%Set up Plot
plotGraphx = plot(time,data_x,'-mo',...
          'LineWidth',1,...
          'MarkerEdgeColor','k',...
          'MarkerFaceColor',[1 1 .63],...
          'MarkerSize',2);
hold on;  % it will hold all the lines on same axis
plotGraphy = plot(time,data_y,'-mo',...   <---- define data_y before this
          'LineWidth',1,...
          'MarkerEdgeColor','b',...
          'MarkerFaceColor',[.49 1 .63],...
          'MarkerSize',2);
plotGraphz = plot(time,data_x,'-mo',...
          'LineWidth',1,...
          'MarkerEdgeColor','r',...
          'MarkerFaceColor',[.49 1 1],...
          'MarkerSize',2);

title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

%Open Serial COM Port
s = serialport(serialPort,Baud);
disp('Close Plot to End Session');

tic 

while ( ishandle(plotGraphx) && ishandle(plotGraphy) && ishandle(plotGraphz)) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)

   %dat = fscanf(s,'%d,%d,%d'); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   dat = readline(s);
   dat=split(dat);
   dat=str2double(dat);
   dat=dat.';
   %disp(dat(4));

   if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
       count = count + 1;    
       time(count) = toc;    %Extract Elapsed Time
       data_x(count) = dat(9);  
       data_y(count) = dat(12); 
       data_z(count) = 200;  

       %Set Axis according to Scroll Width
       % if(scrollWidth > 0)
       set(plotGraphx,'XData',time(time > time(count)-scrollWidth),'YData',data_x(time > time(count)-scrollWidth));
       set(plotGraphy,'XData',time(time > time(count)-scrollWidth),'YData',data_y(time > time(count)-scrollWidth));
       set(plotGraphz,'XData',time(time > time(count)-scrollWidth),'YData',data_z(time > time(count)-scrollWidth));
       axis([time(count)-scrollWidth time(count) min max]);
       % else
       % set(plotGraphx,'XData',time,'YData',data_x);
       % set(plotGraphy,'XData',time,'YData',data_y);
       % set(plotGraphz,'XData',time,'YData',data_z);
       % axis([0 time(count) min max]);
       % end

       %Allow MATLAB to Update Plot
       pause(delay);
   end
end

%Close Serial COM Port and Delete useless Variables
delete(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
       scrollWidth serialPort xLabel yLabel;

disp('Session Terminated...');