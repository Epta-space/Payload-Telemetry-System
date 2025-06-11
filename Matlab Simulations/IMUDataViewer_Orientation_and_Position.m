clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
%Baud=921600;  % define Baudrate #
Baud=115200;
scrollWidth = 15;               % display period in plot, plot entire data log if <= 0
delay = 0;                    % make sure sample faster than resolution
count = 0;

%Define Function Variables
Phi = 0;    d_X = 0; 
Theta = 0;  d_Y = 0; 
Gama = 0;   d_Z = 0; 

data=table(Phi,Theta,Gama,d_X,d_Y,d_Z);

time = 0;

%Set up Plot

%plottools on
figure(1)

q = quaternion([0 0 0],"eulerd","ZYX","frame");
position = [0 0 0];

%file = "Payload_Completa.stl"; 
file = "rocket3.stl"; 

plotOrientation = poseplot(q,position,MeshFileName=file,ScaleFactor=0.01,PatchFaceAlpha=0.8,PatchFaceColor="b");

h_lim_sup = 7;     v_lim_sup = 10;
h_lim_inf = -7;    v_lim_inf = -10;

xlim([h_lim_inf h_lim_sup]); ylim([h_lim_inf h_lim_sup]); zlim([v_lim_inf v_lim_sup]);

xlabel("North-x (m)")
ylabel("East-y (m)")
zlabel("Down-z (m)");

%% While loop
%Open Serial COM Port
s = serialport(serialPort,Baud);
disp('Close Plot to End Session');

tic 

while (ishandle(plotOrientation)) %Loop when Plot is Active && ishandle(plotGraphy) && ishandle(plotGraphz)

   %dat = fscanf(s,'%d,%d,%d'); %Read Data from Serial as Float: PLEASE NOTE: Here i modified the code, in order to adjust the correct format of data in input.
   dat = readline(s);
   dat=split(dat);
   dat=str2double(dat);
   dat=dat.';
   %disp(dat(4));

   if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
       count = count + 1;    
       %time(count) = toc;    %Extract Elapsed Time

       data.Phi(count)     = dat(1);  data.Theta(count)   = dat(2);  data.Gama(count)   = dat(3);
    
       data.d_X(count)     = dat(4);  data.d_Y(count)    = dat(5);  data.d_Z(count)    = dat(6);

       data.time(count)    = dat(7);

       time(count) = data.time(count);

       %Plota
      if(ishandle(plotOrientation))
          q = quaternion([data.Gama(count),data.Theta(count),data.Phi(count)],"eulerd","ZYX","frame");
          set(plotOrientation,Orientation=q,Position=[data.d_X(count),data.d_Y(count),data.d_Z(count)]); 

      end

   end
   %Allow MATLAB to Update Plot
   pause(delay);
end

%Close Serial COM Port and Delete useless Variables
% delete(s);
% clear;

disp('Session Terminated...');