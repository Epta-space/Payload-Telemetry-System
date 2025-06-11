clear
clc
% Data

%User Defined Properties 
serialPort = "/dev/ttyUSB0";            % define COM port #
Baud=115200;
%Baud=921600;                    % define Baudrate #
Sampling_Time=150;
delay = .005;                    % make sure sample faster than resolution

%Define Function Variables
time = 0;
data=0;
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

writetable(data);
scatter3(data.Mag_X,data.Mag_Y,data.Mag_Z,"magenta","filled");
title('Magnetometer Data - Uncalibrated');
xlabel('Mag X');
ylabel('Mag Y')
zlabel('Mag Z')

%Close Serial COM Port and Delete useless Variables
clear count dat delay s serialPort;
disp('Session Terminated...');