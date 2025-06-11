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
       %dat(1)=(time(count)-time(count-1))*1000;
       %disp(dat);

       %Parsing to table
       data.Elapse(count-1)=(time(count)-time(count-1))*1000;
       data.AccX(count-1)=dat(1);
       data.AccY(count-1)=dat(2);
       data.AccZ(count-1)=dat(3);
       data.GyroX(count-1)=dat(4);
       data.GyroY(count-1)=dat(5);
       data.GyroZ(count-1)=dat(6);
       data.MagX(count-1)=dat(7);
       data.MagY(count-1)=dat(8);
       data.MagZ(count-1)=dat(9);
       % data.Elapse(count-1)=(time(count)-time(count-1))*1000;
       % data.Mag_raw_X(count-1)=dat(1);
       % data.Mag_raw_Y(count-1)=dat(2);
       % data.Mag_raw_Z(count-1)=dat(3);

       % data.Gyro_dPhi(count-1)=dat(1);
       % data.Gyro_dTheta(count-1)=dat(2);
       % data.Gyro_dGama(count-1)=dat(3);
       % data.Gyro_Phi(count-1)=dat(4);
       % data.Gyro_Theta(count-1)=dat(5);
       % data.Gyro_Gama(count-1)=dat(6);

       % data.A_X_f(count)   = dat(1);   data.A_Y_f(count)  = dat(2);   data.A_Z_f(count)  = dat(3); 

       % data.A_X(count)     = dat(4);   data.A_Y(count)    = dat(5);   data.A_Z(count)    = dat(6); 

       % data.G_X_f(count)   = dat(7);   data.G_Y_f(count)  = dat(8);   data.G_Z_f(count)  = dat(9); 

       % data.G_X(count)     = dat(10);  data.G_Y(count)    = dat(11);  data.G_Z(count)    = dat(12); 
       
       % data.M_X_f(count)     = dat(13);  data.M_Y_f(count)    = dat(14);  data.M_Z_f(count)    = dat(15); 

       % data.M_X(count)   = dat(16);  data.M_Y(count)  = dat(17);  data.M_Z(count)  = dat(18); 

       % data.A_Phi(count)   = dat(19);  data.A_Theta(count)= dat(20);  data.A_Gama(count) = dat(21);
       % 
       % data.G_Phi(count)   = dat(22);  data.G_Theta(count)= dat(23);  data.G_Gama(count) = dat(24);
       % 
       % data.M_Phi(count)   = dat(25);  data.M_Theta(count)= dat(26);  data.M_Gama(count) = dat(27);
       % 
       % data.Phi(count)     = dat(28);  data.Theta(count)  = dat(29);  data.Gama(count)   = dat(30);
       %Allow MATLAB to Update Plot
       %pause(delay);
       disp(data.Elapse(count-1));
       disp(time(count-1));

   end
end

writetable(data);

%Close Serial COM Port and Delete useless Variables
clear count dat delay s serialPort;
disp('Session Terminated...');