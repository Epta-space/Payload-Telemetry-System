clear; close all; clc;

%Porta serial
Port='COM4';
Baud=115200;
s = serialport(Port,Baud);
txt=".txt";
name="IMU__1k";
file_name=name+txt;
fileID = fopen(file_name,'w');

%fclose(fileID);

%Plot

plotGraphx = plot(time,data_x,'-mo',...
          'LineWidth',1,...
          'MarkerEdgeColor','k',...
          'MarkerFaceColor',[1 1 .63],...
          'MarkerSize',2);

i=1;
while(ishandle(plotGraphx))
    data=readline(s);
    data = string(readline(s));
    %writelines(data,file_name);
    fprintf(fileID,'%s',data);

    % if i < 150
    %     plot(data, 'LineWidth',1.5);
    % else
    %     plot(data(end-150:end), 'LineWidth',1.5);
    % end
    % %set de limites: ylim([0 3.5]);
    % pause(0.01);
    % i=i+1;
end

