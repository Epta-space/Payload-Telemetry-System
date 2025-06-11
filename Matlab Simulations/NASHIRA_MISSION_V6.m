clc
clear variables
close all

%   AMOSTRAGEM DE DADOS
M = readtable('data 31-19-24.txt');

Acc = table2array([M(2:end, 'AccX'), M(2:end, 'AccY'), M(2:end, 'AccZ')]); %x, y, z
Mag = table2array([M(2:end, 'MagX'),M(2:end, 'MagY'),M(2:end, 'MagZ')]); %x, y, z
Gyro = table2array([M(2:end, 'GyroX'),M(2:end, 'GyroY'),M(2:end, 'GyroZ')]); %x, y, z

%   PARÂMETROS DE SIMULAÇÃO
Fs = 200;   % Freq de amostragem
T = 1/Fs;   %Tempo de amostragem
Lans = size(Gyro);
L = Lans(1);
t = (0:(L-1))*T;

acc_sensi = 32767/2;
acc_off = [0 0 0];
Acc2 = (Acc/acc_sensi) - acc_off;

gyro_sensi = 32767/2000;
gyro_off = [mean([-0.7150 -0.7607 -0.7429 -0.6969]) mean([-0.1851 -0.1803 -0.1709 -0.0781 0]) mean([-0.0512 +0.0225 +0.0701 +0.0347])];
Gyro2 = (Gyro/gyro_sensi) - gyro_off;

mag_sensi = 32767/4912;
mag_off = [-30.7535, 56.3195, -16.0127];
Mag2 = [(Mag(:,1)/mag_sensi), (Mag(:,2)/mag_sensi), (Mag(:,3)/mag_sensi)];

[A,b,expmfs] = magcal(Mag2);
MagC = (Mag2-b)*A;

figure()
scatter3(Mag2(:,1),Mag2(:,2),Mag2(:,3),"blue","filled")
hold on
grid(gca,'on')
scatter3(MagC(:,1),MagC(:,2),MagC(:,3),"red","filled")
axis equal
xlabel("x")
ylabel("y")
zlabel("z")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

% FILTRO PASSA-BAIXA
fmag = dfilt.df2t([0.2483 0.4967 0.2483], [1 -0.1842 0.1776]);
facc = dfilt.df2t([0.02008 0.04017 0.02008], [1 -1.561 0.6414]);
fgyro = dfilt.df2t([0.009448 0.01763 0.01763 0.009448], [1 -2.148 1.623 -0.4211]);

%   FILTRO PASSA ALTA GIROSCOPIO
fgyro2 = dfilt.df2t([0.755 -1.51 0.755], [1 -1.656 0.7328]);

Acc_f = filter(facc, Acc2);
Mag_f = filter(fmag, MagC);
Gyro_f2 = filter(fgyro, Gyro2);

Gyro_f = filter(fgyro2, Gyro_f2);

%   CÁLCULO DE ATITUDE A PARTIR DO ACELERÔMERTRO

Acc_Abs = sqrt(Acc_f(:,1).^2 + Acc_f(:,2).^2 + Acc_f(:,3).^2);
acc_lim = 0.1;

Acc_Phi = zeros(size(Acc_f,1), 1);
Acc_Theta = zeros(size(Acc_f,1), 1);
Acc_Gama = zeros(size(Acc_f,1), 1);

for i = 1:length(Acc_f)
    if Acc_Abs(i) < (1 + acc_lim) || Acc_Abs(i) > (1 - acc_lim)
        if Acc_f(i,3) > 2 * acc_lim || Acc_f(i,3) < -2 * acc_lim
            Acc_Phi(i) = atan2(Acc_f(i,2), Acc_f(i,3));
            Acc_Theta(i) = asin(Acc_f(i,1) / Acc_Abs(i));
        else
            Acc_Phi(i) = atan2(Acc_f(i,2), sqrt(Acc_f(i,3)^2 + Acc_f(i,1)^2));
            Acc_Theta(i) = asin(Acc_f(i,1) / Acc_Abs(i));
        end
        Acc_Gama(i) = 0;
    end
end

%   CÁLCULO DE ATITUDE A PARTIR DO MAGNETÔMETRO

Mag_Gama = zeros(size(Mag_f,1), 1);
Mag_Theta = zeros(size(Mag_f,1), 1);

Phi = Acc_Phi(1);
Theta = Acc_Theta(1);

for i = 1:length(Mag_f)
    Mag_Gama(i) = -atan2(Mag_f(i,3) * sin(Phi) - Mag_f(i,2) * cos(Phi), Mag_f(i,1) * cos(Theta) + (Mag_f(i,2) * sin(Phi) + Mag_f(i,3) * cos(Phi)) * sin(Theta));
    end

%   CÁLCULO DE ATITUDE A PARTIR DE UM GIROSCÓPIO

Gyro_X_BF = zeros(size(Gyro_f,1),1);
Gyro_Y_BF = zeros(size(Gyro_f,1),1);
Gyro_Z_BF = zeros(size(Gyro_f,1),1);

Gyro_Phi = zeros(size(Gyro_f,1),1);
Gyro_Theta = zeros(size(Gyro_f,1),1);
Gyro_Gama = zeros(size(Gyro_f,1),1);

Gyro_dPhi = zeros(size(Gyro_f,1),1);
Gyro_dTheta = zeros(size(Gyro_f,1),1);
Gyro_dGama = zeros(size(Gyro_f,1),1);

for i = 1:length(Gyro_f)

    Gyro_X_BF(i) = Gyro_f(i,1) + sin(Phi) .* tan(Theta) .* Gyro_f(i,2) + cos(Phi) .* tan(Theta) .* Gyro_f(i ,3);
    Gyro_Y_BF(i) = cos(Phi) .* Gyro_f(i,2) - sin(Phi) .* Gyro_f(i,3);
    Gyro_Z_BF(i) = sin(Phi) ./ cos(Theta) .* Gyro_f(i,2) + cos(Phi) ./ cos(Theta) .* Gyro_f(i,3);

    Gyro_dPhi(i) = Gyro_X_BF(i) * T / (180*pi);
    Gyro_dTheta(i) = Gyro_Y_BF(i) * T / (180 * pi);
    Gyro_dGama(i) = Gyro_Z_BF(i) * T / (180 * pi); 

    Gyro_Phi(i) = Gyro_Phi(i) + Gyro_dPhi(i);
    Gyro_Theta(i) = Gyro_Theta(i) + Gyro_dTheta(i);
    Gyro_Gama(i) = Gyro_Gama(i) + Gyro_dGama(i);
end


%   OBSERVADOR DE ESTADOS

Ad = [1, T; 
      0, 1];

Bd = [T; 
      0];

Cd = [1, 0]; 

desired_poles = [0.7, 0.71]; % Polos ajustados
Ke = place(Ad', Cd', desired_poles)';
disp('Observer gain Ke:');
disp(Ke);

% Estado inicial
x_hat0 = [0; 0];
y_hat0 = [0; 0];
z_hat0 = [0; 0];

% Inicializar o estado estimado
x_hat = x_hat0;
y_hat = y_hat0;
z_hat = z_hat0;

x_hat_history = x_hat';
y_hat_history = y_hat';
z_hat_history = z_hat';


attitude = zeros(3, L); % Inicializar o vetor attitude com o mesmo tamanho de t


x_hat_pred = x_hat;
y_hat_pred = y_hat;
z_hat_pred = z_hat;

% Cálculos do Observador

for k = 1:L

        %   EIXO X

    % Atualização do observador - Correção
    x_hat = x_hat_pred + Ke * (Acc_Phi(k) - Cd * x_hat_pred);

    % Atualização do observador - Predição
     x_hat_pred = Ad * x_hat_history(k,:)' + Bd * Gyro_Phi(k);

    % Armazenar o estado estimado
    x_hat_history = [x_hat_history; x_hat'];

    % Atualizar a atitude
    attitude(k,1) = x_hat(1); 

        %   EIXO Y

    % Atualização do observador - Correção
    y_hat = y_hat_pred + Ke * (Acc_Theta(k) - Cd * y_hat_pred);

    % Atualização do observador - Predição
    y_hat_pred = Ad * y_hat_history(k,:)' + Bd * Gyro_Theta(k);

    % Armazenar o estado estimado
    y_hat_history = [y_hat_history; y_hat'];

    % Atualizar a atitude
    attitude(k,2) = y_hat(1); 

            %   EIXO Z

    % Atualização do observador - Correção
    z_hat = z_hat_pred + Ke * (Mag_Gama(k) - Cd * z_hat_pred);

    % Atualização do observador - Predição
    z_hat_pred = Ad * z_hat_history(k,:)' + Bd * Gyro_Gama(k);

    % Armazenar o estado estimado
    z_hat_history = [z_hat_history; z_hat'];

    % Atualizar a atitude
    attitude(k,3) = z_hat(1); 

end

% Plotar os resultados do observador de estados após todos os cálculos
figure;
plot(t, attitude, 'r--', 'DisplayName', 'Estimated Attitude');
hold on;
plot(t, Acc_Phi, 'b', 'DisplayName', 'Accelerometer Data');
%plot(t, Mag_Theta, 'g', 'DisplayName', 'Magnetometer Data');
plot(t, Gyro_Phi, 'k--', 'DisplayName', 'Gyroscope Data');
xlabel('Time (s)');
ylabel('Attitude (rad)');
legend;
title('Luenberger State Observer with Sensor Data');