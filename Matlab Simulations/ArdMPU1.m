clear; close all; clc

a = arduino('COM3', 'Uno', 'Libraries', 'I2C');

fs = 100; % Sample Rate in Hz   
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix'); 

ts = tic; 
stopTimer = 50; 
magReadings=[]; 

disp("Calibrando");

while(toc(ts) < stopTimer) 
    % Rotate the sensor along x axis from 0 to 360 degree. 
    % Take 2-3 rotations to improve accuracy. 
    % For other axes, rotate along that axes. 
   [accel,gyro,mag] = read(imu); 
   magReadings = [magReadings;mag]; 
end 

[A, b] = magcal(magReadings); % A = 3x3 matrix for soft iron correction 
                              % b = 3x1 vector for hard iron correction 
                              
% GyroscopeNoise and AccelerometerNoise is determined from datasheet.
GyroscopeNoiseMPU9250 = 3.0462e-06; % GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoiseMPU9250 = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2
viewer = HelperOrientationViewer('Title',{'AHRS Filter'});
FUSE = ahrsfilter('SampleRate',imu.SampleRate, 'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise',AccelerometerNoiseMPU9250);
stopTimer = 100;

% The correction factors A and b are obtained using magcal function as explained in one of the
% previous sections, 'Compensating Magnetometer Distortions'.
magx_correction = b(1);
magy_correction = b(2);
magz_correction = b(3);
ts = tic;
while(toc(ts) < stopTimer)
    [accel,gyro,mag] = read(imu);
    % Align coordinates in accordance with NED convention
    accel = [-accel(:,2), -accel(:,1), accel(:,3)];
    gyro = [gyro(:,2), gyro(:,1), -gyro(:,3)];
    mag = [mag(:,1)-magx_correction, mag(:, 2)- magy_correction, mag(:,3)-magz_correction] * A;
    rotators = FUSE(accel,gyro,mag);
    for j = numel(rotators)
        viewer(rotators(j));
    end
end