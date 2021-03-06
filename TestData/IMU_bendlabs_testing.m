%% GOATSES BendLabs and IMU Integration Testing
%QM, CW, IT, AH
%4/2/22
%

%% housekeeping
clear 
clc
close all

%% set filenames

filename_BL = 'TestIMU';
filename_IMU = 'IMU_Integration_Test.csv';

%% Read in bendlabs data
BL = readmatrix(filename_BL);

%% Read in IMU data
IMU = csvread(filename_IMU);

%Time vector for BL data
t_BL = 1:length(BL(:, 1));

%Time vector for IMU data
t_IMU = 1:length(IMU(:, 1));


%Plot BL data
plot(t_BL, BL(:, 1));
hold on
%plot(t_BL, BL(:, 2)); %gonna need to know how much of a second each thing is for BL
hold off
xlabel('Time [not real units currently!]');
ylabel('Angle [degrees]');
title('BendLabs data')
legend('Angle of ... somethin', 'Location', 'Best');
%k so this might not be the raw raw data i'm not sure the second data set
%(BL(:, 2)) can be uncommented and that's super noisy but probably also
%useful
%and we could smooth it

%Plot IMU data - I don't know which axes we want so I'm gonna stick em all
%on a plot for now
figure
%plot(t_IMU, IMU(:, 1));
hold on
plot(t_IMU, IMU(:, 2));
plot(t_IMU, IMU(:, 3));
plot(t_IMU, IMU(:, 4));
plot(t_IMU, IMU(:, 5));
plot(t_IMU, IMU(:, 6));
plot(t_IMU, IMU(:, 7));
plot(t_IMU, IMU(:, 8));
hold off
xlabel('Time [not real units currently!');
ylabel('IMU reading [?]');
title ('IMU data');
legend('2', '3','4', '5', '6', '7', '8', 'Location','Best');