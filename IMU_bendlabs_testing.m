%% GOATSES BendLabs and IMU Integration Testing
%QM, CW, IT, AH
%4/2/22
%

%% housekeeping
clear 
clc
close all

%% set filenames

%filename_BL = 'TestIMU';
filename_IMU = 'LOG1.csv';

%% Read in bendlabs data
BL = readmatrix('ClosedLoopTest2.log');

%% Read in IMU data
IMU = csvread('LOG7.csv');
IMU1 = csvread('LOG1.csv');
IMU2 = csvread('LOG2.csv');
IMU3 = csvread('LOG3.csv');
IMU4 = csvread('LOG4.csv');
IMU5 = csvread('LOG5.csv');
IMU6 = csvread('LOG6.csv');

IMU_BL = csvread('LOG14.csv');
%Time vector for BL data
t_BL = 1:length(BL(:, 1));


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
%plot(IMU(:, 1), IMU(:, 1));
hold on
time = IMU_BL(:,1)/1000;
plot(time, IMU_BL(:, 2));
plot(time, IMU_BL(:, 3));
plot(time, IMU_BL(:, 4));
plot(time, IMU_BL(:, 5));
plot(time, IMU_BL(:, 6));
plot(time, IMU_BL(:, 7));
plot(time, IMU_BL(:, 8));
hold off
xlabel('Time [ms]');
ylabel('IMU reading [?]');
title ('IMU data');
legend('x', 'y','z', 'phi', 'theta', 'psi', 'compass', 'Location','Best');

%plot the compass headings
offset = mean(IMU2(:, 8));
error = 2;
figure
hold on
plot(IMU2(:,1)/1000, IMU2(:, 8)-offset);
plot(IMU3(:,1)/1000, IMU3(:, 8)-offset);
plot(IMU4(:,1)/1000, IMU4(:, 8)-offset);
plot(IMU5(:,1)/1000, IMU5(:, 8)-offset);
plot(IMU6(:,1)/1000, IMU6(:, 8)-offset);

yline(0);
yline(-30);
yline(-45);
yline(-60);
yline(-90);

fill([0, IMU2(end,1)/1000, IMU2(end,1)/1000, 0],[mean(IMU2(:, 8))-offset+error*ones(1,2),mean(IMU2(:, 8))-offset-error*ones(1,2)],'cyan','FaceAlpha',0.2,'EdgeAlpha',0)
fill([0, IMU3(end,1)/1000, IMU3(end,1)/1000, 0],[mean(IMU3(:, 8))-offset+error*ones(1,2),mean(IMU3(:, 8))-offset-error*ones(1,2)],'cyan','FaceAlpha',0.2,'EdgeAlpha',0)
fill([0, IMU4(end,1)/1000, IMU4(end,1)/1000, 0],[mean(IMU4(:, 8))-offset+error*ones(1,2),mean(IMU4(:, 8))-offset-error*ones(1,2)],'cyan','FaceAlpha',0.2,'EdgeAlpha',0)
fill([0, IMU5(end,1)/1000, IMU5(end,1)/1000, 0],[mean(IMU5(:, 8))-offset+error*ones(1,2),mean(IMU5(:, 8))-offset-error*ones(1,2)],'cyan','FaceAlpha',0.2,'EdgeAlpha',0)
fill([0, IMU6(end,1)/1000, IMU6(end,1)/1000, 0],[mean(IMU6(:, 8))-offset+error*ones(1,2),mean(IMU6(:, 8))-offset-error*ones(1,2)],'cyan','FaceAlpha',0.2,'EdgeAlpha',0)

hold off

ylim([-100, 10])
xlabel('Time [ms]');
ylabel('Compass reading');
title ('IMU Compass data');
legend('0^\circ', '30^\circ','45^\circ', '60^\circ', '90^\circ', 'Location','Best');