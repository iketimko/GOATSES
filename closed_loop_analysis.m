%% GOATSES BendLabs and IMU Integration Testing
%QM, CW, IT, AH
%4/2/22
%

%% housekeeping
clear 
clc
close all

%% Read in bendlabs data
BL_R = readmatrix('ClosedLoop6_R.log');
BL_L = readmatrix('ClosedLoopTest6_L.log');
% remove NaN values
BL_L = BL_L(10:end-2,:);
BL_R = BL_R(1:end-2,:);

%% Read in IMU data
IMU = csvread('ClosedLoop6_IMU_Both_.csv');
time = IMU(:,1)/1000;
t_start = find(time > 20, 1);
t = time(t_start:end);

%% Compute the Bendlabs sensor sanmpling rate
deltat = time(end)-time(1);
f_s = length(BL_R(:,1))/deltat;
fprintf('The closed loop operation frequency is: %3.2f Hz \n', f_s)

t_BL_R = linspace(0,length(BL_R(:,1)) /f_s, length(BL_R(:,1)) );
t_BL_L = linspace(0,length(BL_L(:,1)) /f_s, length(BL_L(:,1)) );

%% Compute the Sway parameters
compass_data = rmoutliers(IMU(t_start:end, 8));
compass_data = compass_data-mean(compass_data);

figure 
plot(t(1:end-13),compass_data)
title('Compass Data')
ylabel('Degrees')
xlabel('Seconds')

fs_IMU = 1000;
Y = fft(compass_data);
L = deltat;
P2 = abs(Y/L);
P1 = P2(1:L/2+1);

P1(2:end-1) = 2*P1(2:end-1);
f = fs_IMU*(0:(L/2))/(700*L);

figure
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of The Compass Measurements')
xlabel('f (Hz)')
ylabel('|P1(f)|')

disp('From the IMU Data')
disp('***********************************************')
disp('The sway frequency is: 0.122 Hz')
fprintf('The recorded lean angle +/- %2.2f degrees \n', mean([max(compass_data),-min(compass_data)]))

%% Compute the resultant Vector R
alpha = BL_L(:,1);
beta = BL_R(:,1);
R = @(F1,F2,alpha,beta) [-sind(alpha), sind(beta); cosd(alpha), cosd(beta)]*[F1; F2];

%% Plot the Resultant Vector values
F = 10;
resultant = zeros(2,length(BL_L));
theta_err = zeros(1,length(BL_L));

for i = 1:length(BL_L)
    resultant(:,i) = R(F ,F, BL_L(i,1) ,BL_R(i,1));
    theta_err(i) = atand(resultant(1,i)/resultant(2,i));
end

figure
plot(t_BL_R, theta_err)
yline(2)
yline(-2)
ylim([-3 3])
title('Error in the vector angle')
ylabel('Degrees')
xlabel('Seconds')

%% Plot Bend Labs data
figure
plot(t_BL_R, BL_R(:,1));
hold on
plot(t_BL_L, BL_L(:,1));
hold off
xlabel('Time [s]');
ylabel('Angle [degrees]');
title('BendLabs data')
legend('Right Sensor', 'Left Sensor', 'Location', 'Best');

%% Plot IMU Data
figure
hold on
plot(time, IMU(:, 2));
plot(time, IMU(:, 3));
plot(time, IMU(:, 4));
plot(time, IMU(:, 5));
plot(time, IMU(:, 6));
plot(time, IMU(:, 7));
plot(time, IMU(:, 8));
hold off
xlabel('Time [ms]');
ylabel('degrees/m');
title ('IMU data');
legend('x', 'y','z', 'phi', 'theta', 'psi', 'compass', 'Location','Best');

figure
hold on

plot(t, IMU(t_start:end, 3));
plot(t, IMU(t_start:end, 4));
plot(t, IMU(t_start:end, 7));
%plot(t, IMU(t_start:end, 8));
plot(t(1:end-13), rmoutliers(IMU(t_start:end, 8)));
hold off
xlabel('Time [s]');
ylabel('degrees/m');
title ('IMU data');
legend('y','z','psi', 'compass', 'Location','Best');
