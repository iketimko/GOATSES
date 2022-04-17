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
t_BL = t_BL_R;

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

setpoint = mean([mean(BL_L(:,1)) mean(BL_R(:,1))]);

%% Plot the Resultant Vector values
F = 66.7;
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



%% plot the load magnitude error
F = linspace(0,1500,100);
LME = zeros(1,length(F));

for j = 1:length(F)
    resultant = zeros(2,length(BL_L));
    resultant_IDEAL = zeros(2,length(BL_L));
    theta_err = zeros(1,length(BL_L));
    loadmag_err = zeros(1,length(BL_L));

    for i = 1:length(BL_L)
        resultant(:,i) = R(F(j),F(j) , BL_L(i,1) ,BL_R(i,1));
        resultant_IDEAL(:,i) = R(F(j), F(j), setpoint , setpoint);
        theta_err(i) = atand(resultant(1,i)/resultant(2,i));
        loadmag_err(i) = norm(resultant(:,i))-norm(resultant_IDEAL(:,i));
    end
    LME(j) = max(loadmag_err);
end

figure
hold on
plot(F(1:42), LME(1:42))
plot(F(42:end), LME(42:end),'--')
yline(10,'--r')
ylim([0, 12])
xline(1112.06/(2*sind(setpoint)),'--k')
title('Error in the force Magnitude')
legend('Load Magnitude Error', 'Extrapolated Data', 'Magnitude Requirement', 'Maximum Design Load', 'Location', 'SE')
ylabel('Loading error [N]')
xlabel('Counterweight Weight [N]')

%% Plot BendLabs data
figure
plot(t_BL_R, BL_R(:,1));
hold on
plot(t_BL_L, BL_L(:,1));
% plot(t_BL_R, sin(2*pi*.093*t_BL_R)+mean(BL_R(:,1)));
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

%% generate a gif of the plot
if true %for debugging 
    h = figure;
    scale = 1;
    h.Position = [10 50 scale*1500 scale*800];
    set(gcf,'color','w');
    axis tight manual % this ensures that getframe() returns a consistent size
    filename = 'compare.gif';
    i = 1;
    for n = 1:round(length(t_BL)/(30*10)):length(t_BL)
        clf
        % for Position
        subplot(2,1,1)
        plot(t_BL_R(1:n), BL_R(1:n,1),'Linewidth', 2)
        hold on
        plot(t_BL_L(1:n), BL_L(1:n,1),'Linewidth', 2)
        yline(setpoint,'b--','Linewidth', 2)
        legend('Right Sensor','Left Sensor','Setpoint','Location','SE')
        title('Angle Sensor Measurements','FontSize', 14)
        xlabel('Time (s)','FontSize', 12)
        ylabel('Measured Angle (Deg)','FontSize', 12)
        xlim([0,t_BL(end)]);
        ylim([39,44]);
        % for Error
        subplot(2,1,2);
        plot(t_BL(1:n),theta_err(1:n),'Linewidth', 2)
        hold on
    %     x_patch = [time(1:n), flip(time(1:n))];
    %     y_patch = [error_vec+angle_error, flip(error_vec-angle_error)];
    %     patch(x_patch,y_patch,'yellow','FaceAlpha',.3)
        yline(2,'r--','Linewidth', 3)
        yline(-2,'r--','Linewidth', 3)
        legend('Time Delay', 'System Error','Location','SE');
        ylim([-2.1,2.1])
        xlim([0,t_BL(end)]);
        title('Error in Vector Angle','FontSize', 14);
        xlabel('Time (s)','FontSize', 12)
        ylabel('Angle Error (Deg)','FontSize', 12);

        drawnow 

        % Capture the plot as an image 
        frame = getframe(h); 
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256); 

        % Write to the GIF File 
        if n == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
        else 
          imwrite(imind,cm,filename,'gif','DelayTime',1/30,'WriteMode','append'); 
        end 

    end
end