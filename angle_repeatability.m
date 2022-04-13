%% GOATSES BendLabs and IMU Integration Testing
%QM, CW, IT, AH
%4/2/22
%

%% housekeeping
clear 
clc
close all

%% Plot the angle repeatablilty

figure 
hold on
title('Sensor Data')
ylabel('Degrees')
xlabel('Seconds')

% Test 6
BL_R_6 = readmatrix('ClosedLoop6_R.log');
BL_L_6 = readmatrix('ClosedLoopTest6_L.log');
% remove NaN values and ensure consistant vector lengths
BL_L{6}(:,:) = BL_L_6(10:end-2,:);
BL_R{6}(:,:) = BL_R_6(1:end-2,:);
%plot the values
plot(BL_L{6}(:,1))
plot(BL_R{6}(:,1))

% Test 5
BL_R_5 = readmatrix('ClosedLoop5_R.log');
BL_L_5 = readmatrix('ClosedLoopTest5_L.log');
% remove NaN values and ensure consistant vector lengths
BL_L{5}(:,:) = BL_L_5(1:end-2-(length(BL_L_5)-length(BL_R_5)),:);
BL_R{5}(:,:) = BL_R_5(1:end-2,:);
%plot the values
plot(BL_L{5}(:,1))
plot(BL_R{5}(:,1))

% Test 4
BL_R_4 = readmatrix('ClosedLoop4_R.log');
BL_L_4 = readmatrix('ClosedLoopTest4_L.log');
% remove NaN values and ensure consistant vector lengths
BL_L{4}(:,:) = BL_L_4(1:end-2-(length(BL_L_4)-length(BL_R_4)),:);
BL_R{4}(:,:) = BL_R_4(1:end-2,:);
%plot the values
plot(BL_L{4}(:,1))
plot(BL_R{4}(:,1))

% Test 3
BL_R_3 = readmatrix('ClosedLoop3_R.log');
BL_L_3 = readmatrix('ClosedLoopTest3_L.log');
% remove NaN values and ensure consistant vector lengths
BL_L{3}(:,:) = BL_L_3(1:end-2-(length(BL_L_3)-length(BL_R_3)),:);
BL_R{3}(:,:) = BL_R_3(1:end-2,:);
%plot the values
plot(BL_L{3}(:,1))
plot(BL_R{3}(:,1))

% % Test 2
% BL_R_2 = readmatrix('ClosedLoop2_R.log');
% BL_L_2 = readmatrix('ClosedLoopTest2_L.log');
% % remove NaN values and ensure consistant vector lengths
% BL_L{2}(:,:) = BL_L_2(1:end-2-(length(BL_L_2)-length(BL_R_2)),:);
% BL_R{2}(:,:) = BL_R_2(1:end-2,:);
% %plot the values
% plot(BL_L{2}(:,1))
% plot(BL_R{2}(:,1))

% % Test 1
% BL_R_1 = readmatrix('ClosedLoop1_R.log');
% BL_L_1 = readmatrix('ClosedLoopTest1_L.log');
% % remove NaN values and ensure consistant vector lengths
% BL_L{1}(:,:) = BL_L_1(1:end-2-(length(BL_L_1)-length(BL_R_1)),:);
% BL_R{1}(:,:) = BL_R_1(1:end-2,:);
% %plot the values
% plot(BL_L{1}(:,1))
% plot(BL_R{1}(:,1))

F = 66.7;

figure
hold on

for j = 3:6
    % Compute the resultant Vector R
    R = @(F1,F2,alpha,beta) [-sind(alpha), sind(beta); cosd(alpha), cosd(beta)]*[F1; F2];

    setpoint = mean([mean(BL_L{j}(:,1)) mean(BL_R{j}(:,1))]);
    resultant = zeros(2,length(BL_L{j}));
    theta_err = zeros(1,length(BL_L{j}));

    for i = 1:length(BL_L{j})
        resultant(:,i) = R(F ,F, BL_L{j}(i,1) ,BL_R{j}(i,1));
        theta_err(i) = atand(resultant(1,i)/resultant(2,i));
    end
    t = linspace(0,length(BL_L{j})/56.1655,length(BL_L{j}));
    plot(t, theta_err)
    M(j) = mean(theta_err);
end

yline(2)
yline(-2)
% ylim([-3 3])
title('Error in the vector angle')
ylabel('Degrees')
xlabel('Seconds')

%% Plot the results with error

% define erors 
repeatability_error = sqrt(mean(M))
sensor_error = .2;
    
h = figure;
scale = 1;
h.Position = [10 50 scale*1500 scale*800];
set(gcf,'color','w');
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'repeatability.gif';
i = 1;
for n = 1:round(length(t)/(30*10)):length(t)
    clf
    hold on

    % Sensor error
    x_patch = [t(1:n), flip(t(1:n))];
    y_patch = [theta_err(1:n)+repeatability_error+sensor_error, flip(theta_err(1:n)-repeatability_error-sensor_error)];
    patch(x_patch,y_patch,'yellow','FaceAlpha',1)

    % Repeatability error
    x_patch = [t(1:n), flip(t(1:n))];
    y_patch = [theta_err(1:n)+repeatability_error, flip(theta_err(1:n)-repeatability_error)];
    patch(x_patch,y_patch,'cyan','FaceAlpha',1)

    % Plot the line
    plot(t(1:n),theta_err(1:n),'k','Linewidth', 2)

    yline(2,'r--','Linewidth', 3)
    yline(-2,'r--','Linewidth', 3)
    legend('Sensor error', 'Repeatability error', 'Vector Angle Error','Location','SE');
    ylim([-3,3])
    xlim([0,t(end)]);
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