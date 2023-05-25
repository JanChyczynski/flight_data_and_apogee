close all
clear all
clc
addpath('Final')
addpath('Quaternions');

g = 9.81;

%Has to be changed according to dataset
DataSet = load('..\Data\data.mat');%csvread("..\Data\spiralStairs_GaitTracking.csv");
DataSet = DataSet.data;
startTime = 0; %4
stopTime = 8; %47

samplePeriod = 1/100;


%extract data from DataSet
time = DataSet(:,1);
gyrX = DataSet(:,2);    %Degrees/s
gyrY = DataSet(:,3);
gyrZ = DataSet(:,4);
accX = DataSet(:,5);    %g
accY = DataSet(:,6);
accZ = DataSet(:,7);
magX = DataSet(:,8).*1000;    %Gauß = e-4 T
magY = DataSet(:,9).*1000;
magZ = DataSet(:,10).*1000;

%Cut off a bit of start and ending period
time = 0:samplePeriod:time(end,1);
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel)';
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);
magX = magX(indexSel, :);
magY = magY(indexSel, :);
magZ = magZ(indexSel, :);

T_s = 0.04;
A = [1 -T_s;
     0 1];
B = [T_s;
     0];
C = [1 0];

X = 0;
P = 0;

%% Correction

% #############################
% %Acc correction
% % Compute accelerometer magnitude
% acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
% 
% % HP filter accelerometer data
% filtCutOff = 0.001;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
% acc_magFilt = filtfilt(b, a, acc_mag);
% 
% % Compute absolute value
% acc_magFilt = abs(acc_magFilt);
% 
% % LP filter accelerometer data
% filtCutOff = 5;
% [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc_magFilt = filtfilt(b, a, acc_magFilt);
% 
% stationary = acc_magFilt < 0; %0.05
% 
% ####################
% -------------------------------------------------------------------------
% Compute orientation
% 
% quat = zeros(length(time), 4);
% AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit', 1);
% 
% % Initial convergence
% initPeriod = 2;
% indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
% for i = 1:2000
%     AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
% end
% 
% % For all data
% for t = 1:length(time)
% %     if(stationary(t))
% %         AHRSalgorithm.Kp = 0.5;
% %     else
% %         AHRSalgorithm.Kp = 0;
% %     end
%     AHRSalgorithm.Kp = 0; % TODO
%     AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
%     quat(t,:) = AHRSalgorithm.Quaternion;
% end
% 
% -------------------------------------------------------------------------
% Compute translational accelerations
%
% Rotate body accelerations to Earth frame
% acc = quaternRotate([accX accY accZ], quaternConj(quat));

% ##################
% acc = acc * g;
% accX = acc(:,1);
% accY = acc(:,2);
% accZ = acc(:,3)-g;
% ##########################################

% %Acc correction
% acc0_mean(1,1) = mean(accX(:,1));
% acc0_mean(2,1) = mean(accY(:,1));
% acc0_mean(3,1) = mean(accZ(:,1));
% 
% accX(:,1) = accX(:,1)-acc0_mean(1,1);
% accY(:,1) = (accY(:,1)-acc0_mean(2,1))*(1);    % Correction y 
% accZ(:,1) = (accZ(:,1)+(1-acc0_mean(3,1)))*(1);% Correction z
% 
% %Gyr correction
% gyro_Factor_Conversion = 131.0;
% 
% %Calculation of the average
% gyr0_mean(1,1) = mean(gyrX(:,1)/gyro_Factor_Conversion);
% gyr0_mean(2,1) = mean(gyrY(:,1)/gyro_Factor_Conversion);
% gyr0_mean(3,1) = mean(gyrZ(:,1)/gyro_Factor_Conversion);
% 
% 
% gyrX(:,1) =  gyrX(:,1)/gyro_Factor_Conversion-gyr0_mean(1,1);
% gyrY(:,1) = (gyrY(:,1)/gyro_Factor_Conversion-gyr0_mean(2,1));% Correction y
% gyrZ(:,1) = (gyrZ(:,1)/gyro_Factor_Conversion-gyr0_mean(3,1));% Correction z

%Needed for mag correction
mag_0_Yaw = atan2(-magY(:),magX(:))*180/pi;
mag_0_Yaw_Mean = mean(mag_0_Yaw);
mag_Signal = -1;

%Mag correction
yaw_mag_cor(1,1)= 0;
for i=1:size(magX,1)
    %Yaw = atan2(-ay,ax);
    yaw_mag(i,1) = atan2(magY(i),magX(i))*180/pi;   %Transformation to degrees
    
    if i==1
        yaw_mag_cor(1,1) = yaw_mag(i,1)-mag_0_Yaw_Mean;
    else
        if abs(yaw_mag(i-1,1))>150 && abs(yaw_mag(i,1))>150 && sign(yaw_mag(i,1)) ~= sign(yaw_mag(i-1,1))
            if(sign(yaw_mag(i-1,1))>0)
                yaw_mag_cor(i,1) = yaw_mag_cor(i-1,1) + (360-abs(yaw_mag(i,1))-abs(yaw_mag(i-1,1)));
            else
                yaw_mag_cor(i,1) = yaw_mag_cor(i-1,1) - (360-abs(yaw_mag(i,1))-abs(yaw_mag(i-1,1))) ;
            end
        else
             if(yaw_mag(i,1) >= yaw_mag(i-1,1))
                 yaw_mag_cor(i,1) = yaw_mag_cor(i-1,1) - mag_Signal*abs(yaw_mag(i,1)-yaw_mag(i-1,1)); 

             elseif(yaw_mag(i,1)<yaw_mag(i-1,1))
                 yaw_mag_cor(i,1) = yaw_mag_cor(i-1,1) + mag_Signal*abs(yaw_mag(i,1)-yaw_mag(i-1,1));
            end
        end       
    end
end

%% Pitch angle
X_theta_save = zeros(2,size(gyrY,1));
Y_theta_save = zeros(1,size(gyrY,1));
P_theta_save = zeros(2,size(gyrY,1)*2);

R_pitch = 1;
Q_pitch = [80 0;
           0 80]; %1200; starting with identity and then trial and error to find the best values for the diagonal matrix

for i=1:size(gyrY,1)
    y = atan(-accX(i)/sqrt(accY(i)^2+accZ(i)^2));
    u = gyrY(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [10 0; 
              0 10];
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_pitch, R_pitch);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_pitch, R_pitch);
    end
    
    X_theta_save(:,i) = X; 
    Y_theta_save(1,i) = y;
    P_theta_save(:,i:i+1) = P; %P is a diagonal matrix and the values on the diagonal are equal
end

close all
figure('Name','Pitch angle');
plot(Y_theta_save(1,:)*180/pi)
hold
plot(X_theta_save(1,:)*180/pi)
legend('Theta_m_e_a_s_u_r_e_d','Theta_e_s_t_i_m_a_t_e_d');

%% Roll angle
X_phi_save = zeros(2,size(gyrX,1));
Y_phi_save = zeros(1,size(gyrX,1));
P_phi_save = zeros(2,size(gyrX,1)*2);
R_roll = 1;
Q_roll = [35 0;
           0 35]; %1; starting with identity and then trial and error to find the best values for the diagonal matrix

for i=1:size(gyrX,1)
    y = atan(accY(i)/accZ(i));
    u = gyrX(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [10 0; 
              0 10];
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_roll, R_roll);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_roll, R_roll);
    end
    
    X_phi_save(:,i) = X; 
    Y_phi_save(1,i) = y;
    P_phi_save(:,i:i+1) = P; %P is a diagonal matrix and the values on the diagonal are equal
end

figure('Name','Roll angle');
plot(Y_phi_save(1,:))
hold
plot(X_phi_save(1,:))
legend('Phi_m_e_a_s_u_r_e_d','Phi_e_s_t_i_m_a_t_e_d');

%% Yaw angle
X_yaw_save = zeros(2,size(gyrZ,1));
Y_yaw_save = zeros(1,size(gyrZ,1));
P_yaw_save = zeros(2,size(gyrZ,1)*2);
R_yaw = 3;
Q_yaw = [3 0;
           0 3]; %0.5; starting with identity and then trial and error to find the best values for the diagonal matrix


for i=1:size(magX,1)
    
    y = yaw_mag_cor(i,1);
    u = gyrZ(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [50 0; 
              0 50];
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_yaw, R_yaw);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_yaw, R_yaw);
    end
    
    X_yaw_save(:,i) = X; 
    Y_yaw_save(1,i) = y;
    P_yaw_save(:,i:i+1) = P; %P is a diagonal matrix and the values on the diagonal are equal
end

figure('Name','Yaw angle');
plot(Y_yaw_save(1,:))
hold
plot(X_yaw_save(1,:))
legend('Psi_m_e_a_s_u_r_e_d','Psi_e_s_t_i_m_a_t_e_d'); %In Degrees!

%% Derivatives
figure('Name','Derivatives Phi and Theta');
plot(X_phi_save(2,:))
hold
plot(X_theta_save(2,:))
legend('Phi_d_o_t','Theta_d_o_t');

figure('Name','Derivative Psi')
plot(X_yaw_save(2,:))
legend('Psi_d_o_t');

%% Calculating position

for i=1:size(accX,1)
    acc_transf = inv(rotationMatrix3D(X_yaw_save(1,i)*pi/180,X_theta_save(1,i),X_phi_save(1,i)))*[accX(i,1); accY(i,1); accZ(i,1)];
    accX(i,1) = acc_transf(1,1);
    accY(i,1) = acc_transf(2,1);
    accZ(i,1) = acc_transf(3,1);
end


%Numerical integration of accX [g]
velX = zeros(size(accX,1),1);
velX(1,1) = 50*cos(pi/4);   %initial speed
for i=2:size(accX,1)
    velX(i,1) = velX(i-1,1) + ((accX(i,1)-accX(i-1,1))/2+accX(i-1,1))*samplePeriod;
end

posX = zeros(size(accX,1),1);
posX(1,1) = 0;
for i=2:size(accX,1)
    posX(i,1) = posX(i-1,1) + ((velX(i,1)-velX(i-1,1))/2+velX(i-1,1))*samplePeriod;
end

% %Numerical integration of accY [g]
velY = zeros(size(accY,1),1);
velY(1,1) = 0;
for i=2:size(accY,1)
    velY(i,1) = velY(i-1,1) + ((accY(i,1)-accY(i-1,1))/2+accY(i-1,1))*samplePeriod;
end

posY = zeros(size(accY,1),1);
posY(1,1) = 0;
for i=2:size(accY,1)
    posY(i,1) = posY(i-1,1) + ((velY(i,1)-velY(i-1,1))/2+velY(i-1,1))*samplePeriod;
end

%Numerical integration of accZ [g]
velZ = zeros(size(accZ,1),1);
velZ(1,1) = 50*sin(pi/4);  %initial speed
for i=2:size(accZ,1)
    velZ(i,1) = velZ(i-1,1) + ((accZ(i,1)-accZ(i-1,1))/2+accZ(i-1,1))*samplePeriod;
end

posZ = zeros(size(accZ,1),1);
posZ(1,1) = 0;
for i=2:size(accZ,1)
    posZ(i,1) = posZ(i-1,1) + ((velZ(i,1)-velZ(i-1,1))/2+velZ(i-1,1))*samplePeriod;
end


% accX_filt = accX;
% accY_filt = accY;
% accZ_filt = accZ;

% ###############
% cut_off = 0; %0.05
% for i=1:length(accX_filt)    %Find stationary parts
%     if acc_magFilt(i,1) >= -cut_off && acc_magFilt(i,1) <= cut_off
%         accX_filt(i,1) = 0;
%         accY_filt(i,1) = 0;
%         accZ_filt(i,1) = 0;
%     end
% end
% ################

% %Integration of acc
% velX = cumtrapz(samplePeriod,accX_filt)+50*cos(pi/4); %TODO
% velY = cumtrapz(samplePeriod,accY_filt);
% velZ = cumtrapz(samplePeriod,accZ_filt)+50*sin(pi/4); %TODO


% #######################
% vel = [velX velY velZ];
% 
% % Compute integral drift during non-stationary periods
% velDrift = zeros(size(vel));
% stationaryStart = find([0; diff(stationary)] == -1);
% stationaryEnd = find([0; diff(stationary)] == 1);
% for i = 1:numel(stationaryEnd)
%     driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
%     enum = 1:(stationaryEnd(i) - stationaryStart(i));
%     drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
%     velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
% end
% 
% % Remove integral drift
% vel = vel - velDrift;
% 
% velX = vel(:,1);
% velY = vel(:,2);
% velZ = vel(:,3);
% #############################

% %Integration of velocity
% posX = cumtrapz(samplePeriod,velX);
% posY = cumtrapz(samplePeriod,velY);
% posZ = cumtrapz(samplePeriod,velZ);



% %transformation to m
% posX = posX/10; 
% posY = posY/10;
% posZ = posZ/100;

%Plot in 3D
figure('Name','Position 3D')
plot3(posX,posY,posZ)
legend('XYZ')
xlabel('x')
ylabel('y')
zlabel('z')

%Plot X, Y, Z separately over time
figure('Name','Position')
subplot(3,1,1)
plot(time, posX)
xlabel('time [s]')
ylabel('X [m]')
subplot(3,1,2)
plot(time, posY)
xlabel('time [s]')
ylabel('Y [m]')
subplot(3,1,3)
plot(time, posZ)
xlabel('time [s]')
ylabel('Z [m]')

%Plot velocitys over time
figure('Name','Velocity')
hold on;
plot(time, velX)
plot(time, velY)
plot(time, velZ)
hold off;
legend('X','Y','Z')

%% Save X, Y, Z, roll, pitch, yaw
roll = (X_phi_save(1,:))'; 
pitch = (X_theta_save(1,:))';
yaw = (X_yaw_save(1,:)*pi/180)'; %Transformation to radian

%time = time+1;                      %ToDo

%saveVar = [time posX posY posZ pitch yaw roll];
%Call function to plot the measured / calculated trajectory
%plotTrajectory(saveVar);



