%authors: Simon Roß, Pablo Vega Perez

close all
clear all
clc
addpath('Final')

%% PLEASE ADJUST PARAMETERS IN THIS SECTION + Line 223 + weight matrices for Kalman filter
%Import data; has to be changed according to dataset
% DataSet = load('..\Data\data.mat');                   %Parabolic fligth
DataSet = load('..\Data\data_withLateralDrift.mat');    %parabolic fligth with lateral drift (data fpor accY is already in earth frame; it was not calculated correctly by the professor. Therefore, the angles calculated with accY are not 100% correct)
DataSet = DataSet.data;

samplePeriod = 1/100;

%% Process imported data

%extract data from DataSet
time = DataSet(:,1);    %s
gyrX = DataSet(:,2);    %Degrees/s
gyrY = DataSet(:,3);
gyrZ = DataSet(:,4);
accX = DataSet(:,5);    %m/s
accY = DataSet(:,6);
accZ = DataSet(:,7);
magX = DataSet(:,8);    %Gauß = e-4 T
magY = DataSet(:,9);
magZ = DataSet(:,10);

%% Adding random noise to values (since program works with ideal data!)

snr = 20;

accX = awgn(accX,snr,'measured');
accY = awgn(accY,snr,'measured');
accZ = awgn(accZ,snr,'measured');

gyrX = awgn(gyrX,snr,'measured');
gyrY = awgn(gyrY,snr,'measured');
gyrZ = awgn(gyrZ,snr,'measured');


%% Kalman filter parameters

T_s = 0.04;
A = [1 -T_s;
     0 1];
B = [T_s;
     0];
C = [1 0];

X = 0;
P = 0;

%% Correction
%Source: Course "UAV Guidance & Autonomous Control" at UPC ESEIAAT

%Needed for magnetometer correction
mag_0_Yaw = atan2(-magY(:),magX(:))*180/pi;
mag_0_Yaw_Mean = mean(mag_0_Yaw);
mag_Signal = -1;

%Mag correction
yaw_mag_cor(1,1)= 0;
for i=1:size(magX,1)
    yaw_mag(i,1) = atan2(-magY(i),magX(i))*180/pi;   %Transformation to degrees
    
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

%% Pitch angle - Kalman filter
X_theta_save = zeros(2,size(gyrY,1));
Y_theta_save = zeros(1,size(gyrY,1));
P_theta_save = zeros(2,size(gyrY,1)*2);

R_pitch = 1;
Q_pitch = [0.001 0;
           0 0.001]; %starting with identity and then trial and error to find the best values for the diagonal matrix

for i=1:size(gyrY,1)
    y = atan(-accX(i)/sqrt(accY(i)^2+accZ(i)^2));
    u = gyrY(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [5 0; 
              0 5]; %trial and error
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_pitch, R_pitch);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_pitch, R_pitch);
    end
    
    X_theta_save(:,i) = X; 
    Y_theta_save(1,i) = y;
    P_theta_save(:,i:i+1) = P; 
end

figure('Name','Pitch angle');
plot(Y_theta_save(1,:)*180/pi)
hold
plot(X_theta_save(1,:)*180/pi)
ylabel('[Degrees]')
legend('Theta_m_e_a_s_u_r_e_d','Theta_e_s_t_i_m_a_t_e_d');

%% Roll angle - Kalman filter
X_phi_save = zeros(2,size(gyrX,1));
Y_phi_save = zeros(1,size(gyrX,1));
P_phi_save = zeros(2,size(gyrX,1)*2);
R_roll = 1;
Q_roll = [0.01 0;
           0 0.01]; %starting with identity and then trial and error to find the best values for the diagonal matrix

for i=1:size(gyrX,1)
    y = atan(accY(i)/accZ(i));
    u = gyrX(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [5 0; 
              0 5]; %trial and error
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_roll, R_roll);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_roll, R_roll);
    end
    
    X_phi_save(:,i) = X; 
    Y_phi_save(1,i) = y;
    P_phi_save(:,i:i+1) = P;
end

figure('Name','Roll angle');
plot(Y_phi_save(1,:)*180/pi)
hold
plot(X_phi_save(1,:)*180/pi)
ylabel('[Degrees]')
legend('Phi_m_e_a_s_u_r_e_d','Phi_e_s_t_i_m_a_t_e_d');

%% Yaw angle - Kalman filter
X_yaw_save = zeros(2,size(gyrZ,1));
Y_yaw_save = zeros(1,size(gyrZ,1));
P_yaw_save = zeros(2,size(gyrZ,1)*2);
R_yaw = 1;
Q_yaw = [1 0;
           0 1]; %starting with identity and then trial and error to find the best values for the diagonal matrix


for i=1:size(magX,1)
    
    y = yaw_mag_cor(i,1);
    u = gyrZ(i);

    if i==1
        X0 = [0; 
              0];
        P0 = [1 0; 
              0 1]; %trial and error
        [X, P] = KalmanFilter(A, B, C, u, y, X0, P0, Q_yaw, R_yaw);
    else
        
        [X, P] = KalmanFilter(A, B, C, u, y, X, P, Q_yaw, R_yaw);
    end
    
    X_yaw_save(:,i) = X; 
    Y_yaw_save(1,i) = y;
    P_yaw_save(:,i:i+1) = P;
end

figure('Name','Yaw angle');
plot(Y_yaw_save(1,:))       %yaw is already in Degrees!
hold
plot(X_yaw_save(1,:))
ylabel('[Degrees]')
legend('Psi_m_e_a_s_u_r_e_d','Psi_e_s_t_i_m_a_t_e_d');

%% Derivatives
figure('Name','Derivatives Phi and Theta');
plot(X_phi_save(2,:))
hold
plot(X_theta_save(2,:))
legend('Phi_d_o_t','Theta_d_o_t');
ylabel('[rad/s]')

figure('Name','Derivative Psi')
plot(X_yaw_save(2,:))
legend('Psi_d_o_t');
ylabel('[Degrees/s]')

%% Calculating position

%Rotating acc vector from body to earth frame using angles calculated /
%estimated by Kalman filter
for i=1:size(accX,1)
    acc_transf = inv(rotationMatrix3D(X_yaw_save(1,i)*pi/180,X_theta_save(1,i),X_phi_save(1,i)))*[accX(i,1); accY(i,1); accZ(i,1)];
    accX(i,1) = acc_transf(1,1);
    accY(i,1) = acc_transf(2,1);
    accZ(i,1) = acc_transf(3,1);
end

%################################
accY = DataSet(:,6);    %ONLY INCLUDE THIS LINE WHEN USING LATERAL DRIFT DATA, because the acceleration in y direction is already given in body frame in the imported data (was not calculated correctly by the professor) 
%################################

% Numerical integration of accX (trapezoidal rule)
velX = zeros(size(accX,1),1);
velX(1,1) = 50*cos(pi/4);                           %INITIAL SPEED
for i=2:size(accX,1)
    velX(i,1) = velX(i-1,1) + ((accX(i,1)-accX(i-1,1))/2+accX(i-1,1))*samplePeriod;
end

posX = zeros(size(accX,1),1);
posX(1,1) = 0;
for i=2:size(accX,1)
    posX(i,1) = posX(i-1,1) + ((velX(i,1)-velX(i-1,1))/2+velX(i-1,1))*samplePeriod;
end

% Numerical integration of accY (trapezoidal rule)
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

% Numerical integration of accZ (trapezoidal rule)
velZ = zeros(size(accZ,1),1);
velZ(1,1) = 50*sin(pi/4);                           %INITIAL SPEED
for i=2:size(accZ,1)
    velZ(i,1) = velZ(i-1,1) + ((accZ(i,1)-accZ(i-1,1))/2+accZ(i-1,1))*samplePeriod;
end

posZ = zeros(size(accZ,1),1);
posZ(1,1) = 0;
for i=2:size(accZ,1)
    posZ(i,1) = posZ(i-1,1) + ((velZ(i,1)-velZ(i-1,1))/2+velZ(i-1,1))*samplePeriod;
end


% Plot position in 3D
figure('Name','Position 3D')
plot3(posX,posY,posZ)
legend('XYZ')
xlabel('x')
ylabel('y')
zlabel('z')

% Plot X, Y, Z position separately over time
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

% Plot velocities over time
figure('Name','Velocity')
hold on;
plot(time, velX)
plot(time, velY)
plot(time, velZ)
hold off;
legend('X','Y','Z')

%% Start Animation
roll = (X_phi_save(1,:))'; 
pitch = (X_theta_save(1,:))';
pitch = pitch-pi/4;     %Correction needed for the animation
yaw = (X_yaw_save(1,:)*pi/180)'; %Transformation to radian

saveVar = [time posX posY posZ pitch yaw roll];

% Call function to show animation of the calculated trajectory
plotTrajectory(saveVar);



