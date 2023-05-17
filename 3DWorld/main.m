close all
clear all
clc

g = 9.81;
DataSet = csvread("..\Data\data_calib_2.csv");

%extract data from DataSet
time = DataSet(:,1);
gyrX = DataSet(:,2);
gyrY = DataSet(:,3);
gyrZ = DataSet(:,4);
accX = DataSet(:,5);
accY = DataSet(:,6);
accZ = DataSet(:,7);
magX = DataSet(:,8);
magY = DataSet(:,9);
magZ = DataSet(:,10);

T_s = 0.04;
A = [1 -T_s;
     0 1];
B = [T_s;
     0];
C = [1 0];

X = 0;
P = 0;


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
plot(Y_theta_save(1,:))
hold
plot(X_theta_save(1,:))
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

%Needed for correction
mag_0_Yaw = atan2(-magY(:),magX(:))*180/pi;
mag_0_Yaw_Mean = mean(mag_0_Yaw);
mag_Signal = -1;

%Correction
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

%Numerical integration of accX [g]
posX = zeros(size(accX,1)+1,1);
posX(1,1) = 0;
for i=2:size(accX,1)
    if i==2
        posX(i,1) = accX(i-1,1)/g*(time(i-1,1)-0)^2;
    else
        posX(i,1) = posX(i-1,1) + accX(i-1,1)/g*(time(i-1,1)-time(i-2,1))^2;
    end
end

%Numerical integration of accY [g]
posY = zeros(size(accY,1)+1,1);
posY(1,1) = 0;
for i=2:size(accY,1)
    if i==2
        posY(i,1) = accY(i-1,1)/g*(time(i-1,1)-0)^2;
    else
        posY(i,1) = posY(i-1,1) + accY(i-1,1)/g*(time(i-1,1)-time(i-2,1))^2;
    end
end

%Numerical integration of accZ [g]
posZ = zeros(size(accZ,1)+1,1);
posZ(1,1) = 0;
for i=2:size(accZ,1)
    if i==2
        posZ(i,1) = accZ(i-1,1)/g*(time(i-1,1)-0)^2;
    else
        posZ(i,1) = posZ(i-1,1) + accZ(i-1,1)/g*(time(i-1,1)-time(i-2,1))^2;
    end
end

%transformation from cm to m
posX = posX/100; 
posY = posY/100;
posZ = posZ/100;

%Plot in 3D
figure('Name','Position 3D')
plot3(posX,posY,posZ)
legend('XYZ')

%Plot X, Y, Z separately over time
figure('Name','Position')
subplot(3,1,1)
plot([0; time], posX)
xlabel('time [s]')
ylabel('X [m]')
subplot(3,1,2)
plot([0; time], posY)
xlabel('time [s]')
ylabel('Y [m]')
subplot(3,1,3)
plot([0; time], posZ)
xlabel('time [s]')
ylabel('Z [m]')

%% Save X, Y, Z, roll, pitch, yaw
roll = X_phi_save(1,:)';
pitch = X_theta_save(1,:)';
yaw = (X_yaw_save(1,:)*pi/180)'; %Transformation to radian

time = time+1;                      %ToDo

saveVar = [[0; time] posX posY posZ [pitch(1,1); pitch] [yaw(1,1); yaw] [roll(1,1); roll]];
%Call function to plot the measured / calculated trajectory
plotTrajectory(saveVar);
