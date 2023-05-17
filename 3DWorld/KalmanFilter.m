function [x_corrected, P] = KalmanFilter(A, B, C, u, y_measured, x_predicted_before,  P_before, Q, R)


% Prediction
x_predicted = A*x_predicted_before + B*u;
P_predicted = A*P_before*transpose(A)+Q;

% Estimation
K = P_predicted*transpose(C)*inv(C*P_predicted*transpose(C)+R);
P = P_predicted-K*C*P_predicted;
x_corrected = x_predicted + K*(y_measured - C*x_predicted);

end