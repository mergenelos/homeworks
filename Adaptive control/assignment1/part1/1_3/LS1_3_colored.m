%% Clear everything
clear;close all;clc;

%% Generate white noise with variance of 1
desired_variance = 1;
signal_length = 3334; 
standard_noise = randn(signal_length, 1);
current_variance = var(standard_noise);
scaling_factor = sqrt(desired_variance / current_variance);
white_noise = standard_noise * scaling_factor;
actual_variance = var(white_noise);
fprintf('Desired variance: %.2f\n', desired_variance);
fprintf('Actual variance: %.2f\n', actual_variance);

%% Generate output noise 
signal_length = 3334; 
desired_output_variance = 1; 
output_noise = randn(signal_length, 1);
current_output_variance = var(output_noise);
output_scaling_factor = sqrt(desired_output_variance / current_output_variance);
output_noise = output_noise * output_scaling_factor;
output_noise = normalize(output_noise, 'range', [-1 1]);
actual_output_variance = var(output_noise);
%figure;plot(output_noise);grid on;

%% Import Simulink model
sim("LS1_3_clrd.slx")

%% Prepare Data
u = squeeze(ans.input.Data)';
y = squeeze(ans.output.Data)';
yWithoutNoise = squeeze(ans.outputWithoutNoise.Data)';
t = ans.time.Data';

%% Calculate Phi
N = length(u);
phi = [-y(3:N-1)' -y(2:N-2)' -y(1:N-3)' u(3:N-1)' u(2:N-2)' u(1:N-3)'];

%% Calculate Theta_hat
Y = y(4:N)';
theta_hat = inv(phi'*phi)*phi'*Y;

%% Calculate model output and its error
YP = phi*theta_hat;
YActual = yWithoutNoise(4:N)';
error = YActual-YP;
MSE = mse(error);

%% Plot output and estimated output
y_hat = phi*theta_hat;
t=1:length(y_hat);
figure;plot(t,yWithoutNoise(:,1:length(y_hat)),t,y_hat);fontsize( 24 ,"points");legend('measured','simulated');grid on;hold on;
title("LS parameter estimation - White noise input and colored noise on output");xlabel('Sample Number');ylabel('Amplitude');

%% Recreate system model
sysCSD = tf(theta_hat(4:6)',[1 theta_hat(1:3)'],0.3)
sysCS = d2c(sysCSD)



