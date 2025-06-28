%% Clear everything
clear;close all;clc;

%% Generate white noise with variance of 2
desired_variance = 2;
signal_length = 3334; 
standard_noise = randn(signal_length, 1);
current_variance = var(standard_noise);
scaling_factor = sqrt(desired_variance / current_variance);
white_noise = standard_noise * scaling_factor;
actual_variance = var(white_noise);
fprintf('Desired variance of input white noise: %.2f\n', desired_variance);
fprintf('Actual variance of input white noise: %.2f\n', actual_variance);

%% Import Simulink model
sim("LS1_1_NS.slx");

%% Prepare Data
u = squeeze(ans.input.Data)';
y = squeeze(ans.output.Data)';
t = ans.time.Data';

%% Calculate Phi
N = length(u);
phi = [-y(3:N-1)' -y(2:N-2)' -y(1:N-3)' u(3:N-1)' u(2:N-2)' u(1:N-3)'];

%% Check if phi is singular
if cond(phi)> 1e15
    fprintf(2,"phi is singular and can't be inverted.\n")
    pause(0.1)
    return
end   

%% Calculate Theta_hat
Y = y(4:N)';
theta_hat = inv(phi'*phi)*phi'*Y;

%% Calculate model output and its error
YP = phi*theta_hat;
error = Y-YP;
MSE = mse(error);

%% Plot output and estimated output
y_hat = phi*theta_hat;
t=1:length(y_hat);
figure;plot(t,y(:,1:length(y_hat)),t,y_hat);fontsize( 24 ,"points");legend('Measured','Simulated');
grid on;hold on;
title("LS parameter estimation with white noise input");xlabel('Sample Number');ylabel('Amplitude');

%% Recreate system model
sysCSD = tf(theta_hat(4:6)',[1 theta_hat(1:3)'],0.3)
sysCS = d2c(sysCSD)
%stepplot(sysCS)

%% Checking junk
% N = 3337;
% np = 6;
% mo = np/2;
% y= y';
% u = u';
% phis = [];
% for i = mo+1:N-mo
%     phis(i,:)=[-y(i-1:-1:i-mo,:)' u(i-1:-1:i-mo,:)'];
% end
% phi = phis(4:end,:);
% YY = y(mo+1:N-3);
% beta = inv(phi'*phi)*phi'*YY;
% YP = phi*beta;
% t = 1:length(YP);
% figure; plot(t,YY,t,YP);