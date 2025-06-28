%% Setting up
clc, clear, close all

%% Required calculations
s = tf('s');
G = 3*(0.4*s+1)*(s+0.8)/((3*s+1)^2*(s+1));
% Settling time
step_info = stepinfo(G);
settling_time = step_info.SettlingTime;
% Sample time
Ts = floor(settling_time)*0.1;

%%  colored noise coefficients
C = [1,0.6,0.4];

%% Original transfer function
G_s = 3*(0.4*s+1)*(s+0.8)/((3*s+1)^2*(s-1));

%% Discrete transfer function
G_discrete = c2d(G_s,Ts,'zoh');
[B,A] = tfdata(G_discrete,'v');
B(1) = [];