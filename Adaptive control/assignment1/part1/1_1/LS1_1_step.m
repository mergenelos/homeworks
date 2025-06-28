%% Clear everything
clear;close all;clc;

%% Import Simulink model
sim("LS1_1_Stp.slx")

%% Prepare Data
u = ans.input.Data';
y = ans.output.Data';
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

%% Since phi is singular for step response, no need to continue...