%% Clear everything
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('PP1_0.m');
drawPlot = 1;

%% Pole placement with zero/pole cancellation

% Convert to state-space representation
[A, B, C, D] = ssdata(Gz);

% Check controllability
Co = ctrb(A, B);
if rank(Co) == size(A, 1)
    disp('System is controllable');
else
    error('System is not controllable');
end

% Get system zeros and select desired poles
desired_poles = [roots(Gz.num{1})' 0.8];  % Cancel zeros with poles and add fast pole

% Compute state feedback gain
K = place(A, B, desired_poles);

% Create closed-loop system with precompensator
sys_ol = ss(A, B, C, D,sampleTimeIntervals);
sys_cl = ss(A-B*K, B, C, D,sampleTimeIntervals);
sys_cl = minreal(sys_cl);
% Adjust DC gain for unity steady-state
kr = 1/dcgain(sys_cl);
sys_cl = ss(A-B*K, B*kr, C, D,sampleTimeIntervals);
step(sys_cl, 'b');fontsize( 24 ,"points");

% Display settling times
%S_ol = stepinfo(sys_ol,1);
S_cl = stepinfo(sys_cl,1);
%disp(['Original settling time: ', num2str(S_ol.SettlingTime), ' s']);
disp(['Closed-loop settling time (2%): ', num2str(S_cl.SettlingTime), ' s']);

% Plot for pulse train
if drawPlot
    [y5,~,yc] = lsim(sys_cl, u1, t);
    uc = -yc*K' + kr;

    figure;
    subplot(2,1,1);
    plot(t, u1, 'r--', t, y5, 'b-');fontsize( 24 ,"points");

    title(sprintf('Pole placement with pole/zero cancellation'));
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Input', 'Output'); grid on;
    ylim([-0.1 1.1]); 
    xlim([0 500]);

    subplot(2,1,2);
    plot(t,uc,'b-');fontsize( 24 ,"points");
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Control signal'); grid on;
    ylim([-1 2.5]); 
    xlim([0 500]);
end
