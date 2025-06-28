%% Clear everything
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('PP1_0.m');
drawPlot = 1;

%% Pole placement without zero/pole cancellation for a zero outside unit circle
G_mirrored = ((5*((0.7*s)+1)*(s-0.8))/((((3*s)+1)^2)*((2*s)-1)));
Gz = c2d(G_mirrored, sampleTimeIntervals, 'zoh');

figure;
pzplot(Gz);
ee = findobj(gca,'type','line');
for i = 1:length(ee)
    set(ee(i),'markersize',24); 
    set(ee(i), 'linewidth',2) ; 
end
title('Pole-Zero map of discrete system');fontsize( 24 ,"points")

[A, B, C, D] = ssdata(Gz);

% Check controllability
Co = ctrb(A, B);
if rank(Co) == size(A, 1)
    disp('System is controllable');
else
    error('System is not controllable');
end

% Choose desired poles to improve settling time (example: faster real poles)
desired_poles = [0.4, 0.5, 0.8]; 

% Compute state feedback gain matrix K
K = place(A, B, desired_poles);

% Compute precompensator N to ensure unity DC gain
sys_cl_no_kr = ss(A - B*K, B, C, D,sampleTimeIntervals);
kr = 1 / dcgain(sys_cl_no_kr);

% Create closed-loop system with precompensator
B_cl = B * kr;
sys_cl = ss(A - B*K, B_cl, C, D,sampleTimeIntervals);

% Display settling times
S_original = stepinfo(ss(A, B, C, D,sampleTimeIntervals),1);
S_closed = stepinfo(sys_cl);
figure;
step(sys_cl);fontsize( 24 ,"points");
disp(['Closed-loop settling time (2%): ', num2str(S_closed.SettlingTime), ' seconds']);

% Plot for pulse train
if drawPlot
    [y4,~,yc] = lsim(sys_cl, u1, t);
    uc = -yc*K' + kr;

    figure;
    subplot(2,1,1);
    plot(t, u1, 'r--', t, y4, 'b-');fontsize( 24 ,"points");

    title(sprintf('Pole placement without cancellation'));
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Input', 'Output'); grid on;
    ylim([-0.5 1.5]); 
    xlim([0 500]);

    subplot(2,1,2);
    plot(t,uc,'b-');fontsize( 24 ,"points");
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Control signal'); grid on;
    ylim([-7 3]); 
    xlim([0 500]);
end