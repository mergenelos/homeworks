%% Clear everything
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('PP1_0.m');
drawPlot = 1;

%% Pole placement without zero/pole cancellation 

% Convert to state-space representation
[A, B, C, D] = ssdata(Gz);

% Check controllability
Co = ctrb(A, B);
if rank(Co) == size(A, 1)
    disp('System is controllable');
else
    error('System is not controllable');
end

% Choose desired poles to improve settling time
desired_poles = [0.4, 0.5, 0.8]; 

K = place(A, B, desired_poles);

sys_cl_no_kr = ss(A - B*K, B, C, D,sampleTimeIntervals);
kr = 1 / dcgain(sys_cl_no_kr);

% Create closed-loop system with precompensator
B_cl = B * kr;
sys_cl = ss(A - B*K, B_cl, C, D,sampleTimeIntervals);

% Display settling times
S_original = stepinfo(ss(A, B, C, D,sampleTimeIntervals),1);
S_closed = stepinfo(sys_cl);
step(sys_cl, 'b');fontsize( 24 ,"points");
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
    ylim([-0.1 1.1]); 
    xlim([0 500]);

    subplot(2,1,2);
    plot(t,uc,'b-');fontsize( 24 ,"points");
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Control signal'); grid on;
    ylim([-3 7]); 
    xlim([0 500]); 
end
