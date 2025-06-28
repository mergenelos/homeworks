%% Clear everything
clear;clc;close all
global drawPlot;
if drawPlot ~= -1
    drawPlot = 1;
else 
    drawPlot = 0;
end

%% Evaluate transfer function
%tf([3.5,7.8,4],[18,3,-4,-1])
s = tf('s');
G_Original = ((5*((0.7*s)+1)*(s+0.8))/((((3*s)+1)^2)*((2*s)-1)));
if drawPlot
    figure;
    step(G_Original, 'b');
    legend('Step response');
    title('Step Response of Original System');
    fontsize( 24 ,"points");
end

%% Will use stable transfer function to evaluate 
G = ((5*((0.7*s)+1)*(s+0.8))/((((3*s)+1)^2)*((2*s)+1)));

%% Get settling time of step response
sys_info = stepinfo(G);

%% Calculate the time interval for sampling
Ts = sys_info.SettlingTime;
sampleTimeIntervals = round(Ts/50,1);
fprintf("Settling time:%f\nSample time:%.2fs\n", ...
   Ts,sampleTimeIntervals);
fprintf('----------------------------------------------------\n');

%% Discrete transfer function of original system
Gz = c2d(G_Original, sampleTimeIntervals, 'zoh');

%% Bode plot of G
if drawPlot
    figure;bodeplot(G);grid on;fontsize( 24 ,"points");
end

%% Calculate the frequency response
w = logspace(-1, 2, 1000); 
[mag, phase] = bode(G, w); 

%% Find the 3dB frequency
w = logspace(-1, 2, 1000);
[mag, phase] = freqresp(G, w);
[max_mag, max_index] = max(abs(mag));
max_freq = w(max_index);
fprintf('Maximum magnitude in 3db: %.4f\n', max_mag);
fprintf(['Frequency for sine input needs to be less than: %.2f rad/s\n'], max_freq);
fprintf('----------------------------------------------------\n');

%% Discrete transfer function of stable transfer function
G_discrete = c2d(G, sampleTimeIntervals, 'zoh');
% Compare step responses
if drawPlot
    figure;
    step(G, 'b');
    hold on;
    step(G_discrete, 'r');
    legend('Continuous', 'Discrete');
    title('Step Response Comparison');
    fontsize( 24 ,"points");
end

%% Required minimum pulse train period and frequency
k = 5; % 4x settling time
Tp_min = k * Ts; % Minimum recommended period between pulses

fprintf('Minimum Recommended Pulse Period (Tp): %.2f seconds\n', Tp_min);

%Simulation Time
Tsim = 20 * Tp_min; 
t = 0:sampleTimeIntervals:Tsim;

%Pulse Train 1 (Respecting Settling Time)
u1 = gensig('square' ,Tp_min , Tsim , Gz.Ts); 
u2 = gensig('square' ,Tp_min/k , Tsim , Gz.Ts); 

y1 = lsim(G_discrete, u1, t);
y2 = lsim(G_discrete, u2, t);

if drawPlot
    figure;
    subplot(2,1,1);
    plot(t, u1, 'r-', t, y1, 'b-');fontsize( 24 ,"points");
    title(sprintf('Response with fpulse Allows Settling '));
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Input Pulse', 'System Output'); grid on;
    ylim([-0.1 4.5]); 
    
    subplot(2,1,2);
    plot(t, u2, 'r-', t, y2, 'b-');fontsize( 24 ,"points");
    title(sprintf('Response with fpulse Too Fast'));
    xlabel('Time (s)'); ylabel('Amplitude'); legend('Input Pulse', 'System Output'); grid on;
    ylim([-0.1 4.5]);  
end
fprintf('----------------------------------------------------\n');




