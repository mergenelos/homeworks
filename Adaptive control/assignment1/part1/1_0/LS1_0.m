%% Clear everything
clear;clc;close all

%% Evaluate transfer function
s = tf('s');
G = (0.72*s^2 + 2.4*s + 2) / (6*s^3 + 11*s^2 + 6*s + 1);

%% Plot step response
figure;stepplot(G);grid on;fontsize( 24 ,"points");

%% Get settling time of step response
s = stepinfo(G);

%% Calculate the time interval for sampling
sampleTimeIntervals = s.SettlingTime/50;
fprintf("Settling time:%f\nSample time:%.2fs\n", ...
    s.SettlingTime,round(sampleTimeIntervals,1));
fprintf('----------------------------------------------------\n');

%% Bode plot of G
figure;bodeplot(G);grid on;fontsize( 24 ,"points");

%% Calculate the frequency response
w = logspace(-1, 2, 1000); % Frequency vector
[mag, phase] = bode(G, w); % Or use freqs(sys,w)

%% Find the 3dB frequency
w = logspace(-1, 2, 1000);
[mag, phase] = freqresp(G, w);
[max_mag, max_index] = max(abs(mag));
max_freq = w(max_index);
fprintf('Maximum magnitude: %.4f\n', max_mag);
fprintf(['Frequency for sine input needs to be less than:''%.2f rad/s\n'], max_freq);
fprintf('----------------------------------------------------\n');

%% Define the sine wave input
t = 0:0.01:100;
u = 2*sin(0.1*t);

[y,t] = lsim(G, u, t);

figure;subplot(2,1,1);plot(t, u);fontsize( 24 ,"points");
title('Input Signal (2*Sine(0.1*t) Wave)');
xlabel('Time (s)');ylabel('Amplitude');
subplot(2,1,2);plot(t, y);fontsize( 24 ,"points");
title('Output Signal');xlabel('Time (s)');ylabel('Amplitude');
fprintf(['The sine rsponse for 2*sine(0.1*x) has min: %0.2f max: %0.2f peak2peak: %0.2f\n'],min(y),max(y),peak2peak(y));
