run('SSTR_0.m');

%% Diophantine equation
[F,G] = diophantine(A,C,1);

%% Open loop system
G_ol = minreal(G_discrete*tf(A,conv(B,F),Ts));

%% Closed loop system
G_cl = feedback(G_ol,1);

%% Plot
T = (0:Ts:30*Ts)';
[y,t] = impulse(G_cl,T);
figure, hold on, grid on;
subplot(2,1,1) ;
plot(t,y);
grid on
xlabel('Time (s)')
ylabel('Output')
fontsize( 24 ,"points");

[u,t] = impulse(G_ol,T);
subplot(2,1,2) ;
plot(t,u);
xlabel('Time (s)')
ylabel('Control signal')
fontsize( 24 ,"points");

cumLoss = 1/length(t)*cumsum((y-0).^2);
figure, hold on, grid on;
plot(t,cumLoss);
xlabel('Time (s)')
ylabel('Cummulative loss')
fontsize( 24 ,"points");