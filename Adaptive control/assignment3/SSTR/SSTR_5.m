run('SSTR_0.m');

%%  Solve Diophantine equation for d = 2 
d = 2;  % d0=1
[F, G] = diophantine(A, C, d);

%%  Define the MA controller 
MA_controller = tf(conv(F,A), 1, Ts);

%%  Define the open loop system
G_ol = minreal(G_discrete * MA_controller);

%%  Closed-loop system
G_cl = feedback(G_ol, 1);

%%  Plot
T = (0:Ts:30*Ts)';
[y,t] = impulse(G_cl,T);
figure, hold on, grid on;
subplot(2,1,1) ;
plot(t,y);
xlabel('Time (s)')
ylabel('Output')
fontsize( 24 ,"points");

[u,t] = impulse(1/G_cl,T);
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