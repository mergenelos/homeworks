run('SSTR_0.m');

%% Transfer function
G_s = 3*(0.4*s+1)*(s+0.8)/((3*s+1)^2*(s-1))*tf(1,[1,0,0]);

%% Discrete transfer function
G_z = c2d(G_s,Ts,'zoh');
[B,A] = tfdata(G_z,'v');
B(1) = [];

%% Solve Diophantine equation
[F,G] = diophantine(A,C,1);

%%  Closed loop system
G_ol = minreal(G_z*tf(A,conv(B,F),Ts));

%%  closed loop system
G_cl = feedback(G_ol,1);

%% PLot
T = (0:Ts:30*Ts)';
[y,t] = impulse(G_cl,T);
figure, hold on, grid on;
subplot(2,1,1) ;
plot(t,y);
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