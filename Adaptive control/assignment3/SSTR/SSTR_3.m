run('SSTR_0.m');

%% Identification parameters
na = 2; nb = 2; 
theta = zeros(na+nb,1); 
P = 1000*eye(na+nb); 
lambda = 0.98; 

%% Input and output data
N = 50;
u = randn(1,N); 
y = zeros(1,N); 
y_est = zeros(1,N); 

for k = max(na,nb)+1:N
    y(k) = -A(2)*y(k-1) - A(3)*y(k-2) + ...
            B(2)*u(k-1) + B(3)*u(k-2); 
end

%% RLS
for k = max(na,nb)+1:N
    phi = [-y(k-1); -y(k-2); u(k-1); u(k-2)];
    y_hat = theta'*phi;
    e = y(k) - y_hat;
    K = (P*phi)/(lambda + phi'*P*phi);
    theta = theta + K*e;
    P = (P - K*phi'*P)/lambda;

    y_est(k) = y_hat;
    THETA(:,k) = theta; 
end

%% Recover A and B from ?
A_id = [1; theta(1:na)];
B_id = theta(na+1:end);

%% Adaptive controller design
[F,G] = diophantine(A_id, C, 1);

%% Open-loop and closed-loop systems
G_z_hat = tf(B_id', A_id', Ts);
G_ol = minreal(G_z_hat * tf(A_id', conv(B_id', F), Ts));
G_cl = feedback(G_ol, 1);

%%  plot
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

figure, hold on, grid on;
plot((0:N-1)',THETA);
xlabel('Time (s)')
ylabel('\theta')
fontsize( 24 ,"points");
