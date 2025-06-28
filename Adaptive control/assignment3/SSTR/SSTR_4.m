run('SSTR_0.m');

%%  Solve Diophantine equation
[F,G] = diophantine(A,C,1);

%%  Closed loop system
G_ol = minreal(G_discrete*tf(A,conv(B,F),Ts));

%%  closed loop system
G_cl = feedback(G_ol,1);
[B_true,A_true] = tfdata(G_cl,'v');

%% Simulation settings
na = length(A_true)-1; nb = length(B_true)-1;
N = 31;              
theta = zeros(nb + na, 1);  
gamma = 0.01;          

u = ones(1,N);
y = zeros(1,N);
y_ref = ones(1,N);
phi = zeros(nb + na, 1);

%% Simulation loop
for k = 4:N
    e = y_ref(k) - y(k);  
    phi = [-y(k-1); -y(k-2); -y(k-3); u(k-1); u(k-2); u(k-3)];
    u(k) = theta' * phi;
    y(k) = -A_true(2)*y(k-1) - A_true(3)*y(k-2) - A_true(4)*y(k-3) + ...
           B_true(2)*u(k-1) + B_true(3)*u(k-2) + B_true(4)*u(k-3);

    theta = theta - gamma * e * phi;
    THETA(:,k) = theta; 
end

%% Plot results
figure, hold on, grid on;
subplot(2,1,1) ;
plot((0:N-1)',y);
xlabel('Time (s)')
ylabel('Output')
fontsize( 24 ,"points");

subplot(2,1,2) ;
plot((0:N-1)',u);
xlabel('Time (s)')
ylabel('Control signal')
fontsize( 24 ,"points");

cumLoss = 1/N*cumsum((y-0).^2);
figure, hold on, grid on;
plot((0:N-1)',cumLoss);
xlabel('Time (s)')
ylabel('Cummulative loss')
fontsize( 24 ,"points");

figure, hold on, grid on;
plot((0:N-1)',THETA);
xlabel('Time (s)')
ylabel('\theta')
fontsize( 24 ,"points");