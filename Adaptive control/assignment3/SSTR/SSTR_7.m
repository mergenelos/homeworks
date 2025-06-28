run('SSTR_0.m');

%% Solve Diophantine equation for d = 2 (MA controller of order 2)
d = 2;  % d0=1
[F, G] = diophantine(A, C, d);

%%  Define the MA controller 
MA_controller = tf(conv(F,A), 1, Ts);

%% Open loop
G_ol = minreal(G_discrete * MA_controller);

%%  Closed loop
G_cl = feedback(G_ol, 1);
A_true = [-2.5373   -0.1906    0.0497    0.1744    1.0341];
B_true = [-2.5373   -0.1906    0.0497    0.1744    0.0341];

%% Simulation settings
na = length(A_true)-1; nb = length(B_true)-1;
N = 31;             
theta = zeros(nb + na, 1);  
gamma = 0.01;          

u = ones(1,N);
y = zeros(1,N);
y_ref = ones(1,N);
phi = zeros(nb + na, 1);

for k = 5:N
    e = y_ref(k) - y(k);
    phi = [-y(k-1); -y(k-2); -y(k-3); -y(k-4); ...
        u(k-1); u(k-2); u(k-3); u(k-4)];
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