%% Direct STR with zero cancellation
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('STR2_0.m');
drawPlot = 1;

%% Parameters
cancel = 1; % 0 no zero cancel, 1 all zero cancel

%% generate Data
uc = u1 ;
lamda = 1;

% plant model
system_dig = Gz;
[B  ,A] = tfdata(system_dig) ;
A = cell2mat(A) ;
B = cell2mat(B) ; B = B(2:end) ;

% reference model
Am = [1 -1.7 0.92 -0.16];
if cancel
    sys_ref_dig = tf([0 sum(Am) 0],Am,Gz.Ts) ;
    A0 = [1];
else
    beta = sum(Am)/sum(B);
    sys_ref_dig = tf(beta*B,Am,Gz.Ts);
    A0 = [1];
end
[Bm  ,Am] = tfdata(sys_ref_dig);
Am = cell2mat(Am) ;
Bm = cell2mat(Bm) ; Bm = Bm(2:end) ;
y_ref = lsim(sys_ref_dig , uc , t) ;

%% initial parameters
n = numel(A)-1 ;
m = numel(B)-1 ;
d0 = n-m ;
A0Am = conv(A0 , Am) ;
Na0am = numel(A0Am)-1 ;

L = Na0am-d0 ;
Nv = 3*(L+1) ;

%teta = zeros(Nv , 1) ;
%teta(:,1) = [0.104;-0.0431;0.008;0.442;-0.167;-0.194;-0.050;0.086;0.0272];
teta = 1*randn(Nv,1) ;

P = 1e6*eye(Nv) ;
%P = 1e2*eye(Nv) ;
%P = 1e12*eye(Nv) ;

u  = randn(Nv , 1) ;  % initial effort control
y  = randn(Nv , 1) ;  % initial output
uf = randn(Nv , 1) ;  % initial filtered effort control
yf = randn(Nv , 1) ;  % initial filtered output
ucf= randn(Nv , 1) ;  % initial filtered command signal

%% main loop
N = numel(t) ;

for i =1:Nv
    tetas(:,i) = teta;
end

for i = Nv+1:N
    y(i) = -A(2:end)*y(i-1:-1:i-n)+B*(u(i-d0:-1:i-n)) ;
    U = uf(i-d0:-1:i-L-d0) ;
    V = [yf(i-d0:-1:i-L-d0)' , -ucf(i-d0:-1:i-L-d0)']' ;
    Y = y(i)-y_ref(i) ;

    [teta , P] = RLS(U , V , Y , teta , P , Nv,lamda) ;
    tetas(:,i)=teta;

    Rst = teta(1:Nv/3)' ;
    Sst = teta(Nv/3+1:2*Nv/3)' ;
    Tst = teta(2*Nv/3+1:Nv)' ;

    u(i) = (-Rst(2:end)*u(i-1:-1:i-L)+Tst*uc(i:-1:i-L)-Sst*y(i:-1:i-L))/Rst(1) ;
    uf(i) = -A0Am(2:end)*uf(i-1:-1:i-Na0am)+u(i) ;
    yf(i) = -A0Am(2:end)*yf(i-1:-1:i-Na0am)+y(i) ;
    ucf(i) = -A0Am(2:end)*ucf(i-1:-1:i-Na0am)+uc(i) ;
end

%% plot results
if drawPlot
    figure()
    plot(t , uc ,t , y_ref , 'LineWidth' , 2) ;
    xlabel('Time (sec)') ;
    ylabel('Amplitude') ;
    title('reference response') ;
    grid on;
    legend('Uc','Y_ref');
    fontsize( 24 ,"points");
    ylim([-0.1 1.2]);
    xlim([0 500]);

    figure() ;
    subplot(2,1,1) ;
    plot(t , y_ref , t , y , 'LineWidth' , 2) ;
    xlabel('Time (sec)') ;
    ylabel('Amplitude') ;
    title('Reference response Vs Actual response') ;
    grid on
    legend('Y_ref' , 'Y_actual') ;
    fontsize( 24 ,"points");
    ylim([-0.1 1.2]);
    xlim([0 500]);

    subplot(2,1,2) ;
    plot(t , u , 'LineWidth' , 2) ;
    xlabel('Time(s)') ;
    ylabel('Amplitude') ;
    title('Effort control in Direct STR') ;
    grid on
    legend('U') ;
    fontsize( 24 ,"points");
    ylim([-10 10]);
    xlim([0 500]);

    %% R
    figure;
    subplot(3,1,1);
    hold all;
    for i=1:Nv/3
        plot(t,tetas(i,:),'linewidth',2);
        Leg1{i}=['r_' num2str(i)];
    end
    set(gcf,'color','w');
    grid on;
    xlabel('Time (s)');
    ylabel('Controller Parameters');
    legend(Leg1);
    fontsize( 24 ,"points");
    xlim([0 500]);

    %% S
    subplot(3,1,2);
    hold all;
    for i=Nv/3+1:2*Nv/3
        plot(t,tetas(i,:),'linewidth',2);
        Leg2{i-Nv/3}=['s_' num2str(i-Nv/3)];
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Controller Parameters');
    legend(Leg2);
    fontsize( 24 ,"points");
    xlim([0 500]);

    %% T
    subplot(3,1,3);
    hold all;
    for i=2*Nv/3+1:Nv
        plot(t,tetas(i,:),'linewidth',2);
        Leg3{i-2*Nv/3}=['t_' num2str(i-2*Nv/3)];
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Controller Parameters');
    legend(Leg3);
    fontsize( 24 ,"points");
    xlim([0 500]);
end