%% Indirect STR without zero cancellation
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('STR2_0.m');
drawPlot = 1;

%% Parameters
cancel = 0; % 0 no zero cancel, 1 all zero cancel
noise = 0; % if 1 white noise, 2 colored noise
distrubance = 0; % set to 1 for step disturbance
distrubance_fix = 0; % set to 1 to fix system
integral_fix = 0; % set 1 to limit u
vlimit = 3; % limit of u
lamda = 0.98;

if noise
    e = 0.05*randn(length(t),1);
    if noise == 2
        sys_dist = tf(1,[1 -1] , Gz.Ts) ;
        ynoise = lsim(sys_dist , e , t) ;
    else
        ynoise = e;
    end
else
    ynoise = zeros(length(t),1);
end

if distrubance
    e = 0.1*randn(length(t),1);
    for i =50:length(e)
        e(i)=0;
    end
    sys_dist = tf(1,[1 -1] , Gz.Ts) ;
    vdist = lsim(sys_dist , e , t) ;
    %vdist(ceil(length(vdist)/2)+1:end) = 1;
else
    vdist = zeros(length(t),1);
end

%%  plant input
uc = u1 ;

% plant model
[B , A] = tfdata(Gz) ;
A = cell2mat(A) ;
B = cell2mat(B) ; B = B(2:end) ;

na = numel(A)-1;
nb = numel(B)-1;
d0 = na-nb;

% reference model
Am = [1 -1.7 0.92 -0.16];
if cancel
    sys_ref_dis = tf([0 sum(Am) 0],Am,Gz.Ts) ;
else
    beta = sum(Am)/sum(B);
    sys_ref_dis = tf(beta*B,Am,Gz.Ts) ;
end

[Bm , Am] = tfdata(sys_ref_dis) ;
Am = cell2mat(Am) ;
Bm = cell2mat(Bm) ; Bm = Bm(2:end) ;
y_ref = lsim(sys_ref_dis , uc , t);

if cancel
    A0 = 0;
    Ac = poly([A0 roots(B)']) ;
else
    if distrubance_fix
        A0 = [0 0 0 0] ;
    else
        A0 = [0 0] ;
        %A0 = q^2 -0,5 q +0.06
    end
    
    Ac = poly([A0 roots(Am)']) ;
end

%% indirect str for pole placement
N = numel(t) ;
Nv = na+nb+1;

teta = zeros(Nv , 1) ;
for i =1:Nv
    tetas(:,i) = teta;
end

P = 1e2*eye(Nv) ;

y = randn(Nv , 1);
u = randn(Nv , 1);

%main loop
for i = Nv+1:N

    y(i) = -A(2:end)*y(i-1:-1:i-na)+B*(u(i-d0:-1:i-na)+vdist(i-(numel(A)-numel(B)):-1:i-(numel(A)-1))+ynoise(i-(numel(A)-numel(B)):-1:i-(numel(A)-1))) ;
    Y = [-y(i-1) , -y(i-2), -y(i-3)];
    U = [u(i-1)+vdist(i-1), u(i-2)+vdist(i-2), u(i-3)+vdist(i-3)] ;

    [teta , P] = RLS(Y ,U , y(i) , teta , P , Nv,lamda) ;
    tetas(:,i)=teta;

    Aes = [1 teta(1:Nv/2)'] ;
    Bes = teta(Nv/2+1:end)' ;

    if cancel
        S = zeros(1,numel(Am)-1);
        R = zeros(1,numel(Am)-1);
        R(1) = 1;
        for j = 1:numel(Am)-1
            S(j) = (Am(j+1)-Aes(j+1))/Bes(1);
        end
        for j = 2:3
            R(j) = Bes(j)/Bes(1);
        end
        T = [0,Bm(2)/Bes(1),0] ;
    else
        if distrubance_fix
            [Rbar , S] = Diophantine(conv(Aes,[1 -1]) , Bes , Ac)  ;
            R = conv(Rbar , [1 -1]);
            AcBm = conv(Ac , Bm) ;
            AmB = conv(Am , Bes) ;
            T = [0,0,sum(AcBm) / sum(AmB)] ;
        else
            [R , S] = Diophantine(Aes , Bes , Ac)  ;
            AcBm = conv(Ac , Bm) ;
            AmB = conv(Am , Bes) ;
            T = [(sum(AcBm) / sum(AmB))*poly(A0)] ;
        end

    end

    u(i) = (-R(2:end)*u(i-1:-1:i-(numel(R)-1))+T*uc(i-(numel(R)-numel(T)):-1:i-(numel(R)-1))-S*y(i-(numel(R)-numel(S)):-1:i-(numel(R)-1)))/R(1) ;
    if integral_fix && i>500
        if u(i)<-vlimit
            u(i)=-vlimit;
        elseif u(i) > vlimit
            u(i) = vlimit;
        end
    end
end
%% plot results

if drawPlot
    figure();
    subplot(2,1,1) ;
    plot(t,y_ref,t,y , 'LineWidth' , 2) ;
    xlabel('Time (sec)') ;
    ylabel('Amplitude') ;
    title('Reference response vs Indirect STR') ;
    grid on;
    legend('y_ref' , 'y_actual');
    fontsize( 24 ,"points");
    ylim([-0.1 1.2]);
    xlim([0 500]);

    subplot(2,1,2) ;
    plot(t , u, 'LineWidth' , 2) ;
    xlabel('Time (sec)') ;
    ylabel('Effort control') ;
    title('Effor control in Indirect STR') ;
    grid on;
    legend('U') ;
    fontsize( 24 ,"points");
    ylim([-2.5 2.5]);
    xlim([0 500]);

    %% a
    figure;
    subplot(2,1,1)
    hold all;
    for i=1:Nv/2
        plot(t,tetas(i,:),'linewidth',2);
        Leg1{i}=['a_' num2str(i)];
    end
    set(gcf,'color','w');
    grid on;
    xlabel('Time (s)');
    ylabel('Parameters');
    legend(Leg1);
    fontsize( 24 ,"points");
    xlim([0 500]);

    %% b
    subplot(2,1,2);
    hold all;
    for i=Nv/2+1:Nv
        plot(t,tetas(i,:),'linewidth',2);
        Leg2{i-Nv/2}=['b_' num2str(i-Nv/2)];
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Parameters');
    legend(Leg2);
    fontsize( 24 ,"points");
    xlim([0 500]);
end