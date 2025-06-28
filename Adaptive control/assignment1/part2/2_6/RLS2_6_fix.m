%% Clear everything
clear;close all;clc;

%% Generate white noise with variance of 2
desired_variance = 2;
signal_length = 3334; 
standard_noise = randn(signal_length, 1);
current_variance = var(standard_noise);
scaling_factor = sqrt(desired_variance / current_variance);
white_noise = standard_noise * scaling_factor;
actual_variance = var(white_noise);
fprintf('Desired variance of input white noise: %.2f\n', desired_variance);
fprintf('Actual variance of input white noise: %.2f\n', actual_variance);

%% Generate output noise 
desired_output_variance = 1; 
output_noise = randn(signal_length, 1);
current_output_variance = var(output_noise);
output_scaling_factor = sqrt(desired_output_variance / current_output_variance);
output_noise = output_noise * output_scaling_factor;
output_noise = normalize(output_noise, 'range', [-0.1 0.1]);
actual_output_variance = var(output_noise);
%figure;plot(output_noise);grid on;

%% Gradually change parameters
a2 = ones(signal_length);a2=a2(1:end,1:2)
a3 = ones(signal_length);a3=a3(1:end,1:2);a3(:,2)=1/6*a3(1:end,2);

% Calculate the step size for a linear change
%final_value-initial_value/sample_period
step_size_a2 = (0.3 - 1) / 50;
step_size_a3 = ((0.3*(1/6)) - 1/6) / 50;

for n=1:length(a2(1:end,1))
    a2(n,1) = 0.3*n;
    a3(n,1) = 0.3*n;
end

% change a2 form 50 to 101
for n=50:101
    a2(n,2) = a2(n,2)+step_size_a2*(n-50);
end
a2(101:end,2) = a2(101,2);

% change a3 form 200 to 251
for n=200:251
    a3(n,2) = a3(n,2)+step_size_a3*(n-200);
end
a3(251:end,2) = a3(251,2);

%% Import Simulink model
sim("RLS2_6_NS.slx")

%% Prepare Data
u = squeeze(ans.input.Data)';
y = squeeze(ans.output.Data)';
yActual= squeeze(ans.actualOutput.Data)';
t = ans.time.Data';

lamda = inv(0.95);

Plot=[1 5 6];
na =3; nb=2;d=0;Ts=0.3;
N=max(na+1,nb+d+1);
for L=1:N
        P{L}=eye(na+nb+1)*10^(3);
end
theta_hat(:,1:N)=zeros(na+nb+1,N);
epslon(1:N)=0;
y1(1:N)=y(1:N);
for i=N:length(y)
    for j=1:na
        if i-j <=0
            phiT(i,j)=0;
        else
            phiT(i,j)=[-y(i-j)];
        end
    end
    for j=0:nb
        if i-j-d <= 0
            phiT(i,j+1+na)=0;
        else
            phiT(i,j+1+na)=[u(i-j-d)];
        end
    end
                                        %1x6 x 6x6 x 6x1 = 1x1
    K{i}=P{i-1}*phiT(i,:)'*inv(lamda+phiT(i,:)*P{i-1}*phiT(i,:)');
    epslon(i)=y(i)-phiT(i,:)*theta_hat(:,i-1);
    theta_hat(:,i)=theta_hat(:,i-1)+K{i}*epslon(i);
    P{i}=(eye(length(K{i}*phiT(i,:)))-K{i}*phiT(i,:))*P{i-1}/lamda;
    P{i}=(P{i}+P{i}')/2;
end

Theta_hat=theta_hat(:,end);
Gz=tf([Theta_hat(na+1:end)'],[1,Theta_hat(1:na)'],Ts)

for l=1:length(phiT(:,1))
    y1(l)=phiT(l,:)*theta_hat(:,l);
end
Y=size(y);
Y1=size(y1);
if Y(1) ~= Y1(1)
        y1=y1';
end

    figure(Plot(2));
    set(gcf,'color','w')
    hold all;
    for k=1:na+nb+1
        plot((0:length(y1)-1)*Ts,theta_hat(k,:),'linewidth',2);fontsize( 24 ,"points");
    end
    grid on;
    for m=1:na
        Ylabel{m}=['a_' num2str(m) ', '];
        Leg{m}=['a_' num2str(m) ];
    end
    for n=1:nb+1
        if n<nb+1
            Ylabel{n+na}=['b_' num2str(n-1) ', '];
            Leg{n+na}=['b_' num2str(n-1) ];
        else
            Ylabel{n+na}=['b_' num2str(n-1)];
            Leg{n+na}=['b_' num2str(n-1) ];
        end
    end
    xlabel('t(s)');
    Ylabel=cell2mat(Ylabel);
    ylabel(Ylabel);
    legend(Leg)
    title('Parameter estimation with time')
    
    figure(Plot(3));
    subplot(3,1,1:2)
    set(gcf,'color','w')
    plot((0:length(y1)-1)*Ts,y,(0:length(y1)-1)*Ts,y1,'-o','linewidth',2);fontsize( 24 ,"points");
    grid on;
    ylabel('y, y_e_s_t');
    legend('y','y_e_s_t')
    subplot(3,1,3)
    plot((0:length(y1)-1)*Ts,abs(y-y1));fontsize( 24 ,"points");
    grid on;
    xlabel('t(s)');
    ylabel('y-y_e_s_t');
    legend('error')


%% Calculate model output and its error
error = yActual-y1;
MSE = mse(error);

%% 3 st order estimation

sysCS = d2c(Gz)


%% Plotting data
subplot(3,1,1);
plot(t,u,t,y);fontsize( 24 ,"points");
legend('input','output');grid on;hold on;
title("Input & Output");;

subplot(3,1,2);
plot(t,y,t,y1);fontsize( 24 ,"points");
legend('output','estimated output');grid on;hold on;
title("Output & Estimated output");;

subplot(3,1,3);
plot(t,error);fontsize( 24 ,"points");
legend('error');grid on;hold on;
title("Error over iterations");;























%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Import Simulink model
% sim("RLS2_5_NS.slx")
% 
% %% Prepare Data
% u = squeeze(ans.input.Data);
% y = squeeze(ans.output.Data);
% t = ans.time.Data';
% 
% %%
% np = 6; model = np/2;
% lamda=0.5;
% 
% N=3334;
% bn=zeros(np,N);y_predicted=[];
% phi=zeros(np,1);
% P=10e6*eye;
% for i=model+1:N
%     phi(:,i)=[y(i-1:-1:i-model)' u(i-1:-1:i-model)']';
%     theta(:,i) = inv(lamda)*P*phi(:,i);
%     K=inv(1+phi(:,i)'*theta(:,i))*theta(:,i);
%     P=(inv(lamda)*P)-theta(:,i)'*K;
%     y_predicted(i,1)=phi(:,i)'*bn(:,i-1);
%     error(i,1)=y(i,1)-y_predicted(i,1);
%     bn(:,i)=bn(:,i-1)+K*error(i,1);
% end
% theta_hat = theta(:,end);
% 
% 
% 
% 
% 
%        figure; plot(t,theta_hat(2,:),'linewidth',2);
% 
% 
% %% Plotting data
% subplot(3,1,1);
% plot(t,u,t,y);
% legend('input','output');grid on;hold on;
% title("Input & Output");;
% 
% subplot(3,1,2);
% plot(t,y(1:N),t,y_predicted(1:N));
% legend('output','estimated output');grid on;hold on;
% title("Output & Estimated output");;
% 
% subplot(3,1,3);
% plot(t,error);
% legend('error');grid on;hold on;
% title("Error over iterations");;
% 
% %% Calculate model output and its error
% MSE = mse(error);
% 
% sysCSD = tf(theta_hat(4:6)',[1 theta_hat(1:3)'],0.3)
% sysCS = d2c(sysCSD)