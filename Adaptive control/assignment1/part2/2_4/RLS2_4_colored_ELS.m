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
figure;plot(output_noise);grid on;

%% Import Simulink model
sim("RLS2_4_clrd.slx")

%% Prepare Data
u = squeeze(ans.input.Data)';
y = squeeze(ans.output.Data)';
yActual = squeeze(ans.outputWithoutNoise.Data)';
t = ans.time.Data';

Plot=[1 1:3];
nc=2;na=3; nb=2;d=0;Ts=0.3;
N=max(na+1,nb+d+1);
for L=1:N
        P{L}=eye(na+nb+nc+1)*10^(3);
end
theta_hat(:,1:N)=zeros(na+nb+nc+1,N);
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
        for j=1:nc
                if i-j <=0
                        phiT(i,j+na+nb+1)=0;
                else
                        phiT(i,j+na+nb+1)=[epslon(i-j)];
                end
        end
        K{i}=P{i-1}*phiT(i,:)'*inv(1+phiT(i,:)*P{i-1}*phiT(i,:)');
        epslon(i)=y(i)-phiT(i,:)*theta_hat(:,i-1);
        theta_hat(:,i)=theta_hat(:,i-1)+K{i}*epslon(i);
        P{i}=(eye(length(K{i}*phiT(i,:)))-K{i}*phiT(i,:))*P{i-1};
        P{i}=(P{i}+P{i}')/2;
end

Theta_hat=theta_hat(:,end);
Guz=tf([Theta_hat(na+1:end-nc)'],[1,Theta_hat(1:na)'],Ts);
Gez=tf([1, Theta_hat(na+nb+2:end)'],[1,Theta_hat(1:na)'],Ts);

for l=1:length(phiT(:,1))
        y1(l)=phiT(l,:)*theta_hat(:,l);
end
Y=size(y);
Y1=size(y1);
if Y(1) ~= Y1(1)
        y1=y1';
end
%% plotting

        figure(Plot(2));
        set(gcf,'color','w')
        hold all;
        for k=1:na+nb+1
                if k <= na
                        H='-';
                else
                        H='--';
                end
                plot((0:length(y1)-1)*Ts,theta_hat(k,:),H,'linewidth',2);fontsize( 24 ,"points");
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
        xlabel('Time (s)');
        Ylabel=cell2mat(Ylabel);
        ylabel(Ylabel);
        legend(Leg)
        title('System parameter estimation with time')
        
        
        
        figure(Plot(3));
        set(gcf,'color','w')
        hold all;
        for k=[1:na, na+nb+2:na+nb+nc+1]
                if k <= na
                        H='-';
                else
                        H='--';
                end
                plot((0:length(y1)-1)*Ts,theta_hat(k,:),H,'linewidth',2);fontsize( 24 ,"points");
        end
        grid on;
        for m=1:na
                Ylabel2{m}=['a_' num2str(m) ', '];
                Leg2{m}=['a_' num2str(m) ];
        end
        for c=1:nc
                if c<nc
                        Ylabel2{c+na}=['c_' num2str(c) ', '];
                        Leg2{c+na}=['c_' num2str(c) ];
                else
                        Ylabel2{c+na}=['c_' num2str(c)];
                        Leg2{c+na}=['c_' num2str(c) ];
                end
        end
        xlabel('Time (s)');
        Ylabel2=cell2mat(Ylabel2);
        ylabel(Ylabel2);
        legend(Leg2)
        title('System noise parameter estimation with time')
        
        figure(Plot(4));
        subplot(3,1,1:2)
        set(gcf,'color','w')
        plot((0:length(y1)-1)*Ts,y,(0:length(y1)-1)*Ts,y1,'-o','linewidth',2);fontsize( 24 ,"points");
        grid on;
        ylabel('y, y_e_s_t');
        legend('y','y_e_s_t');
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
Guz
sysCS = d2c(Guz)






