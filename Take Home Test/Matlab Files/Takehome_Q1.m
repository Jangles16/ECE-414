%% Q1 - Root Locus Design

%% Get the Plant
clc; clear all;

G = ece414planttf(6,10,.45);

%% Plot root locus for all Alpha
% will use to see where zeros can be placed

figure(1); clf;
for i = 0:.1:.9
    G = ece414planttf(6,10,i);
    rlocus(G)
    grid on
    axis([-20 0 -25 25]);
    hold on
end
legend(['alpha = ' num2str(0)],['alpha = ' num2str(.1)],['alpha = ' num2str(0.2)],['alpha = ' num2str(0.3)], ['alpha = ' num2str(0.4)], ['alpha = ' num2str(0.5)], ['alpha = ' num2str(0.6)], ['alpha = ' num2str(0.7)], ['alpha = ' num2str(0.8)], ['alpha = ' num2str(0.9)]);

%% Set up PI controller
% Need at least I, as we need pole at origin for ess=0, and both PD and PID are not realiziable due to the improper controller effort transfer function

s=tf('s');

%zero location and gain
z = 18;
k = 29;

%controller and system tf
D = (k * (s + z))/s

%% Use to test step response and final rlocus

figure(1); clf;
L = D*G;
rlocus(L)
grid on;
% T = feedback(L,1);
% 
% figure(1); clf;
% step(T);
% axis([0 2.5 0 1.1])


%% Now test to see how paramters change with alpha for same controller

% for i = 0:1:99
%     G = ece414planttf(6,10,(i/100));
%     L = D*G;
%     U = D/(1+L);
%     info = stepinfo(U);
%     peak(i+1) = info.Peak;
%     info = margins(L);
%     PM(i+1) = info.Pm;
%     T = feedback(L,1);
%     info = stepinfo(T);
%     OS(i+1) = info.Overshoot;
%     ts(i+1) = info.SettlingTime;
%     [Smax,Wsp]=PeakSens(L);
%     max(i+1) = Smax;
% end
% 
% i=linspace(0,.99,100);
% figure(1); clf;
% subplot(2,2,1)
% yyaxis left
% plot(i,max)
% ylabel('Peak Sensitivity')
% yyaxis right
% plot(i,ts)
% xlabel('\alpha')
% ylabel('Settling Time (s)')
% title('Peak Sensitivity and Settling Time vs \alpha')
% subplot(2,2,2)
% plot(i,OS)
% xlabel('\alpha')
% ylabel('Overshoot (%)')
% title('%Overshoot vs \alpha')
% subplot(2,2,3)
% plot(i,peak)
% xlabel('\alpha')
% ylabel('Peak Controller Effort')
% title('Peak Controller Effort vs \alpha')
% subplot(2,2,4)
% plot(i,PM)
% xlabel('\alpha')
% ylabel('Phase Margin (\circ)')
% title('Phase Margin vs \alpha')



