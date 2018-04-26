%% Question 4 - Unity LAM takehome test

% First find nominal plant, will use alpha = 0.5
clear all; clc;
G=ece414planttf(6,10,0.5)
%G=tf(G)
s=tf('s');
N=26.97;
Ds = s^3 + 29.2*s^2 + 453.3*s + 2779
Dneg = -s^3 + 29.2*s^2 - 453.3*s + 2779
% now we need to find Do for the lamdesign function, which means find Qs
% Q=D(s)*D(-s)+qN(s)N(-s) where G=N/D
% DnegS = -s^3 + 29.2*s^2 - 453.3*s + 2779
% %and N(s) = N(-s) = 26.972
% N = 26.97;
% % Now we need to pick an arbitrary q
% %and compute Q(s)
% %q = 100;
q = 100;
Dd=Ds*Dneg
Q = Dd + q*N^2
% %and writing as vector     
r = roots(Q.Numerator{1,1})
% % take the neg real roots 
Do = poly([r(1) r(2) r(3) r(1) r(2) r(3)])
[D,T,Tu,Td,L]=lamdesign(G,Do);
% figure(4); clf;
% subplot(1,2,1)
% step(T);
% subplot(1,2,2)
% step(Tu);
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
