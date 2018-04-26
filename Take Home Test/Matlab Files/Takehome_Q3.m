%% Q3 - PIDsearch Design

G = ece414planttf(6,10,0.45);
Cbase = (37.9*(s+18))/s
C = pidsearch(G, Cbase, 'OS')
L = G*C
T=feedback(L,1)
% figure(2); clf;
% step(T)




% for i = 0:1:99
%     G = ece414planttf(6,10,(i/100));
%     L = C*G;
%     U = C/(1+L);
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