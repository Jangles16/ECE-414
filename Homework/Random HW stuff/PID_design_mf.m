%PID controller design for 
clear all
close all
clc

%% Model parameters

M    = 1;
D    = 1;
Ks   = 1;

G    = tf([1],[M D Ks]);

%% Design specifications

Ts   = 3.47;                                    %Ts<3.5s
PO   = 5;                                       %PO<10
zeta = -log(PO/100)/(pi^2+(log(PO/100))^2)^0.5; %damping ratio
wn   = 4/Ts/zeta;                               %natural frequency


%% Model free PID controller
%First attempt
% kp   = 5;  
% kd   = 0;
% ki   = 0;

% %Second attempt
% kp   = 1;  
% kd   = 1;
% ki   = 2;
% 
% % %Third attempt
% kp   = 1;  
% kd   = 2;
% ki   = 2;
% 
% % %Fourth attempt
% kp   = 1;  
% kd   = 3;
% ki   = 2;
% 
% %Fifth attempt
% kp   = 5;  
% kd   = 3;
% ki   = 2;
% 
% 
% %Final tuning
kp   = 5;
kd   = 3.1;
ki   = 4;
% % 
% %K    = tf([kd kp ki],[1 0])
% 
% %K     = kp;
% 
K_mf    = kp+ kd*tf([1 0],1)+ki*tf(1,[1 0]);

% Closed loop system

T    = feedback(G*K_mf,1,-1);

U    = feedback(K_mf,G,-1)*tf(1,[1 1]);

E    = feedback(1,K_mf*G,-1);

%% Open loop system

figure(1);
step(G)


%% Step response of the closed loop system

figure(2);
step(T);

%% Controller effort
figure(3);
step(U)

%% Steady state error
figure(4);
step(E)