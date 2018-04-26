%% Josh Andrews
% HW 2

% Start off fresh

clear all;
clc;
clf;

%% Question 1
% set Z = 0.7 and vary Wn from 10 to 50 to see how the specifications vary
Z= 0.7;
t=linspace(0,1,.01);
for i = 10:10:50
% Vary value of Wn
    Wn = i;
% Now create the transfer function based on variables above
% Divide by 10 to index 1-5
    H(i/10) = tf(Wn.^2, [1 2*Z*Wn Wn.^2]);
    hinf(i/10) = stepinfo(H(i/10));
    rise(i/10) = hinf(i/10).RiseTime;
end

% And finally plot step response for each Wn
figure(1);
step(H(1),H(2),H(3),H(4),H(5))
hold on;
figure(4);
plot(t,rise(1),t,rise(2), t,rise(3),t,rise(4),t,rise(5))
legend('Wn=10','Wn=20','Wn=30','Wn=40','Wn=50');
hold on;

% The step response info for Wn=10 and Wn=50
disp(stepinfo(H(1)));
disp(stepinfo(H(5)));

% Wn seems to only affect the timing of the system.  A higher Wn results in
% a faster rise time, settle time, and peak time.  All other parameters
% remained constant

%% Question 2
% set Wn=50 and vary Z from 0.1 to 0.9
Wn = 50;

for i = 1:1:9  % integer values required
% Divide to get proper Z value 
    Z = i/10; 
% Calculate the transfer function    
    H(i) = tf(Wn.^2, [1 2*Z*Wn Wn.^2]);
end
% And finally plot step response for each Z
figure(2);
step(H(1),H(2),H(3),H(4),H(5),H(6),H(7),H(8),H(9))
legend('Z=0.1','Z=0.2','Z=0.3','Z=0.4','Z=0.5','Z=0.6','Z=0.7','Z=0.8','Z=0.9')

% The step response info for Z=0.1,0.5, and 0.9
disp(stepinfo(H(1)));
disp(stepinfo(H(5)));
disp(stepinfo(H(9)));

% Varying Z changes all but the undershoot parameter.  Increasing the value
% of Z results in less overshoot and a shorter settling time but increases
% the rise time of the system

%% Question 3
% vary Z from 0.1 to 0.9 and keep Z*Wn = 10 for each case

for i = 1:1:9  % integer values required
% Divide
    Z = i/10;
    Wn = 10/Z;    
    H(i) = tf(Wn.^2, [1 2*Z*Wn Wn.^2]);
end
figure(3);
step(H(1),H(2),H(3),H(4),H(5),H(6),H(7),H(8),H(9))
legend('Z=0.1','Z=0.2','Z=0.3','Z=0.4','Z=0.5','Z=0.6','Z=0.7','Z=0.8','Z=0.9')

% The step response info for Z=0.1,0.5, and 0.9
disp(stepinfo(H(1)));
disp(stepinfo(H(5)));
disp(stepinfo(H(9)));

%% Question 4
% compare calculations and approximations