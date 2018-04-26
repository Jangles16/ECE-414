%% ECE 414 Homework #3 - Josh Andrews

%% Question 1
% First gather the original system data for Wn = 100 and Z = 0.7
%%
% Clean everything up before we start

clear;
clc;

%%
% First Set Wn to initial value of 100 and Z to 0.7
Wn = 100;
Z = 0.7;
%% 
% find transfer function
H(1)=tf(Wn.^2, [1 2*Z*Wn Wn.^2]);
Hinfo(1) = stepinfo(H(1));

%%
% plot it
figure(1); clf;
step(H(1));
hold on;
disp(Hinfo(1));

%% 
% Now






