%% ECE 414 - Final Project
% Written by Josh Andrews, Riley McKay

%% Step One, Get the TF's in here and other variables needed
% Need N,D,C equations, and most parameters (make data structure?)
clear all; clc;
s = tf('s');

% Make structure containing constant overall parameters (ie no motor stuff)
d.Jp = 6.2e-6;          %pulley inertia
d.Js = 1.4e-6;          %position sensor inertia 
d.Gv = 5;               %Voltage amplifier gain (V/V)
d.Ks = 25;              %position sensor gain (V/m)
d.Rp = 0.0075;          %pulley radius (m)
d.Jb = 1.0125e-6;       %belt inertia
d.Mc = [0.050 0.150 0.250];           %nominal cartridge mass (kg)
d.Jc = d.Mc * (d.Rp)^2; %nominal cartridge inertia

for i = 1:3
    
    d.Jx(i) = (d.Jp)*2 + d.Js + d.Jb + d.Jc(i);   %The constant part of inertia (only motor inertia change) when fixing Cartridge mass.
end

%% Find the Nominal plants
% Need to do this for all for motors

for m = 1:4
    
    motor = MotorNum(m); % Choose one of the four available motors
    % Inertia calculations
    motor.Jsys = motor.Jm + d.Jx(2);   % Calculate total system inertia
    
    motor.Numerator = (d.Gv * motor.Ktnom * d.Ks * d.Rp)/(motor.L * motor.Jsys);
    motor.sOneTerm = (motor.R/motor.L)+(motor.Bm/motor.Jsys);
    motor.sZeroTerm = (((motor.Ktnom)^2)+ motor.R * motor.Bm)/(motor.L * motor.Jsys);

    % Calculate plant transfer function
    motor.G = motor.Numerator/(s*(s^2 + motor.sOneTerm * s + motor.sZeroTerm)); % Calculate plant transfer function
    
    % Save the data so we have it for later
    Plant(m) = motor;
    % Plot Root Locus of plant transfer function
%     figure(m+10); clf;
%     rlocus(Plant(m).G);
end


%% Step Two, Do Root Locus of Plants as Parameters Change
% Plot change in poles as Kt and Mi change.  Mi=cartridge mass, Kt = motor
% torque constant. Will have 3 variations of each parameter (Km and Mi)

%%% Still Need to do (maybe)  %%%



%% Our Reference Input
% refer to diagram in notebook, 5/4/18 pg. 2
T = [0 .08 .59 .67];
F = 0.44;
Type = 1;
[fun,dfun,ifun] = spulse(T,F,Type);

tfun = 0:0.0001:0.9;
pos = ifun(tfun);
vel = fun(tfun);
acc = dfun(tfun);


%% Test Stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(10);clf;
% plot(tfun,vel);
% figure(11);clf;
% plot(tfun,pos);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Controller
% need to find out which plant setup to optimize for, might actually be several we
% have to look at. I'm thinking right now that Motor 1 with full ink and
% high inertia will be the slowest to respond.

C=20;   % P type to test output plots

%%% will need a method to find controller
%%% Step is actually faster than ours, so need to experiment.  Also want to
%%% keep phase margin high.




%% Calculate Loop TF
for i=1:4
    L=C*Plant(i).G;


    %% Velocity Transfer Function
    % The basic plant (input/output at each end of -kt loop repectivly) is Z
    Znum=(Plant(i).Ktnom)/(Plant(i).L*Plant(i).Jsys);
    Zden =(s^2 + (Plant(i).R/Plant(i).L)*s+(Plant(i).Bm/Plant(i).Jsys)*s+((Plant(i).Ktnom^2 + Plant(i).R*Plant(i).Bm)/(Plant(i).L*Plant(i).Jsys)));
    Z=Znum/Zden;

    Plant(i).Vel_tf = d.Ks*C*Z*d.Gv*d.Rp/(1+L);


    %% System Transfer Function
    Plant(i).Sys_tf = L/(1+L);      %whole system (position out)


    %% Voltage Transfer Function
    Plant(i).Volt_tf = (C*d.Gv)/(1+L);


    %% Current Transfer Function
    Curr_plant_num = (d.Gv/Plant(i).L)*(s+(Plant(i).Bm/Plant(i).Jsys));
    Curr_plant_den = s^2+((Plant(i).R/Plant(i).L)+(Plant(i).Bm/Plant(i).Jsys))*s + ((((Plant(i).Ktnom)^2)+Plant(i).R*Plant(i).Bm)/(Plant(i).L*Plant(i).Jsys));
    Curr_tmp = Curr_plant_num/Curr_plant_den;
    Plant(i).Curr_tf = (C*Curr_tmp)/(1+L);
end


%% Position Response v Time
% figure(1);clf;
% for i=1:4
%     [Pos_act,tref]=lsim(Plant(i).Sys_tf,pos,tfun);
%     plot(tref,Pos_act)
%     hold on
%     
% end
% plot(tfun,pos)      %variables from spluse function
% legend('1','2','3','4','ref')
% title('Whole System')

% %% Voltage Response v Time
% [Volt_act,tref]=lsim(Volt_tf,pos,tfun);
% figure(3);clf;
% plot(tref,Volt_act)
% title('Voltage')
% 
% 
% %% Current Response v Time
% [Curr_act,tref]=lsim(Curr_tf,pos,tfun);
% figure(4);clf;
% plot(tref,Curr_act)
% title('Current')
% 
% 
% %% Velocity Response v Time

% figure(2);clf;
% plot(tref,Vel_act)
% title('better Xdot')
% 
% 
%% Velocity Versus Position
figure(6);clf;
for i=1:4
    [Vel_act,tref]=lsim(Plant(i).Vel_tf,pos,tfun);
    plot(pos*100,Vel_act*100)
    hold on
end
legend('1','2','3','4')
plot([2 2],ylim,'--');      % 2cm edge
hold on
plot([24 24],ylim,'--');    % 24cm edge
hold on
plot([26 26],ylim,'--')     % opp. pos limit
hold on
plot(xlim,[0.1 0.1],'--')   % max speed when hitting 26cm
hold on
plot(pos*100,vel*100,'r')   %% referece vel v pos
hold on
plot(xlim,[1.0001*44 1.0001*44])
hold on
plot(xlim,[.9999*44 .9999*44])
hold off
title('Velocity vs Position')
xlabel('Position (cm)')
ylabel('Velocity (cm/s)')
xlim([0 27])
% 
% %% Power Calculation and Plot
% % probably don't have to plot this one, just need the max
% 
% figure(7);clf;
% Power_act = abs(Volt_act.*Curr_act);
% plot(tref, Power_act)
% title('Power')
% xlabel('Time (s)')
% ylabel('Power (W)')
% 
% %% There are a few more plots requested
% 
% 
% 
% %% 































% figure(1);clf;
% for i = 1:4
%     [y,t] = lsim(Plant(i).G,pos,tfun);
%     plot(t,y)
%     hold on
%     
% end
% legend('1','2','3','4')


