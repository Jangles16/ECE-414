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
d.Mc = 0.150;           %nominal cartridge mass (kg)
d.Jc = d.Mc * (d.Rp)^2; %nominal cartridge inertia

%% Find the Nominal plants
% Need to do this for all for motors

% Find Total inertia - motor inertia
d.Jx = (d.Jp)*2 + d.Js + d.Jb + d.Jc;   %The constant part of inertia (only motor inertia change) when fixing Cartridge mass.

for m = 1:4
    motor = MotorNum(m); % Choose one of the four available motors
    % Inertia calculations
    motor.Jsys = motor.Jm + d.Jx;   % Calculate total system inertia
    
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
% torque constant.
d.Mrange = [50,100,150,200,250]./1000;
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
C=20;   % P type to test output plots

%%% will need a method to find controller




%% Calculate Loop TF
L=C*Plant(1).G;


%% Velocity Transfer Function
% The basic plant (input/output at each end of -kt loop repectivly) is Z
Znum=(Plant(1).Ktnom)/(Plant(1).L*Plant(1).Jsys);
Zden =(s^2 + (Plant(1).R/Plant(1).L)*s+(Plant(1).Bm/Plant(1).Jsys)*s+((Plant(1).Ktnom^2 + Plant(1).R*Plant(1).Bm)/(Plant(1).L*Plant(1).Jsys)));
Z=Znum/Zden;

Vel_tf = d.Ks*C*Z*d.Gv*d.Rp/(1+L);


%% System Transfer Function
Sys_tf = L/(1+L);      %whole system (position out)


%% Voltage Transfer Function
Volt_tf = (C*d.Gv)/(1+L);


%% Current Transfer Function
Curr_plant_num = (d.Gv/Plant(1).L)*(s+(Plant(1).Bm/Plant(1).Jsys));
Curr_plant_den = s^2+((Plant(1).R/Plant(1).L)+(Plant(1).Bm/Plant(1).Jsys))*s + ((((Plant(1).Ktnom)^2)+Plant(1).R*Plant(1).Bm)/(Plant(1).L*Plant(1).Jsys));
Curr_tmp = Curr_plant_num/Curr_plant_den;
Curr_tf = (C*Curr_tmp)/(1+L);


%% Position Response v Time
[Pos_act,tref]=lsim(Sys_tf,pos,tfun);
figure(1);clf;
plot(tref,Pos_act)
hold on
plot(tfun,pos)      %variables from spluse function
legend('Actual','Reference')
title('Whole System')


%% Voltage Response v Time
[Volt_act,tref]=lsim(Volt_tf,pos,tfun);
figure(3);clf;
plot(tref,Volt_act)
title('Voltage')


%% Current Response v Time
[Curr_act,tref]=lsim(Curr_tf,pos,tfun);
figure(4);clf;
plot(tref,Curr_act)
title('Current')


%% Velocity Response v Time
[Vel_act,tref]=lsim(Vel_tf,pos,tfun);
figure(2);clf;
plot(tref,Vel_act)
title('better Xdot')


%% Velocity Versus Position
figure(6);clf;
plot(pos*100,Vel_act*100)
hold on
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

%% Power Calculation and Plot

figure(7);clf;
Power_act = abs(Volt_act.*Curr_act);
plot(tref, Power_act)
title('Power')
xlabel('Time (s)')
ylabel('Power (W)')































% figure(1);clf;
% for i = 1:4
%     [y,t] = lsim(Plant(i).G,pos,tfun);
%     plot(t,y)
%     hold on
%     
% end
% legend('1','2','3','4')


